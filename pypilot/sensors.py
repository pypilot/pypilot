#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from client import *
from values import *
from resolv import resolv

from gpsd import gpsd

import datetime
import math

# favor lower priority sources
source_priority = {'gpsd' : 1, 'servo': 1, 'serial' : 2, 'tcp' : 3, 'signalk' : 4, 'none' : 5}

class Sensor(object):
    def __init__(self, client, name):
        self.source = client.register(StringValue(name + '.source', 'none'))
        if name != 'apb':
            self.rate = client.register(RangeProperty(name + '.rate', 4, 0, 50))
        self.lastupdate = 0
        self.device = None
        self.name = name
        self.client = client
            
    def write(self, data, source):
        if source_priority[self.source.value] < source_priority[source]:
            return False               

        # if there are more than one device for a source at the same priority,
        # we only use data from one rather than randomly switching between the two
        if source_priority[self.source.value] == source_priority[source] and \
           data['device'] != self.device:
            return False

        #timestamp = data['timestamp'] if 'timestamp' in data else time.monotonic()-self.starttime
        self.update(data)
                
        if self.source.value != source:
            print(_('sensor found'), self.name, source, data['device'])
            self.source.set(source)
            self.device = data['device']
        self.lastupdate = time.monotonic()
        
        return True

    def getddmmyy(self):
        today = datetime.date.today()
        return '%02d%02d%02d' % (today.day, today.month, (today.year%100))

    def gethhmmss(self):
        t = datetime.datetime.now()
        return t.strftime("%H%M%S.%f")[:-4]

    def getddmmmmmm(self, degrees, n, s):
        minutes = (abs(degrees) - abs(int(degrees))) * 60
        return '%02d%07.4f,%c' % ( abs(degrees), minutes, n if degrees > 0 else s)

    def getrmc(self):
        return 'APRMC,' + self.gethhmmss() + ',A,' \
            + self.getddmmmmmm(self.lat.value, 'N', 'S') + ',' \
            + self.getddmmmmmm(self.lon.value, 'E', 'W') \
            + ',%.2f' % self.speed.value + ',%.2f,' % self.track.value + ',' \
            + self.getddmmyy() + ',' + ',' + ',A'

    def reset(self):
        raise 'reset should be overloaded'

    def update(self, data):
        raise 'update should be overloaded'

    def register(self, _type, name, *args, **kwargs):
        return self.client.register(_type(*([self.name + '.' + name] + list(args)), **kwargs))

class Wind(Sensor):
    def __init__(self, client, boatimu):
        super(Wind, self).__init__(client, 'wind')

        self.boatimu = boatimu

        self.direction = self.register(SensorValue, 'direction', directional=True)
        self.speed = self.register(SensorValue, 'speed')
        self.offset = self.register(RangeSetting, 'offset', 0, -180, 180, 'deg')
        self.compensation_height = self.register(RangeProperty, 'sensors_height', 0, 0, 100, persistent=True)

    def update(self, data):
        if 'direction' in data:
            data['direction'] += self.offset.value

        if self.compensation_height.value and 'direction' in data and 'speed' in data and self.boatimu:
            # use imu boat motion to compensate wind reading
            # compute apparent wind at sensor height from pitch and roll rates
            # for this to work, the heading alighment must be correct
            # for now just use roll an pitch, eventually also accelerations?

            # convert input data into rectangular with y along boat forward direction
            speed = data['speed']
            direction = data['direction']
            dx = speed*math.sin(math.radians(direction))
            dy = speed*math.cos(math.radians(direction))

            m = math.radians(self.compensation_height.value)
            dx -= m * self.boatimu.SensorValues['rollrate'] # is this positive or negative?!?
            dy -= m * self.boatimu.SensorValues['pitchrate']

            data['speed'] = math.hypot(dx, dy)
            data['direction'] = math.degrees(math.atan2(dx, dy))

        if 'direction' in data:
            # direction is from -180 to 180
            self.direction.set(resolv(data['direction']))
        if 'speed' in data:
            self.speed.set(data['speed'])

    def reset(self):
        self.direction.set(False)
        self.speed.set(False)

class APB(Sensor):
    def __init__(self, client):
        super(APB, self).__init__(client, 'apb')
        self.track = self.register(SensorValue, 'track', directional=True)
        self.xte = self.register(SensorValue, 'xte')
        # 300 is 30 degrees for 1/10th mile
        self.gain = self.register(RangeProperty, 'xte.gain', 300, 0, 3000, persistent=True)
        self.last_time = time.monotonic()

    def reset(self):
       self.xte.update(0)

    def update(self, data):
        t = time.monotonic()
        if t - self.last_time < .5: # only accept apb update at 2hz
            return

        self.last_time = t
        self.track.update(data['track'])
        if 'xte' in data:
            xte = data['xte']
            self.xte.update(xte)
        else:
            xte = 0

        # ignore message if autopilot is not enabled
        if not self.client.values.values['ap.enabled'].value:
            return

        data_mode = data['mode'] if 'mode' in data else 'gps'
        mode = self.client.values.values['ap.mode']
        if mode.value != data_mode:
            # for GPAPB, ignore message on wrong mode
            if 'senderid' in data and data['senderid'] != 'GP':
                mode.set(data_mode)
            else:
                return 
                # APB is from GP with no gps mode selected so exit

        command = data['track'] + self.gain.value*xte
        #print('apb command', command, data)

        heading_command = self.client.values.values['ap.heading_command']
        if abs(heading_command.value - command) > .1:
            heading_command.set(command)

class gps(Sensor):
    def __init__(self, client):
        super(gps, self).__init__(client, 'gps')
        self.track = self.register(SensorValue, 'track', directional=True)
        self.speed = self.register(SensorValue, 'speed')
        self.lat = self.register(SensorValue, 'lat', fmt='%.11f')
        self.lon = self.register(SensorValue, 'lon', fmt='%.11f')
        self.rate.set(1.0)

    def update(self, data):
        self.speed.set(data['speed'])
        if 'track' in data:
            self.track.set(data['track'])
        if 'lat' in data and 'lon' in data:
            self.lat.set(data['lat'])
            self.lon.set(data['lon'])

    def reset(self):
        self.track.set(False)
        self.speed.set(False)

class Sensors(object):
    def __init__(self, client, boatimu):
        from rudder import Rudder
        from nmea import Nmea
        from signalk import signalk

        self.client = client

        # services that can receive sensor data
        self.nmea = Nmea(self)
        self.signalk = signalk(self)
        self.gpsd = gpsd(self)

        # actual sensors supported
        self.gps = gps(client)
        self.wind = Wind(client, boatimu)
        self.rudder = Rudder(client)
        self.apb = APB(client)

        self.sensors = {'gps': self.gps, 'wind': self.wind, 'rudder': self.rudder, 'apb': self.apb}

    def poll(self):
        self.nmea.poll()
        self.signalk.poll()
        self.gpsd.poll()
        self.rudder.poll()

        # timeout sources
        t = time.monotonic()
        for name in self.sensors:
            sensor = self.sensors[name]
            if sensor.source.value == 'none':
                continue
            if t - sensor.lastupdate > 8:
                self.lostsensor(sensor)

    def lostsensor(self, sensor):
        print('sensor', sensor.name, 'lost', sensor.source.value, sensor.device)
        sensor.source.set('none')
        sensor.reset()
        sensor.device = None

    def lostgpsd(self):
        if self.gps.source.value == 'gpsd':
            self.lostsensor(self.gps)
            
    def write(self, sensor, data, source):
        if not sensor in self.sensors:
            print(_('unknown data parsed!'), sensor)
            return
        self.sensors[sensor].write(data, source)
        
    def lostdevice(self, device):
        # optional routine  useful when a device is
        # unplugged to skip the normal data timeout
        for name in self.sensors:
            sensor = self.sensors[name]
            if sensor.device and sensor.device[2:] == device:
                self.lostsensor(sensor)
