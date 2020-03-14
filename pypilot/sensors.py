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

# favor lower priority sources
source_priority = {'gpsd' : 1, 'servo': 1, 'serial' : 2, 'tcp' : 3, 'signalk' : 4, 'none' : 5}

class Sensor(object):
    def __init__(self, client, name):
        self.source = client.register(StringValue(name + '.source', 'none'))
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
            print('found', self.name, 'on', source, data['device'])
            self.source.set(source)
            self.device = data['device']
        self.lastupdate = time.monotonic()
        
        return True

    def reset(self):
        raise 'reset should be overloaded'

    def update(self, data):
        raise 'update should be overloaded'

    def register(self, _type, name, *args, **kwargs):
        return self.client.register(_type(*([self.name + '.' + name] + list(args)), **kwargs))

class Wind(Sensor):
    def __init__(self, client):
        super(Wind, self).__init__(client, 'wind')

        self.direction = self.register(SensorValue, 'direction', directional=True)
        self.speed = self.register(SensorValue, 'speed')
        self.offset = self.register(RangeSetting, 'offset', 0, -180, 180, 'deg')

    def update(self, data):
        if 'direction' in data:
            # direction is from -180 to 180
            self.direction.set(resolv(data['direction'] + self.offset.value))
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
        self.xte.update(data['xte'])

        if not self.client.values['ap.enabled'].value:
            return

        mode = self.client.values['ap.mode'].value
        if mode.value != data['mode']:
            # for GPAPB, ignore message on wrong mode
            if data['isgp'] != 'GP':
                mode.set(data['mode'])
            else:
                return 
                # APB is from GP with no gps mode selected so exit

        command = data['track'] + self.gain.value*data['xte']

        heading_command = self.server.values['ap.heading_command']
        if abs(heading_command.value - command) > .1:
            heading_command.set(command)

class gps(Sensor):
    def __init__(self, client):
        super(gps, self).__init__(client, 'gps')
        self.track = self.register(SensorValue, 'track', directional=True)
        self.speed = self.register(SensorValue, 'speed')
        self.latitude = self.register(SensorValue, 'latitude', fmt='%.11f')
        self.longitude = self.register(SensorValue, 'longitude')

    def update(self, data):
        self.track.set(data['track'])
        self.speed.set(data['speed'])
        if 'latitude' in data and 'longitude' in data:
            self.latitude.set(data['latitude'])
            self.longitude.set(data['longitude'])

    def reset(self):
        self.track.set(False)
        self.speed.set(False)

class Sensors(object):
    def __init__(self, client):
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
        self.wind = Wind(client)
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
            
    def write(self, sensor, data, source):
        if not sensor in self.sensors:
            print('unknown data parsed!', sensor)
            return
        self.sensors[sensor].write(data, source)

    def lostdevice(self, device):
        # optional routine  useful when a device is
        # unplugged to skip the normal data timeout
        for name in self.sensors:
            sensor = self.sensors[name]
            if sensor.device and sensor.device[2:] == device:
                self.lostsensor(sensor)
