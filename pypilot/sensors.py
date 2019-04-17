#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# autopilot base handles reading from the imu (boatimu)

from signalk.values import *
from resolv import resolv

# favor lower priority sources
source_priority = {'gpsd' : 1, 'servo': 1, 'serial' : 2, 'tcp' : 3, 'signalk' : 4, 'none' : 5}

class SensorTrackSignalk(SensorValue): # same as Value with added timestamp
    def __init__(self, name, sensor, timestamp, initial=False, **kwargs):
        super(SensorValue, self).__init__(name, initial, **kwargs)
        self.sensor = sensor
        self.client_can_set = True

    def Register(self, _type, name, *args, **kwargs):
        return self.server.Register(_type(*(['wind.' + name] + list(args)), **kwargs))

    def set_sensor(self, value):
        super(SensorValue, self).set(value)

    def set(self, value):
        # set by signalk
        if self.sensor.set(value):
            self.set_sensor(value)

class Sensor(object):
    def __init__(self, server, name):
        self.source = server.Register(StringValue(name + '.source', 'none'))
        self.lastupdate = 0
        self.device = None
        self.name = name
        self.server = server
        self.starttime = time.time()
            
    def write(self, data, source):
        if source_priority[self.source.value] < source_priority[source]:
            return False               

        # if there are more than one device for a source at the same priority,
        # we only use data from one rather than randomly switching between the two
        if source_priority[self.source.value] == source_priority[source] and \
           data['device'] != self.device:
            return False

        timestamp = data['timestamp'] if 'timestamp' in data else time.time()-self.starttime
        self.server.TimeStamp(self.name, timestamp)
        self.update(data)
                
        if self.source.value != source:
            print 'found', self.name, 'on', source, data['device']
            self.source.set(source)
            self.device = data['device']
        self.lastupdate = time.time()
        
        return True

    def update(self, data):
        raise 'update should be overloaded'

    def Register(self, _type, name, *args, **kwargs):
        return self.server.Register(_type(*([self.name + '.' + name] + list(args)), **kwargs))

class Wind(Sensor):
    def __init__(self, server):
        super(Wind, self).__init__(server, 'wind')

        timestamp = server.TimeStamp('wind')
        self.direction = self.Register(SensorValue, 'direction', timestamp, directional=True)
        self.speed = self.Register(SensorValue, 'speed', timestamp)
        self.offset = self.Register(RangeProperty, 'offset', 0, -180, 180, persistent=True)

    def update(self, data):
        self.direction.set(resolv(data['direction'] + self.offset.value, 180))
        if 'speed' in data:
            self.speed.set(data['speed'])
        
class Sensors(object):
    def __init__(self, server):
        from gpsd import Gpsd
        from rudder import Rudder
        from nmea import Nmea
        
        self.server = server
        self.nmea = Nmea(server, self)
        self.gps = Gpsd(server, self)
        self.wind = Wind(server)
        self.rudder = Rudder(server)

        self.sensors = {'gps': self.gps, 'wind': self.wind, 'rudder': self.rudder}

    def poll(self):
        self.gps.poll()
        self.nmea.poll()
        self.rudder.poll()

        # timeout sources
        t = time.time()
        for name in self.sensors:
            sensor = self.sensors[name]
            if sensor.source.value == 'none':
                continue
            if t - sensor.lastupdate > 8:
                print 'sensor timeout for', name, 'source', sensor.source.value, t - sensor.lastupdate
                sensor.source.set('none')
                sensor.device = None
                break

    def write(self, sensor, data, source):
        if not sensor in self.sensors:
            print 'unknown data parsed!', sensor
            return

        self.sensors[sensor].write(data, source)

    def lostdevice(self, device):
        # optional routine  useful when a device is
        # unplugged to skip the normal data timeout
        print 'sensor lost device', device
        for name in self.sensors:
            sensor = self.sensors[name]
            if sensor.device[2:] == device:
                sensor.source.set('none')
                sensor.device = None
