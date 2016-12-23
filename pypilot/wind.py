#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time, serial

from signalk.values import *

'''
   ** MWV - Wind Speed and Angle
   **
   **
   **
   ** $--MWV,x.x,a,x.x,a*hh<CR><LF>**
   ** Field Number:
   **  1) Wind Angle, 0 to 360 degrees
   **  2) Reference, R = Relative, T = True
   **  3) Wind Speed
   **  4) Wind Speed Units, K/M/N
   **  5) Status, A = Data Valid
   **  6) Checksum
'''
def parse_nmea(line):
    if line[0:1] != '$' or line[3:6] != 'MWV':
        return False

    nline = ""
    i = 1
    while i < len(line):
        c = line[i]
        i+=1
        if c=='*':
            break
        nline += c

    try:
        cksum = line[i] + line[i+1];
        cksum = int(cksum, 16)
    except:
        return False

    comp_cksum = 0
    for c in nline:
        comp_cksum ^= ord(c)

    if cksum != comp_cksum:
        print 'wind sensor cksum failed'
        return False
        
    data = line.split(',')
    return {'direction': float(data[1]), 'speed': float(data[3])}


class NMEAWindSensor():
    def __init__(self, device):
#        if device[0] == '/dev/ttyUSB1':
#            raise 'die'
        self.device = serial.Serial(*device)
        self.device.timeout = 0 # nonblocking

    def poll(self):
        while True:
            line = self.device.readline()
            if not line:
                break
            return parse_nmea(line.rstrip())

class Wind():
    def __init__(self, server, serialprobe):
        self.driver = False
        self.server = server
        self.serialprobe = serialprobe
        self.timestamp = time.time()
        self.direction = self.Register(SensorValue, 'direction', self)
        self.speed = self.Register(SensorValue, 'speed', self)

    def Register(self, _type, name, *args):
        return self.server.Register(_type(*(['wind/' + name] + list(args))))

    def poll(self):
        if not self.driver:
            device = self.serialprobe.probe('wind', [38400, 4800])
            if device:
                try:
                    self.driver = NMEAWindSensor(device)
                    self.direction.timestamp = time.time()
                except serial.serialutil.SerialException:
                    print 'failed to open', device

        if self.driver:
            winddata = False
            while True:
                try:
                    val = self.driver.poll()
                except:
                    self.driver = False
                    break
                if not val:
                    break
                winddata = val

            if winddata:
                self.serialprobe.probe_success('wind')
                self.timestamp = time.time()
                self.direction.set(winddata['direction'])
                self.speed.set(winddata['speed'])
                return True
                
            if time.time() - self.direction.timestamp > 5:
                self.driver = False
        return False
                

if __name__ == "__main__":
    from signalk.server import SignalKServer
    server = SignalKServer()
    wind = Wind(server)
    count = 0;
    while True:
        if wind.poll():
            print 'Wind demo', count, wind.direction.value
        count += 1
        time.sleep(.1)
