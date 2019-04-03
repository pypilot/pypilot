#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import multiprocessing, time, socket
from signalk.pipeserver import NonBlockingPipe
import select

from signalk.values import *
import serialprobe
from sensors import Sensor

class GpsProcess(multiprocessing.Process):
    def __init__(self):
        # split pipe ends
        self.pipe, pipe = NonBlockingPipe('gpsprocess')
        super(GpsProcess, self).__init__(target=self.gps_process, args=(pipe, ))
        self.devices = []

    def connect(self):
        while True: # connection to gpsd loop
            try:
                import gps
                self.gpsd = gps.gps(mode=gps.WATCH_ENABLE) #starting the stream of info
                self.gpsd.next() # flush initial message
                print 'connected to gpsd'
                return
            except socket.error:
                time.sleep(3)

    def read(self, pipe):
        lasttime = time.time()
        while True:
            try:
                gpsdata = self.gpsd.next()
                device = None
                if 'device' in gpsdata:
                    device = gpsdata['device']
                    if not device in self.devices:
                        pipe.send({'device': device})
                        self.devices.append(device)

                if self.gpsd.fix.mode == 3 and \
                   time.time() - lasttime > .25:
                    fix = {}
                    #fix['time'] = self.gpsd.fix.time
                    fix['track'] = self.gpsd.fix.track
                    fix['speed'] = self.gpsd.fix.speed * 1.944 # knots
                    fix['device'] = device
                    pipe.send(fix, False)
                    lasttime = time.time()

            except StopIteration:
                print 'lost connection to gpsd'
                break

    def gps_process(self, pipe):
        import os
        #print 'gps on', os.getpid()
        while True:
            self.connect()
            self.read(pipe)
            
class Gpsd(Sensor):
    def __init__(self, server, sensors):
        super(Gpsd, self).__init__(server, 'gps')

        timestamp = server.TimeStamp('gps')
        self.track = self.Register(SensorValue, 'track', timestamp, directional=True)
        self.speed = self.Register(SensorValue, 'speed', timestamp)
        
        self.sensors = sensors

        self.process = False
        self.devices = []

        self.process = GpsProcess()
        self.process.start()
        READ_ONLY = select.POLLIN | select.POLLHUP | select.POLLERR
        self.poller = select.poll()
        self.poller.register(self.process.pipe.fileno(), READ_ONLY)
        self.fd = self.process.pipe.fileno()

    def read(self):
        fix = self.process.pipe.recv()

        if 'device' in fix:
            device = fix['device']
            if not device in self.devices:
                self.devices.append(device)
                print 'gpsd is using device', device
                serialprobe.reserve(device)
                return

        def fval(name):
            return fix[name] if name in fix else False

        # use fix timestamp??
        val = {'timestamp' : time.time(), 'track': fval('track'), 'speed': fval('speed'), 'device': fval('device')}
        self.sensors.write('gps', val, 'gpsd')

    def poll(self):
        while True:
            events = self.poller.poll(0)
            if not events:
                break
            while events:
                event = events.pop()
                fd, flag = event
                if fd == self.fd:
                    if flag != select.POLLIN:
                        print 'nmea got flag for gpsd pipe:', flag
                    else:
                        self.read()


    def update(self, data):
        self.track.set(data['track'])
        self.speed.set(data['speed'])
        

if __name__ == '__main__':
    import gps
    gpsd = gps.gps(mode=gps.WATCH_ENABLE)
    while True:
        print gpsd.next()
        print 'fix:', gpsd.fix.mode, gpsd.fix.track, gpsd.fix.speed
