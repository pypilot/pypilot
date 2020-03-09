#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from __future__ import print_function
import multiprocessing, time, socket, select
from pipeserver import NonBlockingPipe
from values import *
import serialprobe

class gpsProcess(multiprocessing.Process):
    def __init__(self):
        # split pipe ends
        self.pipe, pipe = NonBlockingPipe('gps_pipe')
        super(gpsProcess, self).__init__(target=self.gps_process, args=(pipe,))
        self.devices = []

    def connect(self):
        while True: # connection to gpsd loop
            try:
                import gps
                self.gpsd = gps.gps(mode=gps.WATCH_ENABLE) #starting the stream of info
                self.gpsd.next() # flush initial message
                print('connected to gpsd')
                return
            except socket.error:
                time.sleep(3)

    def read(self, pipe):
        lasttime = time.monotonic()
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
                   time.monotonic() - lasttime > .25:
                    fix = {}
                    #fix['time'] = self.gpsd.fix.time
                    fix['track'] = self.gpsd.fix.track
                    fix['speed'] = self.gpsd.fix.speed * 1.944 # knots
                    fix['device'] = device
                    pipe.send(fix, False)
                    lasttime = time.monotonic()

            except StopIteration:
                print('lost connection to gpsd')
                break

    def gps_process(self, pipe):
        import os
        #print('gps on', os.getpid())
        while True:
            self.connect()
            self.read(pipe)
            
class gpsd(Sensor):
    def __init__(self, ap):
        super(gpsd, self).__init__(ap, 'gps')
        self.track = self.register(SensorValue, 'track', directional=True)
        self.speed = self.register(SensorValue, 'speed')

        self.process = False
        self.devices = []

        self.process = gpsProcess()

        self.process.start()
        READ_ONLY = select.POLLIN | select.POLLHUP | select.POLLERR
        self.poller = select.poll()
        self.poller.register(self.process.pipe.fileno(), READ_ONLY)
        self.fd = self.process.pipe.fileno()

    def read(self):
        fix = self.process.pipe.recv()
        if 'device' in fix:
            device = fix['device']
            if device and not device in self.devices:
                self.devices.append(device)
                print('gpsd is using device', device)
                serialprobe.reserve(device)
                return

        def fval(name):
            return fix[name] if name in fix else False

        # use fix timestamp?
        val = {'track': fval('track'), 'speed': fval('speed'), 'device': fval('device')}
        self.ap.sensors.write('gps', val, 'gpsd')

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
                        print('nmea got flag for gpsd pipe:', flag)
                    else:
                        self.read()

    def update(self, data):
        self.track.set(data['track'])
        self.speed.set(data['speed'])

    def reset(self):
        self.track.set(False)
        self.speed.set(False)
        

if __name__ == '__main__':
    import gps
    gpsd = gps.gps(mode=gps.WATCH_ENABLE)
    while True:
        print(gpsd.next())
        print('fix:', gpsd.fix.mode, gpsd.fix.track, gpsd.fix.speed)
