#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import multiprocessing, time, socket, select
from nonblockingpipe import NonBlockingPipe
from values import *
import serialprobe

class gpsProcess(multiprocessing.Process):
    def __init__(self):
        # split pipe ends
        self.pipe, pipe = NonBlockingPipe('gps_pipe', True)
        super(gpsProcess, self).__init__(target=self.gps_process, args=(pipe,), daemon=True)
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
                continue
            except Exception as e:
                print('failed to load gps module', e)
            time.sleep(600)

    def read(self, pipe):
        lasttime = time.monotonic()
        while True:
            try:
                gpsdata = self.gpsd.next()
                device = None
                if 'device' in gpsdata:
                    device = gpsdata['device']
                    if not device in self.devices:
                        #pipe.send({'device': device})
                        self.devices.append(device)

                if self.gpsd.fix.mode == 3 and \
                   time.monotonic() - lasttime > .25:
                    fix = {}
                    #fix['time'] = self.gpsd.fix.time
                    fix['track'] = self.gpsd.fix.track
                    fix['speed'] = self.gpsd.fix.speed * 1.944 # knots
                    fix['latitude'] = self.gpsd.fix.latitude
                    fix['longitude'] = self.gpsd.fix.longitude
                    fix['device'] = device
                    pipe.send(fix, False)
                    lasttime = time.monotonic()

            except StopIteration:
                print('gpsd lost connection')
                break
            except Exception as e:
                print('gpsd unhandled exception', e)
                break

    def gps_process(self, pipe):
        print('gps process', os.getpid())
        while True:
            self.connect()
            self.read(pipe)


class gpsd(object):
    def __init__(self, sensors):
        self.sensors = sensors
        self.devices = [] # list of devices used

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

        self.sensors.write('gps', fix, 'gpsd')

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
                        print('got flag for gpsd pipe:', flag)

                    else:
                        self.read()
            
if __name__ == '__main__':
    import gps
    gpsd = gps.gps(mode=gps.WATCH_ENABLE)
    while True:
        print(gpsd.next())
        print('fix:', gpsd.fix.mode, gpsd.fix.track, gpsd.fix.speed, gpsd.fix.latitude, gpsd.fix.longitude)
