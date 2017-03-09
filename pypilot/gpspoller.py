#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import gps, multiprocessing, time, socket

import serialprobe

from signalk.values import *

class GpsProcess(multiprocessing.Process):
    def __init__(self):
        self.queue = multiprocessing.Queue()
        super(GpsProcess, self).__init__(target=self.gps_process, args=(self.queue, ))
        self.devices = []

    def connect(self):
        while True: # connection to gpsd loop
            try:
                self.gpsd = gps.gps(mode=gps.WATCH_ENABLE) #starting the stream of info
                self.gpsd.next() # flush initial message
                print 'GPS connected to gpsd'
                return
            except socket.error:
                time.sleep(3)

    def read(self, queue):
        while True:
            try:
                gpsdata = self.gpsd.next()

                if 'device' in gpsdata:
                    device = gpsdata['device']
                    if not device in self.devices:
                        queue.put({'device': device})
                        self.devices.append(device)

                if self.gpsd.fix.mode == 3:
                    fix = {}
                    fix['time'] = self.gpsd.fix.time
                    fix['track'] = self.gpsd.fix.track
                    fix['speed'] = self.gpsd.fix.speed
                    queue.put(fix)
                    
            except StopIteration:
                print 'GPS lost gpsd'
                break

    def gps_process(self, queue):
        while True:
            self.connect()
            self.read(queue)
            
class GpsPoller():
    def __init__(self, server):
        self.server = server
        self.track = self.Register(SensorValue, 'track')
        self.speed = self.Register(SensorValue, 'speed')
        self.source = self.Register(Value, 'source', 'none')
        self.lastsource = self.source.value
        self.process = False
        self.devices = []
        serialprobe.gpsdevices(self.devices)


    def Register(self, _type, name, *args):
        return self.server.Register(apply(_type, ['gps/' + name] + list(args)))

    def poll(self):
        if not self.process or not self.process.is_alive():
            self.process = GpsProcess()
            print 'start gps'
            self.process.start()
        
        if self.process.queue.qsize() > 0:
            fix = False
            # flush queue entries
            while self.process.queue.qsize() > 0:
                fix = self.process.queue.get()
                if 'device' in fix:
                    self.devices.append(fix['device'])
                    fix = False

            if fix:
                def fval(name):
                    return self.fix[name] if name in self.fix else False
            
                self.track.set(fval('track'))
                self.speed.set(fval('speed'))
                self.source.update('internal')

        if time.time() - self.track.timestamp > 3:
            self.source.update('none')

        source = self.source.value
        if (not source) != (not self.lastsource):
            print 'GPS Source changed to:', source
            self.lastsource = source

if __name__ == "__main__":
    from signalk.server import SignalKServer
    server = SignalKServer()
    gpsp = GpsPoller(server)
    count = 0;
    while True:
        gpsp.poll()
        print 'GPSPoller demo', count, gpsp.fix
        count += 1
        time.sleep(1)
