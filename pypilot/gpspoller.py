#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import gps, multiprocessing, time, socket

from signalk.values import *

class GpsProcess(multiprocessing.Process):
    def __init__(self):
        self.queue = multiprocessing.Queue()
        super(GpsProcess, self).__init__(target=self.gps_process, args=(self.queue, ))

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
                self.gpsd.next()
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
        try:
            while True:
                self.connect()
                self.read(queue)
        except KeyboardInterrupt:
            print 'Keyboard interrupt, gps process exit'
            
            
class GpsPoller():
    def __init__(self, server):
        self.server = server
        self.time = self.Register(Value, 'time', False)
        self.track = self.Register(SensorValue, 'track', self)
        self.speed = self.Register(SensorValue, 'speed', self)
        self.fix = False
        self.lastfix = False
        self.process = False
        self.timestamp = time.time()

    def Register(self, _type, name, *args):
        return self.server.Register(apply(_type, ['gps/' + name] + list(args)))

    def poll(self):
        if not self.process or not self.process.is_alive():
            self.process = GpsProcess()
            self.process.start()
        
        if self.process.queue.qsize() > 0:
            # flush queue entries
            while self.process.queue.qsize() > 0:
                self.fix = self.process.queue.get()

            def fval(name):
                return self.fix[name] if name in self.fix else False
            
            self.time.set(fval('time'))
            self.track.set(fval('track'))
            self.speed.set(fval('speed'))
            #self.timestamp = self.fix.time
            self.timestamp = time.time()

        if time.time() - self.timestamp > 3:
            self.fix = False

        if (not self.fix) != (not self.lastfix):
            print 'GPS ' + ('got' if self.fix else 'lost') + ' fix'
            self.lastfix = self.fix

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
