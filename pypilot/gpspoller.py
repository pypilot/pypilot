#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import gps, multiprocessing, time

from signalk.values import *

class GpsProcess(multiprocessing.Process):
    def __init__(self):
        self.queue = multiprocessing.Queue()
        super(GpsProcess, self).__init__(target=self.gps_process, args=(self.queue, ))
        self.show_gps_fail = True
        
    def gps_process(self, queue):
        while True:
            try:
                gpsd = gps.gps(mode=gps.WATCH_ENABLE) #starting the stream of info
                break
            except:
                if self.show_gps_fail:
                    self.show_gps_fail = False
                    print 'GPS Process failed to connect to gpsd.'
            
                time.sleep(3)
            
        print 'GPS Process connected to gpsd.'
        self.show_gps_fail = True
            
        while True:
            try:
                gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer
            except KeyboardInterrupt:
                print 'Keyboard interrupt, gps process exit'
                break
            except:
                print 'GPS Process lost gpsd'
                break
            queue.put(gpsd.fix)

            
class GpsPoller():
    def __init__(self, server):
        self.server = server
        self.track = self.Register(SensorValue, 'track', self)
        self.speed = self.Register(SensorValue, 'speed', self)
        self.fix = False
        self.process = False
        self.timestamp = time.time()

    def Register(self, _type, name, *args):
        return self.server.Register(apply(_type, ['gps/' + name] + list(args)))

    def poll(self):
        if not self.process or not self.process.is_alive():
            self.process = GpsProcess()
            self.process.start()
        
        if self.process.queue.qsize() == 0:
            return

        # flush queue entries
        while self.process.queue.qsize() > 1:
            self.process.queue.get()

        self.fix = self.process.queue.get()
        if self.fix and self.fix.mode == 3:
            self.track.set(self.fix.track)
            self.speed.set(self.fix.speed)
            #self.timestamp = self.fix.time
            self.timestamp = time.time()
