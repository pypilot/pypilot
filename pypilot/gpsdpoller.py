#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import gps, multiprocessing, time, socket
import select

from signalk.values import *
from signalk.pipeserver import nonblockingpipe

class GpsProcess(multiprocessing.Process):
    def __init__(self):
        self.pipe, pipe = nonblockingpipe()
        super(GpsProcess, self).__init__(target=self.gps_process, args=(pipe, ))
        self.devices = []

    def connect(self):
        while True: # connection to gpsd loop
            try:
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
                    fix['speed'] = self.gpsd.fix.speed
                    pipe.send(fix)
                    lasttime = time.time()

            except StopIteration:
                print 'lost connection to gpsd'
                break

    def gps_process(self, pipe):
        import os
        print 'gps on', os.getpid()
        while True:
            self.connect()
            self.read(pipe)
            
class GpsdPoller():
    def __init__(self, nmea):
        self.nmea = nmea

        self.process = False
        self.devices = []
        nmea.serialprobe.gpsdevices = self.devices

        self.process = GpsProcess()
        self.process.start()
        READ_ONLY = select.POLLIN | select.POLLHUP | select.POLLERR
        self.poller = select.poll()
        self.poller.register(self.process.pipe.fileno(), READ_ONLY)

    def poll(self):
        # flush queue entries
        if not self.poller.poll(0):
            return

        try:
            fix = self.process.pipe.recv()
        except IOError:
            return

        if 'device' in fix:
            self.devices.append(fix['device'])
            return

        def fval(name):
            return fix[name] if name in fix else False

        val = {'timestamp' : time.time(), 'track': fval('track'), 'speed': fval('speed')}
        self.nmea.handle_messages({'gps': val}, 'gpsd')
