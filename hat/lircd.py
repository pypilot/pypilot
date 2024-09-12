#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time
import threading

# load lirc in background thread because it takes more than
# 2 seconds and this is an unreasonable delay
class LoadLIRC(threading.Thread):
    def __init__(self):
        super(LoadLIRC, self).__init__()
        self.version = 0
        self.daemon = True

    def run(self):
        warned = False
        while True:
            version = 0
            try:
                time.sleep(1)
                t0 = time.monotonic()
                import lirc as LIRC
                self.LIRC = LIRC
                version = 2
                print('have lirc for remote control', time.monotonic()-t0)
            except Exception as e:
                if not warned:
                    print('failed to load lirc', e)
                try:
                    import pylirc as LIRC
                    self.LIRC = LIRC
                    version = 1
                    print('have old lirc for remote control')
                except Exception as e:
                    if not warned:
                        print('no lirc available', e)
                    time.sleep(30)
                warned = True

            try:
                if version == 1:
                    LIRC.init('pypilot')
                    break
                elif version == 2:
                    self.lircd = LIRC.RawConnection()
                    break
            except Exception as e:
                print('failed to initialize lirc. is .lircrc missing?', e)
                time.sleep(60)
            time.sleep(2)
        self.version = version

class lirc(object):
    def __init__(self, config):
        self.lastkey = False
        self.lastcount = 0
        self.lasttime = time.monotonic()
        self.config = config
        self.LIRC = None

    def fileno(self):
        if self.LIRC and self.LIRC.version == 2:
            return self.LIRC.lircd.fileno()
        return None

    def poll(self):
        if not self.LIRC:
            if self.config['pi.ir']:
                self.LIRC = LoadLIRC()
                self.LIRC.start()
            else:
                return []
                
        if self.LIRC.version == 0:
            return []

        if self.LIRC.version == 1 and not self.LIRC.isAlive():
            return []

        t = time.monotonic()
        events = []
        while self.LIRC.version:
            if self.LIRC.version == 1:
                code = self.LIRC.LIRC.nextcode(0)
                if not code:
                    break
                count = code[0]['repeat']+1
            elif self.LIRC.version == 2:
                code = self.LIRC.lircd.readline(0)
                if not code:
                    break
                codes = code.split()
                count = int(codes[1], 16)+1
                key = codes[2]

            # continue to read from lirc but do not send event
            if not self.config['pi.ir']:
                continue
                
            if (self.lastkey and self.lastkey != key) or \
               self.lastcount >= count:
                events.append((self.lastkey, 0))
            self.lastkey = key
            self.lastcount = count
            self.lasttime = t
            #print('lirc', key, count, time.monotonic())
            events.append((key, count))

        # timeout keyup
        if self.lastkey and t - self.lasttime > .25:
            events.append((self.lastkey, 0))
            self.lastcount = 0
            self.lastkey = False
        return events

def main():
    lircd = lirc({'pi.ir': True})
    while True:
        events = lircd.poll()
        if events:
            print('events', events)
            lircd.events = []
        time.sleep(.02)
            
if __name__ == '__main__':
    main()
