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

    def run(self):
        while True:
            try:
                time.sleep(1)
                t0 = time.monotonic()
                import lirc as LIRC
                self.LIRC = LIRC
                self.version = 2
                print('have lirc for remote control', time.monotonic()-t0)
            except Exception as e:
                print('failed to load lirc', e)
                try:
                    import pylirc as LIRC
                    self.LIRC = LIRC
                    self.version = 1
                    print('have old lirc for remote control')
                except Exception as e:
                    print('no lirc available', e)

            try:
                if self.version == 1:
                    LIRC.init('pypilot')
                    break
                elif self.version == 2:
                    self.lircd = LIRC.RawConnection()
                    break
            except Exception as e:
                print('failed to initialize lirc. is .lircrc missing?', e)
            time.sleep(10)

class lirc(object):
    def __init__(self):
        self.lastkey = False
        self.lasttime = time.time()
        self.enabled = False
        self.LIRC = None

    def poll(self):
        if not self.LIRC:
            if self.enabled:
                self.LIRC = LoadLIRC()
                self.LIRC.start()
            else:
                return []
                
        if self.LIRC.isAlive() or not self.LIRC.version:
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
            if not self.enabled:
                continue
                
            if self.lastkey and self.lastkey != key:
                events.append((self.lastkey, 0))
            self.lastkey = key
            self.lasttime = t
            events.append((key, count))

        # timeout keyup
        if self.lastkey and t - self.lasttime > .25:
            events.append((self.lastkey, 0))
            self.lastkey = False
        return events

def main():
    lircd = lirc()
    while True:
        events = lircd.poll()
        if events:
            print('events', events)
            lircd.events = []
        time.sleep(.1)
            
if __name__ == '__main__':
    main()
