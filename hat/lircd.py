#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time
LIRC = None
LIRC_version = 0
try:
    import lirc as LIRC
    LIRC_version = 2
    print('have lirc for remote control')
    
except Exception as e:
    try:
        import pylirc as LIRC
        LIRC_version = 1
        print('have old lirc for remote control')
    except Exception as e:
        print('no lirc available', e)
        LIRC_version = 0

class lirc(object):
    def __init__(self):
        self.lastkey = False
        self.lasttime = time.monotonic()

        global LIRC_version, LIRC
        try:
            if LIRC_version == 1:
                LIRC.init('pypilot', blocking=False)
            elif LIRC_version == 2:
                self.lircd = LIRC.RawConnection()
            self.lirctime = False
        except Exception as e:
            print('failed to initialize lirc. is .lircrc missing?', e)
            LIRC = None

    def poll(self):
        if not LIRC_version:
            return []

        t = time.monotonic()
        events = []
        while LIRC_version:
            if LIRC_version == 1:
                code = LIRC.nextcode(0)
                if not code:
                    break
                count = code[0]['repeat']+1
            elif LIRC_version == 2:
                code = self.lircd.readline(0)
                if not code:
                    break
                codes = code.split()
                count = int(codes[1], 16)+1
                key = codes[2]

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
