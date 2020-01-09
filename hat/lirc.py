#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from __future__ import print_function
import time
LIRC = None
try:
    import pylirc as LIRC
    print('have lirc for remote control')
except:
    print('no lirc available')
    LIRC = None

class lirc(object):
    def __init__(self):
        self.events = []
        self.lastkey = False
        self.lasttime = time.time()

        global LIRC
        if LIRC:
            try:
                LIRC.init('pypilot')
                self.lirctime = False
            except:
                print('failed to initialize lirc. is .lircrc missing?')
                LIRC = None

    def poll(self):
        if not LIRC:
            return

        t = time.time()
        while True:
            code = LIRC.nextcode(1)
            if not code:
                break

            count = code[0]['repeat']+1
            key = 'lirc' + code[0]['config']

            if self.lastkey and self.lastkey != key:
                self.events.append((self.lastkey, 0))
            self.lastkey = key
            self.lasttime = t
            self.events.append((key, count))

        # timeout keyup
        if self.lastkey and t - self.lasttime > .25:
            self.events.append((self.lastkey, 0))
            self.lastkey = False
