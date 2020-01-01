#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from __future__ import print_function

LIRC = None
try:
    import pylirc as LIRC
    print('have lirc for remote control')
except:
    print('no lirc available')
    LIRC = None

class lirc(object):
    def __init__(self):
        global LIRC
        if LIRC:
            try:
                LIRC.init('pypilot')
                self.lirctime = False
            except:
                print('failed to initialize lirc. is .lircrc missing?')
                LIRC = None

    def poll(self):
        # handle timeout and key up here
        pass

    def read(self):
        while LIRC:
            code = LIRC.nextcode(1)
            if not code:
                break

            count = code[0]['repeat']+1
            key = 'lirc' + code[0]['config']

            if count == 0:
                return key, count
            elif count == 2:
                return key, count
