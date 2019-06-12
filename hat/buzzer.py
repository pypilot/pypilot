#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from __future__ import print_function

try:
    import RPi.GPIO as GPIO
    print('have gpio for raspberry pi')

except ImportError:
    try:
        import OPi.GPIO as GPIO
        orangepi = True
        print('have gpio for orange pi')
    except:
        print('No gpio available')
        GPIO = None

class buzzer(object):
    def __init__(self, config):
        if 'buzzer' in config:
            self.config = config['buzzer']
        else:
            self.config = False

    def poll(self):
        pass
            
    def buzz(self, frequency, duration):
        if not self.config:
            return

        self.stop
        pass # buzz here
