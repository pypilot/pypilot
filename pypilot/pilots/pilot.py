#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from pypilot.values import *
from pypilot.resolv import resolv

class AutopilotGain(RangeProperty):
    def __init__(self, *cargs):
        super(AutopilotGain, self).__init__(*cargs, persistent=True, profiled=True)
        self.info['AutopilotGain'] = True

class AutopilotPilot(object):
    def __init__(self, name, ap):
        super(AutopilotPilot, self).__init__()
        self.name = name
        self.ap = ap
        self.gains = {}

    def register(self, _type, name, *args, **kwargs):
        return self.ap.client.register(_type(*(['ap.pilot.' + self.name + '.' + name] + list(args)), **kwargs))

    def Gain(self, name, default, min_val, max_val, compute=None):
        if not compute:
            compute = lambda value : value * self.gains[name]['apgain'].value
        self.gains[name] = {'apgain': self.register(AutopilotGain, name, default, min_val, max_val),
                            'sensor': self.register(SensorValue, name+'gain'),
                            'compute': compute}

    def PosGain(self, name, default, max_val):
        self.Gain(name, default, 0, max_val)
    
    def Compute(self, gain_values):
        command = 0
        for gain in self.gains:
            value = gain_values[gain]
            gains = self.gains[gain]
            gains['sensor'].set(gains['compute'](value))
            #print('compute', gain, value, gains['sensor'].value)
            command += gains['sensor'].value
            #print('command', command)
        return command

    def compute_heading(self):        
        ap = self.ap
        compass = ap.boatimu.SensorValues['heading_lowpass'].value

        if ap.mode.value == 'true wind':
            true_wind = resolv(ap.true_wind_compass_offset.value - compass)
            ap.heading.set(true_wind)
        elif ap.mode.value == 'wind':
            wind = resolv(ap.wind_compass_offset.value - compass)
            ap.heading.set(wind)
        elif ap.mode.value == 'gps' or ap.mode.value == 'nav':
            gps = resolv(compass + ap.gps_compass_offset.value, 180)
            ap.heading.set(gps)
        elif ap.mode.value == 'compass':
            ap.heading.set(compass)

    # return new mode if sensors don't support it
    def best_mode(self, mode):
        modes = self.ap.modes.value
        while not mode in modes:
            fallbacks = {'nav': 'gps', 'gps': 'compass', 'wind': 'compass', 'true wind': 'wind'}
            if not mode in fallbacks:
                break
            mode = fallbacks[mode] 
        return mode
