#!/usr/bin/env python
#
#   Copyright (C) 2023 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from pilot import AutopilotPilot
from resolv import resolv
from pypilot.values import *

class RatePilot(AutopilotPilot):
  def __init__(self, ap, name='rate'):
    super(RatePilot, self).__init__(name, ap)

    # create extended pid filter
    self.PosGain('D', .075, 0.3) # rate of derivative
    self.PosGain('DD', .075, 0.3) # rate of derivative
    self.PosGain('FF', .6, 3.0)   # feed forward

    self.maxturnrate = self.register(RangeProperty, 'maxturnrate', 2, .5, 30, persistent=True)
    self.turnraterate = self.register(RangeProperty, 'turnraterate', .5, .1, 5, persistent=True, profiled=True)


  def process(self):
    t = time.monotonic()
    ap = self.ap
    
    # compute command
    headingerror = ap.heading_error.value
    headingrate = ap.boatimu.SensorValues['headingrate_lowpass'].value
    headingraterate = ap.boatimu.SensorValues['headingraterate_lowpass'].value

    maxturnrate = self.maxturnrate.value
    raterate = self.turnraterate.value

    rate = min(max(headingerror*raterate, -maxturnrate), maxturnrate)

    rateerror = headingrate - rate
    gain_values = {'D': rateerror,
                   'DD': headingraterate,
                   'FF': ap.heading_command_rate.value}

    command = self.Compute(gain_values)
    if ap.enabled.value:
        ap.servo.command.command(command)

pilot = RatePilot
