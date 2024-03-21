#!/usr/bin/env python
#
#   Copyright (C) 2023 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  


# this pilot is an experiment to compare performance to others

from pilot import AutopilotPilot
from resolv import resolv
from pypilot.values import *
disabled = True

class DeadZonePilot(AutopilotPilot):
  def __init__(self, ap, name='deadzone'):
    super(DeadZonePilot, self).__init__(name, ap)
    self.deadzone = self.register(RangeProperty, 'deadzone', 5, 1, 30, persistent=True, profiled=True)

    self.PosGain('P',  .003, .15)   # position (heading error)
    self.PosGain('D',  .09, .3)    # derivative (gyro)
    self.PosGain('DD', .075, 0.3) # rate of derivative
    self.PosGain('FF', .6, 3.0)   # feed forward

  def process(self):
    t = time.monotonic()
    ap = self.ap
    
    # compute command
    headingrate = ap.boatimu.SensorValues['headingrate_lowpass'].value
    headingraterate = ap.boatimu.SensorValues['headingraterate_lowpass'].value

    if abs(ap.heading_error.value) < self.deadzone.value:
        command = 0
    else:
        gain_values = {'P': ap.heading_error.value,
                       'D': headingrate,      
                       'DD': headingraterate,
                       'FF': ap.heading_command_rate.value}
        command = self.Compute(gain_values)
        if abs(command) < ap.servo.speed.min.value:
            command = 0
      
    if ap.enabled.value: # perform absolute servo command (not delayed by period)
        ap.servo.command.set(command)

pilot = DeadZonePilot
