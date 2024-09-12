#!/usr/bin/env python
#
#   Copyright (C) 2021 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from pilot import AutopilotPilot
from resolv import resolv
from pypilot.values import *

class BasicPilot(AutopilotPilot):
  def __init__(self, ap, name='basic'):
    super(BasicPilot, self).__init__(name, ap)

    # create extended pid filter
    self.PosGain('P',  .003, .03)   # position (heading error)
    self.PosGain('I',   0, .05)       # integral
    self.PosGain('D',  .09, 0.24)    # derivative (gyro)
    self.PosGain('DD', .075, 0.24) # rate of derivative
    self.PosGain('PR', .005, .02) # position root
    self.PosGain('FF', .6, 2.4)   # feed forward
    #self.PosGain('R',  0.0, 1.0)  # reactive
    #self.reactive_time = self.register(RangeProperty, 'Rtime', 1, 0, 3)

    #self.reactive_value = self.register(SensorValue, 'reactive_value')

  def process(self):
    t = time.monotonic()
    ap = self.ap
    
    # compute command
    headingrate = ap.boatimu.SensorValues['headingrate_lowpass'].value
    headingraterate = ap.boatimu.SensorValues['headingraterate_lowpass'].value
    #reactive_value = self.servocommand_queue.take(t - self.reactive_time.value)
    #self.reactive_value.update(reactive_value)
    
    gain_values = {'P': ap.heading_error.value,
                   'I': ap.heading_error_int.value,
                   'D': headingrate,      
                   'DD': headingraterate,
                   'FF': ap.heading_command_rate.value,
#                   'R': -reactive_value
    }
    PR = math.sqrt(abs(gain_values['P']))
    if gain_values['P'] < 0:
        PR = -PR
    gain_values['PR'] = PR

    command = self.Compute(gain_values)
      
#    rval = self.gains['R']['sensor'].value
    # don't include R contribution to command
#    self.servocommand_queue.add(command - rval)
    
    if ap.enabled.value:
        ap.servo.command.command(command)

pilot = BasicPilot
