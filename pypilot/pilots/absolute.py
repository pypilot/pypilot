#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from pilot import AutopilotPilot

# this pilot is an experiment to command the rudder
# to an absolute position rather than relative speed
# 
# This pilot requires rudder feedback
#
class AbsolutePilot(AutopilotPilot):
  def __init__(self, ap):
    super(AbsolutePilot, self).__init__('absolute', ap)

    # create simple pid filter
    self.gains = {}
    self.PosGain('P', .05, 2)
    self.PosGain('I', 0, .05)
    self.PosGain('D', .2, 2)
    self.PosGain('DD',  0, 1) # rate of derivative

  def process(self, reset):
    ap = self.ap

    if type(ap.sensors.rudder.angle.value) == type(False):
        ap.pilot.set('basic') # fall back to basic pilot if rudder feedback fails
        return
    
    headingrate = ap.boatimu.SensorValues['headingrate'].value
    headingraterate = ap.boatimu.SensorValues['headingraterate_lowpass'].value
    gain_values = {'P': ap.heading_error.value,
                   'I': ap.heading_error_int.value,
                   'D': headingrate,
                   'DD': headingraterate}

    command = self.Compute(gain_values)

    if ap.enabled.value:
        ap.servo.position_command.set(command)

pilot = AbsolutePilot
