#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

try:
  from autopilot import *
except:
  from pypilot.autopilot import *

global default_pilots

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
    timestamp = self.ap.server.TimeStamp('ap')
    self.Gain('P', .05, 0, 2)
    self.Gain('I', 0, 0, .05)
    self.Gain('D', .2, 0, 2)
    self.Gain('DD',  0, 0, 1) # rate of derivative

  def process_imu_data(self):
    ap = self.ap

    if type(ap.sensors.rudder.value) == type(False):
        ap.pilot.set('basic') # fall back to basic pilot if rudder feedback failes
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
