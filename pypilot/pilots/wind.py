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

class WindPilot(AutopilotPilot):
  def __init__(self, ap):
    super(WindPilot, self).__init__('wind', ap)

    # wind pilot
    # create simple pid filter
    self.gains = {}
    timestamp = self.ap.server.TimeStamp('ap')
    def Gain(name, default, max_val):
      self.gains[name] = {'apgain': self.Register(AutopilotGain, name, default, 0, max_val),
                          'sensor': self.Register(SensorValue, name+'gain', timestamp)}
    Gain('P', .005, .025)
    Gain('I', 0, .05)
    Gain('D', .15, .5)
    Gain('WS', .15, .5)
    

  def process_imu_data(self):
    ap = self.ap
    command = 0
    headingrate = ap.boatimu.SensorValues['headingrate'].value

    wind_speed = ap.sensors.wind.speed
    
    gain_values = {'P': ap.heading_error.value,
                   'I': ap.heading_error_int.value,
                   'D': headingrate,
                   'WS': ws}

    command = self.Compute(gain_values)

    if ap.enabled.value:
      ap.servo.command.set(command)
