#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from autopilot import *
import servo

def minmax(value, r):
  return min(max(value, -r), r)

class BasicAutopilot(AutopilotBase):
  def __init__(self, *args, **keywords):
    super(BasicAutopilot, self).__init__('Basic')

    # create filters
    timestamp = self.server.TimeStamp('ap')
    self.heading_error = self.Register(SensorValue, 'heading_error', timestamp)
    self.heading_error_int = self.Register(SensorValue, 'heading_error_int', timestamp)
    self.heading_error_int_time = time.time()
    
    # create simple pid filter
    self.gains = {}
    def Gain(name, default, max_val):
      self.gains[name] = (self.Register(AutopilotGain, name, default, 0, max_val),
                          self.Register(SensorValue, name+'gain', timestamp))
    Gain('P', .005, .05)
    Gain('I',    0, .02)
    Gain('D',  .15, 1.0)

    Gain('P2', 0, .025)
    Gain('PD', 0, .1)
    Gain('D2', 0, 1)

  def process_imu_data(self, boatimu):
    heading = self.heading.value
    headingrate = boatimu.SensorValues['headingrate_lowpass'].value

    heading_command = self.heading_command.value

    t = time.time()

    # filter the incoming heading and gyro heading
    # error +- 60 degrees
    err = minmax(autopilot.resolv(heading - heading_command), 60)
    self.heading_error.set(err)

    # int error +- 60
    dt = t - self.heading_error_int_time
    dt = max(min(dt, 1), 0) # ensure dt is from 0 to 1
    self.heading_error_int_time = t
    err_int = minmax(self.heading_error_int.value + err*dt, 60)
    self.heading_error_int.set(err_int)

    command = 0
    gain_values = {'P': self.heading_error.value,
                   'I': self.heading_error_int.value,
                   'D': headingrate}
    gain_values['P2'] = abs(gain_values['P'])*gain_values['P']
    if gain_values['P']*gain_values['D'] > 0:
      gain_values['PD'] = abs(gain_values['P'])*gain_values['D']
    else:
      gain_values['PD'] = 0
    gain_values['D2'] = abs(gain_values['D'])*gain_values['D']

    self.server.TimeStamp('ap', t)
    for gain in self.gains:
      value = gain_values[gain]
      gains = self.gains[gain]
      gains[1].set(gains[0].value*value)
      command += gains[1].value

    self.servo.command.set(command)

def main():
  ap = BasicAutopilot()
  ap.run()

if __name__ == '__main__':
  main()
