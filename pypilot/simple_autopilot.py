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

class SimpleAutopilot(AutopilotBase):
  def __init__(self, *args, **keywords):
    super(SimpleAutopilot, self).__init__('Simple')

    # create filters
    self.heading_error = self.Register(SensorValue, 'heading_error', 0)
    self.heading_error_int = self.Register(SensorValue, 'heading_error_int', 0)
    self.heading_error_int_time = time.time()
    
    # create simple pid filter
    self.gains = {}
    timestamp = self.server.TimeStamp('ap')
    def Gain(name, default, max_val):
      self.gains[name] = (self.Register(AutopilotGain, name, default, 0, max_val),
                          self.Register(SensorValue, name+'gain', timestamp))
    Gain('P', .005, .025)
    Gain('I', 0, .05)
    Gain('D', .15, .5)

  def process_imu_data(self, boatimu):
    heading = self.heading.value
    headingrate = boatimu.SensorValues['headingrate'].value

    heading_command = self.heading_command.value

    t = time.time()

    # filter the incoming heading and gyro heading
    # error +- 60 degrees
    err = minmax(autopilot.resolv(heading - heading_command), 60)
    self.heading_error.set(err)

    # int error +- 20
    dt = t - self.heading_error_int_time
    dt = max(min(dt, 1), 0) # ensure dt is from 0 to 1
    self.heading_error_int_time = t
    err_int = minmax(self.heading_error_int.value + err*dt, 20)
    self.heading_error_int.set(err_int)

    command = 0
    gain_values = {'P': self.heading_error.value,
                   'I': self.heading_error_int.value,
                   'D': headingrate}

    self.server.TimeStamp('ap', t)
    for gain in self.gains:
      value = gain_values[gain]
      gains = self.gains[gain]
      gains[1].set(gains[0].value*value)
      command += gains[1].value

    self.servo.command.set(command)

def main():
  ap = SimpleAutopilot()
  ap.run()

if __name__ == '__main__':
  main()
