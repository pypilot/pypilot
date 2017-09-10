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

class SimpleAutopilot(AutopilotBase):
  def __init__(self, *args, **keywords):
    super(SimpleAutopilot, self).__init__('Simple')

    # create simple pid filter
    self.gains = {}
    timestamp = self.server.TimeStamp('ap')
    def Gain(name, default, max_val):
      self.gains[name] = (self.Register(AutopilotGain, name, default, 0, max_val),
                          self.Register(SensorValue, name+'gain', timestamp))
    Gain('P', .005, .025)
    Gain('I', 0, .05)
    Gain('D', .15, .5)

  def process_imu_data(self):
    command = 0
    headingrate = self.boatimu.SensorValues['headingrate'].value
    gain_values = {'P': self.heading_error.value,
                   'I': self.heading_error_int.value,
                   'D': headingrate}

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
