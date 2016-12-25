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
    super(SimpleAutopilot, self).__init__(*args, **keywords)

    # create filters
    self.heading_error = self.Register(Value, 'heading_error', 0)
    self.heading_error_int = self.Register(Value, 'heading_error_int', 0)
    
    # create simple pid filter
    self.gains = {}
    self.timestamp = time.time()
    timestamp = self.server.TimeStamp('ap')
    def Gain(name, default, max_val):
      self.gains[name] = (self.Register(AutopilotGain, name, default, 0, max_val),
                          self.Register(SensorValue, name+'gain', timestamp))
    Gain('P', .01, .05)
    Gain('I', 0, .05)
    Gain('D', .1, .25)
    Gain('DD', 0, 1)
    Gain('rP', 0, .1)
    Gain('rD', 0, .1)

  def process_imu_data(self, boatimu):
    heading = self.heading.value
    headingrate = boatimu.SensorValues['headingrate'].value
    headingraterate = boatimu.SensorValues['headingraterate'].value

    roll = boatimu.SensorValues['roll'].value
    rollrate = boatimu.SensorValues['rollrate'].value

    heading_command = self.heading_command.value

    # filter the incoming heading and gyro heading
    err = heading - heading_command
    if err > 180:
      err -= 360
    elif err <= -180:
      err += 360

    # error +- 60 degrees
    err = min(max(err, -60), 60)
      
    self.heading_error.set(err)
    lp = .02
    self.heading_error_int.set(self.heading_error_int.value*(1-lp) + err*lp)

    command = 0
    gain_values = {'P': self.heading_error.value,
                   'I': self.heading_error_int.filtered.value,
                   'D': headingrate,
                   'DD': headingraterate,
                   'rP': roll,
                   'rD': rollrate}

    self.server.TimeStamp('ap', time.time())
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
