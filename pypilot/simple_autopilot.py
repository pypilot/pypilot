#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from signalk.server import *
from autopilot import *
import servo

class SimpleAutopilot(AutopilotBase):
  def __init__(self, *args, **keywords):
    super(SimpleAutopilot, self).__init__(*args, **keywords)

    # create filters
    self.heading_error = self.Register(Value, 'heading_error', 0)
    self.heading_error_int = self.CreateFilter('heading_error_int', 0, .02)
    
    # create simple pid filter
    self.P = self.Register(AutopilotGain, 'P', .01, 0, .05)
    self.I = self.Register(AutopilotGain, 'I', 0, 0, .05)
    self.D = self.Register(AutopilotGain, 'D', .1, 0, .25)
    self.DD = self.Register(AutopilotGain, 'DD', 0, 0, 1)
    self.lastheadingrate = 0

    self.hP = self.Register(AutopilotGain, 'hP', 0, 0, .1)
    self.hD = self.Register(AutopilotGain, 'hD', 0, 0, .1)

  def process_imu_data(self, boatimu):
    heading = self.heading.value
    headingrate = boatimu.SensorValues['headingrate'].value
    headingraterate = boatimu.SensorValues['headingrate'].value - self.lastheadingrate
    self.lastheadingrate = headingrate
    heel = boatimu.SensorValues['heel'].value
    rollrate = boatimu.SensorValues['rollrate'].value

    heading_command = self.heading_command.value

    # filter the incoming heading and gyro heading
    err = heading - heading_command
    if err > 180:
      err -= 360
    elif err <= -180:
      err += 360
    self.heading_error.set(err)
    self.heading_error_int.update(err)

    command = self.P.value*self.heading_error.value + \
              self.I.value*self.heading_error_int.filtered.value + \
              self.D.value*headingrate + \
              self.DD.value*headingraterate*10 + \
              self.hP.value*heel + self.hD.value*rollrate
       
    self.servo.command.set(command)

def main():
  ap = SimpleAutopilot()

  def cleanup(signal_number, frame):
    exit(1)

  import signal
  for s in range(1, 16):
    if s != 9:
      signal.signal(s, cleanup)
  
  try:
    ap.run()
  except KeyboardInterrupt:
    print 'Keyboard interrupt, autopilot process exit'
#  except:
#    print 'exception'

if __name__ == '__main__':
  main()
