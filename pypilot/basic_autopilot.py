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

    def Gain(name, default, min_val, max_val, compute=None):
      if not compute:
        compute = lambda value : value * self.gains[name]['apgain'].value
      self.gains[name] = {'apgain': self.Register(AutopilotGain, name, default, min_val, max_val),
                          'sensor': self.Register(SensorValue, name+'gain', timestamp),
                          'compute': compute}

    def PosGain(name, default, max_val):
      Gain(name, default, 0, max_val)
        
    PosGain('P', .005, .01)
    PosGain('I',    0, .25)
    PosGain('D',  .15, 1.0)

    PosGain('DD',  0, 1.0)
    
    def PosGain2(name, default, max_val, other):
      def compute2(value):
        return value * abs(value) * self.gains[name]['apgain'].value * self.gains[other]['apgain'].value
      Gain(name, default, 0, max_val, compute2)

    PosGain2('P2', 0, 1, 'P')
    PosGain2('D2', 0, 1, 'D')

    self.lastenabled = False

  def process_imu_data(self, boatimu):
    if self.enabled.value != self.lastenabled:
      self.lastenabled = self.enabled.value
      if self.enabled.value:
        self.heading_error_int.set(0) # reset integral
    if not self.enabled.value: # only bother to compute if a client cares
      compute = False
      for gain in self.gains:
        if self.gains[gain]['sensor'].watchers:
          compute = True
          break
      if not compute:
        return
    
    heading = self.heading.value
    headingrate = boatimu.SensorValues['headingrate_lowpass'].value
    headingraterate = boatimu.SensorValues['headingraterate_lowpass'].value

    heading_command = self.heading_command.value

    t = time.time()

    # filter the incoming heading and gyro heading
    # error +- 60 degrees
    err = minmax(autopilot.resolv(heading - heading_command), 60)
    self.heading_error.set(err)

    dt = t - self.heading_error_int_time
    dt = max(min(dt, 1), 0) # ensure dt is from 0 to 1
    self.heading_error_int_time = t
    # int error +- 1, from 0 to 1000 deg/s
    err_int = minmax(self.heading_error_int.value + (err/1000)*dt, 1)
    self.heading_error_int.set(err_int)

    command = 0
    gain_values = {'P': self.heading_error.value,
                   'I': self.heading_error_int.value,
                   'D': headingrate,
                   'DD': headingraterate}
    gain_values['P2'] = gain_values['P']
    gain_values['D2'] = gain_values['D']

    self.server.TimeStamp('ap', t)
    for gain in self.gains:
      value = gain_values[gain]
      gains = self.gains[gain]
      gains['sensor'].set(gains['compute'](value))
      command += gains['sensor'].value

    if self.enabled.value:
      self.servo.command.set(command)

def main():
  ap = BasicAutopilot()
  ap.run()

if __name__ == '__main__':
  main()
