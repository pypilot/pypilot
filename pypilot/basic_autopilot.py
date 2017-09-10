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

class BasicAutopilot(AutopilotBase):
  def __init__(self, *args, **keywords):
    super(BasicAutopilot, self).__init__('Basic')

    # create filters
    timestamp = self.server.TimeStamp('ap')

    self.heading_command_rate = self.Register(SensorValue, 'heading_command_rate', timestamp)
    
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

    PosGain('FF',  0, 1.0)

    self.lastenabled = False

  def process_imu_data(self):
    if self.enabled.value != self.lastenabled:
      self.lastenabled = self.enabled.value
      if self.enabled.value:
        self.heading_error_int.set(0) # reset integral
        # reset feed-forward gain
        self.last_heading_command = self.heading_command.value
        self.heading_command_rate.set(0)

    # reset feed-forward error if mode changed
    if self.mode.value != self.lastmode:
      self.last_heading_command = self.heading_command.value
    
    # if disabled, only bother to compute if a client cares        
    if not self.enabled.value: 
      compute = False
      for gain in self.gains:
        if self.gains[gain]['sensor'].watchers:
          compute = True
          break
      if not compute:
        return
    
    # filter the heading command to compute feed-forward gain
    heading_command_diff = self.heading_command.value - self.last_heading_command
    self.last_heading_command = self.heading_command.value
    lp = .1
    command_rate = (1-lp)*self.heading_command_rate.value + lp*heading_command_diff
    self.heading_command_rate.set(command_rate)

    # compute command
    command = 0
    headingrate = self.boatimu.SensorValues['headingrate_lowpass'].value
    headingraterate = self.boatimu.SensorValues['headingraterate_lowpass'].value
    gain_values = {'P': self.heading_error.value,
                   'I': self.heading_error_int.value,
                   'D': headingrate,
                   'DD': headingraterate,
                   'FF': self.heading_command_rate.value}
    gain_values['P2'] = gain_values['P']
    gain_values['D2'] = gain_values['D']

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
