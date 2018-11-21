#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from autopilot import *

global default_pilots

class BasicPilot(AutopilotPilot):
  def __init__(self, ap):
    super(BasicPilot, self).__init__('basic', ap)

    # create filters
    timestamp = self.ap.server.TimeStamp('ap')

    self.heading_command_rate = self.Register(SensorValue, 'heading_command_rate', timestamp)
    self.headingrate_queue = TimedQueue(6)

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
        
    PosGain('P', .003, .02)
    PosGain('I', 0, .1)
    PosGain('D',  .1, 1.0)
    PosGain('DD',  .05, 1.0)
    
    def PosGain2(name, default, max_val):
      def compute2(value):
        return value * abs(value) * self.gains[name]['apgain'].value
      Gain(name, default, 0, max_val, compute2)

    PosGain('PR',  0, .05)
    PosGain2('D2', 0, .05)

    PosGain('FF',  .5, 3.0)

    PosGain('R',  .1, 1.0)

    self.lastenabled = False

  def process_imu_data(self):
    ap = self.ap
    if ap.enabled.value != self.lastenabled:
      self.lastenabled = ap.enabled.value
      if ap.enabled.value:
        ap.heading_error_int.set(0) # reset integral
        # reset feed-forward gain
        self.last_heading_command = self.heading_command.value
        self.heading_command_rate.set(0)

    # reset feed-forward error if mode changed
    if ap.mode.value != ap.lastmode:
      self.last_heading_command = ap.heading_command.value
    
    # if disabled, only bother to compute if a client cares        
    if not ap.enabled.value: 
      compute = False
      for gain in self.gains:
        if self.gains[gain]['sensor'].watchers:
          compute = True
          break
      if not compute:
        return
    
    # filter the heading command to compute feed-forward gain
    heading_command_diff = resolv(ap.heading_command.value - self.last_heading_command)
    self.last_heading_command = ap.heading_command.value
    lp = .1
    command_rate = (1-lp)*self.heading_command_rate.value + lp*heading_command_diff
    self.heading_command_rate.set(command_rate)

    # compute command
    command = 0
    headingrate = ap.boatimu.SensorValues['headingrate_lowpass'].value
    headingraterate = ap.boatimu.SensorValues['headingraterate_lowpass'].value
    feedforward_value = self.heading_command_rate.value
    self.headingrate_queue.add(headingrate)
    reactive_value = self.headingrate_queue.take(time.time() - ap.servo.period.value*6)
    if not 'wind' in ap.mode.value:
      feedforward_value = -feedforward_value
    gain_values = {'P': ap.heading_error.value,
                   'I': ap.heading_error_int.value,
                   'D': headingrate,      
                   'DD': headingraterate,
                   'FF': feedforward_value,
                   'R': reactive_value}
    PR = math.sqrt(abs(gain_values['D']))
    if gain_values['P'] < 0:
      PR = -PR
    gain_values['PR'] = PR
    gain_values['D2'] = gain_values['D']

    for gain in self.gains:
      value = gain_values[gain]
      gains = self.gains[gain]
      gains['sensor'].set(gains['compute'](value))
      command += gains['sensor'].value

    if ap.enabled.value:
      ap.servo.command.set(command)
