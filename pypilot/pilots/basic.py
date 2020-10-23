#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from pilot import AutopilotPilot
from resolv import resolv
from pypilot.values import *

class TimedQueue(object):
  def __init__(self, length):
    self.data = []
    self.length = length

  def add(self, data):
    t = time.monotonic()
    while self.data and self.data[0][1] < t-self.length:
      self.data = self.data[1:]
    self.data.append((data, t))

  def take(self, t):
    while self.data and self.data[0][1] < t:
        self.data = self.data[1:]
    if self.data:
      return self.data[0][0]
    return 0

class BasicPilot(AutopilotPilot):
  def __init__(self, ap):
    super(BasicPilot, self).__init__('basic', ap)

    # create filters
    self.heading_command_rate = self.register(SensorValue, 'heading_command_rate')
    self.heading_command_rate.time = 0
    self.servocommand_queue = TimedQueue(10) # remember at most 10 seconds

    # create extended pid filter
    self.gains = {}
        
    self.PosGain('P', .003, .02)   # position (heading error)
    self.PosGain('I', 0.005, .1)   # integral
    self.PosGain('D',  .09, 1.0)   # derivative (gyro)
    self.PosGain('DD',  .075, 1.0) # rate of derivative
    self.PosGain('PR',  .005, .05)  # position root
    self.PosGain('FF',  .6, 3.0) # feed forward
    self.PosGain('R',  0.0, 1.0)  # reactive
    self.reactive_time = self.register(RangeProperty, 'Rtime', 1, 0, 3)

    self.reactive_value = self.register(SensorValue, 'reactive_value')
                                    
    self.last_heading_mode = False

  def process(self, reset):
    t = time.monotonic()
    ap = self.ap
    if reset:
        self.heading_command_rate.set(0)
        # reset feed-forward gain
        self.last_heading_mode = False

    # reset feed-forward error if mode changed, or last command is older than 1 second
    if self.last_heading_mode != ap.mode.value or t - self.heading_command_rate.time > 1:
      self.last_heading_command = ap.heading_command.value
    
    # if disabled, only compute if a client cares
    if not ap.enabled.value: 
      compute = False
      for gain in self.gains:
        if self.gains[gain]['sensor'].watch:
          compute = True
          break
      if not compute:
        return

    # filter the heading command to compute feed-forward gain
    heading_command_diff = resolv(ap.heading_command.value - self.last_heading_command)
    self.last_heading_command = ap.heading_command.value
    self.last_heading_mode = ap.mode.value
    self.heading_command_rate.time = t;
    lp = .1
    command_rate = (1-lp)*self.heading_command_rate.value + lp*heading_command_diff
    self.heading_command_rate.update(command_rate)

    # compute command
    headingrate = ap.boatimu.SensorValues['headingrate_lowpass'].value
    headingraterate = ap.boatimu.SensorValues['headingraterate_lowpass'].value
    feedforward_value = self.heading_command_rate.value
    reactive_value = self.servocommand_queue.take(t - self.reactive_time.value)
    self.reactive_value.update(reactive_value)
    
    if not 'wind' in ap.mode.value: # wind mode needs opposite gain
        feedforward_value = -feedforward_value
    gain_values = {'P': ap.heading_error.value,
                   'I': ap.heading_error_int.value,
                   'D': headingrate,      
                   'DD': headingraterate,
                   'FF': feedforward_value,
                   'R': -reactive_value}
    PR = math.sqrt(abs(gain_values['P']))
    if gain_values['P'] < 0:
        PR = -PR
    gain_values['PR'] = PR

    command = self.Compute(gain_values)
      
    rval = self.gains['R']['sensor'].value
    # don't include R contribution to command
    self.servocommand_queue.add(command - rval)
    
    if ap.enabled.value:
        ap.servo.command.set(command)

pilot = BasicPilot
