#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from signalk.values import *
from resolv import *

class Tack(object):
  def __init__(self, ap):
    self.ap = ap

    # tacking states
    # none - not tacking, normal ap operation
    # begin - control application sets this to initiate tack
    # waiting - waiting delay seconds before beginning to tack
    # tacking - rudder is moving at tack rate until threshold
    
    self.state = self.Register(EnumProperty, 'state', 'none', ['none', 'begin', 'waiting', 'tacking'])
    self.timeout = self.Register(Value, 'timeout', 0)

    self.delay = self.Register(RangeProperty, 'delay', 0, 0, 60)
    self.angle = self.Register(RangeProperty, 'angle', 100, 10, 180)
    self.rate = self.Register(RangeProperty, 'rate', 20, 1, 100)
    self.threshold = self.Register(RangeProperty, 'threshold', .5, .1, 1)
    self.count = self.Register(ResettableValue, 'count', 0, persistent=True)
    self.direction = self.Register(EnumProperty, 'direction', 'port', ['port', 'starboard'])
    self.current_direction = 'port' # so user can't change while tacking
    self.direction_heel = 0
    self.direction_heel_time = 0

  def Register(self, _type, name, *args, **kwargs):
    return self.ap.server.Register(_type(*(['ap.tack.' + name] + list(args)), **kwargs))

  def process(self):
    t = time.time()

    # disengage cancels any tacking
    if not self.ap.enabled.value:
      self.state.set('none')

    # simple lowpass heel
    self.direction_heel = self.ap.boatimu.heel * .1 + self.direction_heel * .9

    if self.state.value == 'none': # not tacking
      # if we have wind data, use it to determine the tacking direction
      if self.ap.nmea.values['wind']['source'].value != 'none':
        if self.ap.nmea.values['wind']['direction'].value < 180:
            self.direction.update('starboard')
        else:
            self.direction.update('port')
      elif self.direction_heel_time > 30:
        # if we have tacked more than 30 seconds ago, and heeling
        # more than 10 degrees, use this to update tacking direction
        if self.direction_heel > 10:
            self.direction.update('starboard')
        elif self.direction_heel < -10:
            self.direction.update('port')

    # tacking initiated, enter waiting state
    if self.state.value == 'begin':
      self.begin_time = t
      self.current_direction = self.direction.value
      self.state.set('waiting')

    # waiting to tack, update timeout
    if self.state.value == 'waiting':
      remaining = self.delay.value - (t - self.begin_time)
      if remaining > 0:
        self.timeout.set(remaining)
      else:
        self.timeout.set(0)
        self.state.set('tacking')
        if 'wind' in self.ap.mode.value:
          self.tack_angle = 2*self.ap.command # opposite wind direction for wind  mode
        else:
          self.tack_angle = self.angle.value

    # tacking, moving rudder continuously at tack rate
    if self.state.value == 'tacking':
      mul = 1 if self.current_direction == 'port' else -1
      # command servo to turn boat at tack rate
      headingrate = ap.boatimu.SensorValues['headingrate_lowpass'].value
      headingraterate = ap.boatimu.SensorValues['headingraterate_lowpass'].value

      # for now very simple fixed PD filter on turn rate for tacking
      P = headingrate - mul*self.rate.value
      D = headingraterate
      command = .1*P + .1*D
      self.ap.servo.command(command)

      mul = 1 if self.current_direction == 'port' else -1
      current = mul*resolv(self.ap.command.value - self.ap.heading) / self.tack_angle

      # if we reach the threshold, tacking is complete, set the heading command
      # to the new value
      if current > self.threshold.value:
        command = self.ap.command - mul*tack_angle
        self.ap.command.set(resolv(command, 180))
        self.state.set('none')

    return self.state.value == 'tacking'
