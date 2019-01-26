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
    self.state = self.Register(EnumProperty, 'state', '', ['', 'begin', 'waiting', 'tacking', 'finishing'])
    self.timeout = self.Register(Value, 'timeout', 0)
    self.wait_real_time = 0
    self.delay = self.Register(RangeProperty, 'delay', 0, 0, 60)
    self.angle = self.Register(RangeProperty, 'angle', 100, 10, 180)
    self.speed = self.Register(RangeProperty, 'speed', 80, 10, 100)
    self.complete = self.Register(RangeProperty, 'complete', .5, 0, 1)
    self.count = self.Register(ResettableValue, 'count', 0, persistent=True)
    self.direction = self.Register(EnumProperty, 'direction', 'port', ['port', 'starboard'])
    self.current_direction = 'port' # so user can't change while tacking
    self.direction_heel = 0
    self.direction_heel_time = 0

  def Register(self, _type, name, *args, **kwargs):
    return self.ap.server.Register(_type(*(['ap.tack.' + name] + list(args)), **kwargs))

  def process(self):
    t = time.time()

    # simple lowpass heel
    self.direction_heel = self.ap.boatimu.heel * .1 + self.direction_heel * .9

    if self.state.value == '': # not tacking
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
      self.timeout.set(self.delay.value)
      self.wait_real_time = t
      self.current_direction = self.direction.value
      self.state.set('waiting')

    # waiting to tack, update timeout
    if self.state.value == 'waiting':
      remaining = self.delay.value - self.timeout.value + t
      if remaining > 0:
        self.timeout.set(remaining)
      else:
        self.state.set('tacking')
        if 'wind' in self.ap.mode.value:
          command = -self.ap.command
        elif self.current_direction == 'port':
          command = self.ap.command - self.angle
        else:
          command = self.ap.command + self.angle

        self.complete_angle = resolv(self.complete*(self.ap.command + command))
        self.ap.command.set(resolv(command, 180))

    # tacking, moving rudder continuously at tack speed
    if self.state.value == 'tacking':
      mul = 1 if self.current_direction == 'port' else -1
      self.ap.servo.command(mul*self.speed)
      if mul*resolv(self.ap.boatimu.heading - self.complete_angle) > 0:
          self.state.set('finishing')

    # reached complete angle, move rudder the opposite direction
    if self.state.value == 'finishing':
      mul = 1 if self.current_direction == 'port' else -1
      if mul*resolv(self.ap.boatimu.heading - self.ap.command.value) > 0:
          self.ap.servo.command(-mul*self.speed)
      else:
          self.state.set('')
          self.tacks.set(self.tacks.value + 1)

    # in these two states, take over servo control
    return self.state.value == 'tacking' or self.state.value == 'finishing'
