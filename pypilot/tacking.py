#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from values import *
from resolv import *

class TackSensorLog(object):
  def __init__(self, threshold):
    self.log = []
    self.time = time.monotonic()
    self.threshold = threshold

  def update(self, value):
    t = time.monotonic()
    dt = t - self.time
    # limit update rate
    if dt < .25:
      return False

    self.time = t

    # if lagged by second or more, reset
    if dt > 1:
      self.log = []
      return False
    
    
    if len(self.log) < 20:
      self.log.append(value)
      return

    self.log = self.log[1:] + [value]
    port, starboard = True, True
    avg = 0
    for d in self.log:
      if d <= -self.threshold:
        starboard = False
      if d >= self.threshold:
        port = False
      avg += d

    avg /= len(self.log)
    if avg <= 0:
      starboard = False
    if avg >= 0:
      port = False

    if starboard:
      return 'starboard'
    if port:
      return 'port'
    return False


class Tack(object):
  def __init__(self, ap):
    self.ap = ap

    # tacking states
    # none - not tacking, normal ap operation
    # begin - control application sets this to initiate tack
    # waiting - waiting delay seconds before beginning to tack
    # tacking - rudder is moving at tack rate until threshold
    
    self.state = self.register(EnumProperty, 'state', 'none', ['none', 'begin', 'waiting', 'tacking'])
    self.timeout = self.register(Value, 'timeout', 0)

    self.delay = self.register(RangeSetting, 'delay', 0, 0, 60, 'sec')
    self.angle = self.register(RangeSetting, 'angle', 100, 10, 180, 'deg')
    self.rate = self.register(RangeSetting, 'rate', 20, 1, 100, 'deg/s')
    self.threshold = self.register(RangeSetting, 'threshold', 50, 10, 100, '%')
    self.count = self.register(ResettableValue, 'count', 0, persistent=True)
    self.direction = self.register(EnumProperty, 'direction', 'port', ['port', 'starboard'])
    self.current_direction = 'port' # so user can't change while tacking
    self.time = time.monotonic()

    self.wind_log = TackSensorLog(12)
    self.heel_log = TackSensorLog(7)

  def register(self, _type, name, *args, **kwargs):
    return self.ap.client.register(_type(*(['ap.tack.' + name] + list(args)), **kwargs))

  def process(self):
    t = time.monotonic()
    ap = self.ap

    # disengage cancels any tacking
    if not ap.enabled.value:
      self.state.update('none')

    if self.state.value == 'none': # not tacking
      # if we have wind data, use it to determine the tacking direction
      r = False
      if ap.sensors.wind.source.value != 'none':
        d = resolv(ap.sensors.wind.direction.value)
        r = self.wind_log.update(d)
      elif t-self.time > 30:
        r = self.heel_log.update(ap.boatimu.heel)

      if r:
        self.direction.update(r)

    # tacking initiated, enter waiting state
    if self.state.value == 'begin':
      self.time = t
      self.current_direction = self.direction.value
      self.state.update('waiting')

    # waiting to tack, update timeout
    if self.state.value == 'waiting':
      remaining = self.delay.value - (t - self.time)
      if remaining > 0:
        self.timeout.set(remaining)
      else:
        self.timeout.set(0)
        self.set.update('tacking')
        if 'wind' in ap.mode.value:
          self.tack_angle = 2*ap.command # opposite wind direction for wind  mode
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
      ap.servo.do_command(command)

      mul = 1 if self.current_direction == 'port' else -1
      heading_command = ap.heading_command.value
      current = mul*resolv(heading_command - ap.heading.value) / self.tack_angle
      # if we reach the threshold, tacking is complete, set the heading command
      # to the new value
      if current > self.threshold.value:
        heading_command -= mul*tack_angle
        ap.command.set(resolv(heading_command, 180))
        self.set.update('none')

    return self.state.value == 'tacking'
