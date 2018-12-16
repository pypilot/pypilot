#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from signalk.values import Value
from autopilot import *

def FindOffset(queue):
  mindt = 0
  minresidual = 1e10
  for di in range(len(queue)/5):
    for i in range(len(queue)):
      j = i + di
      if j >= len(queue):
        continue

      x, y = queue.data[j][0]
      ys.append(y)
      xs.append(x)

    a, b = numpy.polyfit(xs, ys)
    residual = 0
    n = len(xs)
    for i in range(n):
      x = xs[i]
      y = ys[i]
      d = (x*a + b) - y
      residual += d*d # squared
    residual /= n
    
    if residual < minresidual:
      minresidual = residual
      mindt = queue.data[j][0][1] - queue.data[i][0][1]
  return mindt

'''
class DiffTable(Value):
  def __init__(self):
    super(Value, self).__init__('learning', False)
    
  def set(self, value):
    if value and self.value:
      diff = self.diff(value)
      self.value = value
      if diff:
        self.send('{"' + self.name + '": {"diff": "' + diff + '"}}')
        return
    else:
      self.value = value
    self.send()

  def diff(self, value):
    for
    '''

class GainTable(Value):
  prange = 10, 1
  drange = 10, 1
  def __init__(self, name):
    super(GainTable, self).__init__(name, "False")
    
    self.value = []
    P = .001
    D = .03
    for a in range(int(2*GainTable.prange[0]/GainTable.prange[1]+1)):
      p = a*GainTable.prange[1]-GainTable.prange[0]
      row = []
      for b in range(int(2*GainTable.drange[0]/GainTable.drange[1]+1)):
        d = b*GainTable.drange[1]-GainTable.drange[0]
        row.append(p*P + d*D)
      self.value.append(row)
    print 'value', self.value

  def type(self):
    return 'GainTable'
    
  def indexof(self, p, d):
    p = min(prange[0], max(-prange[0], p))
    d = min(drange[0], max(-drange[0], d))

    a = (p+prange[0])/prange[1]
    b = (d+drange[0])/drange[1]
    return a, b
      
  def gain(self, p, d):
    a, b = indexof(p, d)
    return self.value[a][b]

  def update(self, p, d, error):
    a, b = indexof(p, d)
    gain = self.value[a][b] + error
    self.value[a][b] = min(1, max(-1, gain))
    self.update(self.value)

class LearningPilot(AutopilotPilot):
  def __init__(self, ap):
    super(LearningPilot, self).__init__('learning', ap)
    # create filters
    timestamp = self.ap.server.TimeStamp('ap')

    self.heading_command_rate = self.Register(SensorValue, 'heading_command_rate', timestamp)

    timing_queue = TimedQueue(10)

    # create simple pid filter
    self.P = self.Register(AutopilotGain, 'P', .001, .0001, .01)
    self.D = self.Register(AutopilotGain, 'D', .03, .01, .1)

    self.dt = self.Register(SensorValue, 'dt', 0)

    self.GainTable = self.Register(GainTable, 'GainTable')

    self.lastenabled = False

  def process_imu_data(self):
    return
    ap = self.ap
    if ap.enabled.value != self.lastenabled:
      self.lastenabled = ap.enabled.value
      if ap.enabled.value:
        ap.heading_error_int.set(0) # reset integral
        
    # if disabled, we are done
    if not ap.enabled.value:
      return
    
    # filter the heading command to compute feed-forward gain
    heading_error = ap.heading_error.value,
    headingrate = ap.boatimu.SensorValues['headingrate'].value
    headingraterate = ap.boatimu.SensorValues['headingraterate'].value

    timing_queue.add((headingraterate, command, heading_error, headingrate))
    command = self.GainTable.gain(heading_error, headingrate)

    dt = FindOffset(timing_queue)
    self.dt.set(dt) 
    if dt:
      val = timing_queue.take(dt)
      if val:
        dd, c, p, d = val
        error = self.P.value*p + self.D.value*d
        self.GainTable.update(p, d, error)

    ap.servo.command.set(command)
