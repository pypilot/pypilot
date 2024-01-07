#!/usr/bin/env python
#
#   Copyright (C) 2021 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from pilot import AutopilotPilot
from pypilot.resolv import resolv
#disabled = True

class AutotunePilot(AutopilotPilot):
  def __init__(self, ap):
    super(AutotunePilot, self).__init__('autotune', ap)

    # create simple pid filter
    self.PosGain('P', .003, .025)
    self.PosGain('D', .09, .5)
    self.PosGain('FF',  .6, 3.0) # feed forward

    self.p_search = .0015, .006, .0004
    self.d_search = .05, .18, .01
    self.search_angle = 0
    self.search_angle_change = 30 * 3.14/180 # 30 degrees
    self.search_time = 0
    self.search_count = 0
    self.cost = 0
    self.last_cost = 0
    self.last_cost_dt = 0

  def process(self):
    ap = self.ap
    headingrate = ap.boatimu.SensorValues['headingrate_lowpass'].value
    headingraterate = ap.boatimu.SensorValues['headingraterate_lowpass'].value
    
    gain_values = {'P': ap.heading_error.value,
                   'D': headingrate + headingraterate,
                   'FF': ap.heading_command_rate.value}

    command = self.Compute(gain_values)

    if ap.enabled.value:
        ap.servo.command.command(command)
    else:
        self.search_count = 0

    self.search_count+= 1
    P, D = self.gains['P']['apgain'], self.gains['D']['apgain']
    self.cost += ap.heading_error.value + P.value*4000 + D.value*20
    if self.search_count >= 600:
      self.search_count = 0
      t = time.monotonic()
      search_dt = t - self.search_time
      self.search_time = t

      # update after 600 iterations (30 seconds at 20hz)
      if search_dt < 600/self.boatimu.rate.value*1.05:
        # determine current error and bias toward lower gains slightly

        # determine the change in cost since last update
        cost_dt = cost - self.last_cost
        if cost_dt > 0:
          self.search_angle += 3.14  # reverse search direction
        elif cost_dt > self.last_cost_dt:
          self.search_angle_change = -self.search_angle_change  # turn in other direction if cost reduction is not favorable
        self.last_cost_dt = cost_dt

        # update search angle
        self.search_angle += self.search_angle_change
        self.search_angle = resolve(self.search_angle)

        # apply changes to gains
        Pval = P.value + self.p_search[2]*math.sin(self.search_angle)
        Dval = D.value + self.d_search[2]*math.cos(self.search_angle)

        # keep search space within bounds
        P.set(min(max(Pval, self.p_search[0]), self.p_search[1]))
        D.set(min(max(Dval, self.d_search[0]), self.d_search[1]))

      self.last_cost = cost
      self.cost = 0



pilot = AutotunePilot
