#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from autopilot import HeadingOffset, resolv
from pilot import AutopilotPilot, AutopilotGain
from signalk.values import *

# the wind pilot does not require a compass but does require a wind sensor.
# it does not rely on the compass or calibration (unless in compass mode)
# and is not affected by magnetic changes.   Even in compass mode this pilot
# will follow wind shifts and only hold the general compass course

class WindPilot(AutopilotPilot):
  def __init__(self, ap):
    super(WindPilot, self).__init__('wind', ap)

    # create filters
    self.compass_wind_offset = HeadingOffset()
    self.gps_wind_offset = HeadingOffset()
    self.true_wind_wind_offset = HeadingOffset()

    self.heading = self.Register(SensorValue, 'heading', directional=True)
    self.heading_error = self.Register(SensorValue, 'heading_error')
    self.heading_error_int = self.Register(SensorValue, 'heading_error_int')
    self.heading_error_int_time = time.time()

    # create simple pid filter
    self.gains = {}
    self.lastenabled = False
        
    self.PosGain('P', .003, .02)  # position (heading error)
    self.PosGain('I', 0, .1)      # integral
    self.PosGain('D',  .1, 1.0)   # derivative (gyro)
    self.PosGain('DD',  .05, 1.0)  # position root

  def compute_offsets(self):
    # compute the difference from wind to other headings
    wind = self.ap.wind_direction.value
    compass = self.ap.boatimu.SensorValues['heading_lowpass'].value

    d = .005
    self.compass_wind_offset.update(wind+compass, d)

    sensors = self.ap.sensors
    if sensors.gps.source.value != 'none':
      gps_track  = sensors.gps.track.value
      # difference from gps to wind
      self.gps_wind_offset.update(gps_track+wind, d)

      # compute offset between wind and true wind
      gps_speed = sensors.gps.speed.value
      wind_speed = sensors.wind.speed.value
      wind_direction = sensors.wind.direction.value
      rd = math.radians(wind_direction)
      windv = wind_speed*math.sin(rd), wind_speed*math.cos(rd)
      true_wind = math.degrees(math.atan2(windv[0], windv[1] - gps_speed))
      offset = resolv(true_wind - wind, self.true_wind_wind_offset.value)
      d = .05
      self.true_wind_wind_offset.update(offset, d)

    # compensate compass relative to wind offset
    if self.ap.compass_change:
      self.compass_wind_offset.value += self.ap.compass_change
      
  def compute_heading(self):
    self.compute_offsets()
    ap = self.ap
    wind = ap.wind_direction.value

    mode = ap.mode.value
    if mode == 'gps':
      # if gps drops out switch to wind mode
      if ap.sensors.gps.source.value == 'none':
        ap.mode_lost('wind')
      gps = resolv(ap.wind_compass_offset.value - wind, 180)
      ap.heading.set(gps)

    if mode == 'true wind':
      # for true wind, need gps
      if ap.sensors.gps.source.value == 'none':
        ap.mode_lost('wind')
      true_wind = resolv(ap.true_wind_wind_offset.value + wind, 180)
      ap.heading.set(true_wind)

    if mode == 'compass':
      # compute compass from the wind. this causes the boat
      # to follow wind shifts with an overall average compass course
      compass = resolv(self.compass_wind_offset.value - wind, 180)
      ap.heading.set(compass)

    if mode == 'wind':
      ap.heading.set(wind)

  def best_mode(self, mode):
      sensors = self.ap.sensors
      nocompass = self.ap.boatimu.SensorValues['compass'] == False
      nogps = sensors.gps.source.value == 'none'

      if mode == 'compass':
        if nocompass:
          return 'wind'
      else:
        if nogps:
          return 'wind'

      return mode
      
  def process(self, reset):
    ap = self.ap

    if ap.sensors.wind.source.value == 'none':
        ap.pilot.set('basic') # fall back to basic pilot if wind input fails
        return
    
    # if disabled, only bother to compute if a client cares
    if not ap.enabled.value: 
      compute = False
      for gain in self.gains:
        if self.gains[gain]['sensor'].watchers:
          compute = True
          break
      if not compute:
        return

    # compute command
    headingrate = ap.boatimu.SensorValues['headingrate_lowpass'].value
    headingraterate = ap.boatimu.SensorValues['headingraterate_lowpass'].value
    gain_values = {'P': self.heading_error.value,
                   'I': self.heading_error_int.value,
                   'D': headingrate,      
                   'DD': headingraterate}

    command = self.Compute(gain_values)

    if ap.enabled.value:
      ap.servo.command.set(command)

pilot = WindPilot
