#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from pypilot.autopilot import HeadingOffset, resolv, compute_true_wind
from pilot import AutopilotPilot, AutopilotGain
from pypilot.values import *
disabled = True

# the gps pilot requires gps
# it does not rely on the compass or calibration (unless in compass mode)
# and is therefore not affected by magnetic changes.   Even in compass mode this pilot
# will follow wind shifts and only hold the general compass course

class GPSPilot(AutopilotPilot):
  def __init__(self, ap):
    super(GPSPilot, self).__init__('gps', ap)

    # create filters
    self.wind_gps_offset = HeadingOffset()
    self.true_wind_gps_offset = HeadingOffset()

    # create simple pid filter
    self.gains = {}
        
    self.PosGain('P', .003, .02)  # position (heading error)
    self.PosGain('D',  .1, 1.0)   # derivative (gyro)
    self.PosGain('DD',  .05, 1.0)  # position root
    self.PosGain('FF',  .6, 3.0) # feed forward

    self.wind_gps_offset = HeadingOffset()
    self.true_wind_gps_offset = HeadingOffset()
    
  def compute_heading(self):
    # compute the difference from wind to other headings
    ap = self.ap
    sensors = ap.sensors
    gps_course = ap.sensors.gps.track.value

    # compute offset between wind and gps
    if sensors.wind.source.value != 'none':
      d = .005
      wind = self.ap.wind_direction.value
      self.wind_gps_offset.update(wind_direction + gps_course, d)
      true_wind = autopilot.compute_true_wind(ap.gps_speed, ap.wind_speed,
                                              ap.wind_direction.value)
      offset = resolv(true_wind + gps_course, self.true_wind_gps_offset.value)
      d = .05
      self.true_wind_compass_offset.update(offset, d)
      
    mode = ap.mode.value
    if mode == 'compass': # only if gps failed use compass
      compass = ap.boatimu.SensorValues['heading_lowpass'].value
      ap.heading.set(compass)
    if mode == 'gps':
      ap.heading.set(gps_course) # no filter?
    elif mode == 'wind':
      wind = resolv(self.wind_gps_offset.value - gps_course, 180)  
      ap.heading.set(wind)
    elif mode == 'true wind':
      true_wind = resolve(self.true_wind_gps_offset.value - gps_course, 180)
      ap.heading.set(true_wind)

  def best_mode(self, mode):
      sensors = self.ap.sensors
      gps_speed = sensors.gps.speed.value
      nogps = sensors.gps.source.value == 'none' or gps_speed < 1.2
      nowind = sensors.wind.source.value == 'none'
      if nogps:  # fallback to compass only if gps has failed boat speed < 1.2knots
        return 'compass'
      elif mode == 'compass' or nowind:
        return 'gps' # force gps mode overriding compass mode
      return mode
      
  def process(self):
    ap = self.ap

    # compute command
    headingrate = ap.boatimu.SensorValues['headingrate_lowpass'].value
    headingraterate = ap.boatimu.SensorValues['headingraterate_lowpass'].value
    gain_values = {'P': ap.heading_error.value,
                   'D': headingrate,      
                   'DD': headingraterate,
                   'FF': ap.heading_command_rate.value}

    command = self.Compute(gain_values)

    if ap.enabled.value:
      ap.servo.command.set(command)

pilot = GPSPilot
