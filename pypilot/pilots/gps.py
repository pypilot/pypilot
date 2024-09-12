#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from pypilot.resolv import resolv
from pilot import AutopilotPilot, AutopilotGain
from pypilot.values import *
disabled = True

# the gps pilot requires gps
# it does not rely on the compass or calibration (unless in compass mode)
# and is therefore not affected by magnetic changes.   Even in compass mode this pilot
# will a gps course being more immune to magnetic distortions and only hold the general compass course

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
        offset = resolv(sensors.wind.wdirection + gps_course,
                        self.wind_gps_offset.value)
        self.wind_gps_offset.update(offset, sensors.wind.wfactor)
    if sensors.truewind.source.value != 'none':
        offset = resolv(sensors.truewind.wdirection + gps_course,
                        self.true_wind_gps_offset.value)
        self.true_wind_gps_offset.update(offset, sensors.truewind.wfactor)

    mode = ap.mode.value
    if mode == 'compass': # only if gps failed use compass
      compass = ap.boatimu.SensorValues['heading_lowpass'].value
      ap.heading.set(compass)
    if mode == 'gps' or mode == 'nav':
      ap.heading.set(gps_course) # no filter?
    elif mode == 'wind':
      wind = resolv(self.wind_gps_offset.value - gps_course, 180)  
      ap.heading.set(wind)
    elif mode == 'true wind':
      true_wind = resolve(self.true_wind_gps_offset.value - gps_course, 180)
      ap.heading.set(true_wind)

  def best_mode(self, mode):
      modes = self.ap.modes.value
      if not mode in modes:  # fallback to compass
          if 'gps' in modes:
              return 'gps'
          return 'compass'
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
          ap.servo.command.command(command)

pilot = GPSPilot
