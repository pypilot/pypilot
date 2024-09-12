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

# the wind pilot does not require a compass but does require a wind sensor.
# it does not rely on the compass or calibration (unless in compass mode)
# and is not affected by magnetic changes.   Even in compass mode this pilot
# will follow wind shifts and only hold the general compass course

class WindPilot(AutopilotPilot):
  def __init__(self, ap):
    super(WindPilot, self).__init__('wind', ap)

    # create filters
    self.gps_wind_offset = HeadingOffset()

    self.last_wind_speed = 0
    
    # create simple pid filter
    self.PosGain('P', .003, .02) # position (heading error)
    self.PosGain('I', 0, .1)     # integral
    self.PosGain('D', .1, 1.0)   # derivative (gyro)
    self.PosGain('DD', .05, 1.0) # rate of derivative
    self.Gain('WG', 0, -.1, .1)  # wind gust

  def compute_heading(self):
    ap = self.ap
    # compute the difference from wind to other headings
    compass = self.ap.boatimu.SensorValues['heading_lowpass'].value

    sensors = self.ap.sensors
    wind = sensors.wind.direction.value

    if sensors.gps.source.value != 'none':
      gps_track  = sensors.gps.track.value
      # difference from gps to wind
      if ap.gps_speed > 1:
        d = .005*math.log(ap.gps_speed + 1)
        self.gps_wind_offset.update(wind + gps_track, d)

    mode = ap.mode.value
    if mode == 'compass':
      # compute compass from the wind. this causes the boat
      # to follow wind shifts with an overall average compass course
      compass = resolv(ap.wind_compass_offset.value - wind, 180)
      ap.heading.set(compass)
    elif mode == 'gps' or mode == 'nav':
      gps = resolv(self.gps_wind_offset.value - wind, 180)
      ap.heading.set(gps)
    elif mode == 'true wind':
      if ap.true_wind_sensor.value == 'water':
          boat_speed = sensors.water.speed
      elif ap.true_wind_sensor.value == 'gps':
          boat_speed = ap.gps_speed
      else:
          boat_speed = 0
        
      true_wind = TrueWind.compute_true_wind_direction(boat_speed, sensors.wind.speed, wind)
      ap.heading.set(true_wind)

    elif mode == 'wind':
      ap.heading.set(wind)

  def best_mode(self, mode):
      sensors = self.ap.sensors
      nocompass = self.ap.boatimu.SensorValues['compass'] == False
      nogps = sensors.gps.source.value == 'none'
      nowater = sensors.water.source.value == 'none'

      if mode == 'compass':
        if nocompass:
          return 'wind'
      else:
        if nogps and nowater: # need one of these for true wind
          return 'wind'

      return mode

      
  def process(self):
    ap = self.ap

    if ap.sensors.wind.source.value == 'none':
        ap.pilot.set('basic') # fall back to basic pilot if wind input fails
        return
    
    # compute command
    headingrate = ap.boatimu.SensorValues['headingrate_lowpass'].value
    headingraterate = ap.boatimu.SensorValues['headingraterate_lowpass'].value
    windgust = ap.sensors.wind.speed - self.last_wind_speed
    self.last_wind_speed = ap.sensors.wind.speed
    if ap.sensors.wind.direction < 0:
        windgust = -windgust
    gain_values = {'P': self.heading_error.value,
                   'I': self.heading_error_int.value,
                   'D': headingrate,      
                   'DD': headingraterate,
                   'WG': windgust}

    command = self.Compute(gain_values)

    if ap.enabled.value:
        ap.servo.command.command(command)

pilot = WindPilot
