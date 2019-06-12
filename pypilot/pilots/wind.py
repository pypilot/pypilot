#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

try:
  from autopilot import *
except:
  from pypilot.autopilot import *

global default_pilots

# the wind pilot does not require a compass, instead
# mixing gyro with wind directly
class WindPilot(AutopilotPilot):
  def __init__(self, ap):
    super(WindPilot, self).__init__('wind', ap)

    # create filters
    timestamp = self.ap.server.TimeStamp('ap')

    self.compass_wind_offset = HeadingOffset()
    self.gps_wind_offset = HeadingOffset()
    self.true_wind_wind_offset = HeadingOffset()

    self.heading = self.Register(SensorValue, 'heading', timestamp, directional=True)
    self.heading_error = self.Register(SensorValue, 'heading_error', timestamp)
    self.heading_error_int = self.Register(SensorValue, 'heading_error_int', timestamp)
    self.heading_error_int_time = time.time()

    # create simple pid filter
    self.gains = {}
    self.lastenabled = False

    def Gain(name, default, min_val, max_val, compute=None):
      if not compute:
        compute = lambda value : value * self.gains[name]['apgain'].value
      self.gains[name] = {'apgain': self.Register(AutopilotGain, name, default, min_val, max_val),
                          'sensor': self.Register(SensorValue, name+'gain', timestamp),
                          'compute': compute}

    def PosGain(name, default, max_val):
      Gain(name, default, 0, max_val)
        
    PosGain('P', .003, .02)  # position (heading error)
    PosGain('I', 0, .1)      # integral
    PosGain('D',  .1, 1.0)   # derivative (gyro)
    PosGain('DD',  .05, 1.0)  # position root


  def compute_offsets(self):
    # compute the difference from wind to other headings
    wind = self.ap.wind_direction.value
    compass = self.boatimu.SensorValues['heading_lowpass'].value

    d = .005
    self.compass_wind_offset.update(wind+compass, d)

    sensors = self.ap.sensors
    if sensors.gps.source.value != 'none':
      # difference from gps to wind
      self.gps_wind_offset.update(gps_track+wind)

      # compute offset between wind and true wind
      offset = resolv(self.ap.truewind - wind, self.true_wind_compass_offset.value)
      d = .05
      self.true_wind_wind_offset.update(offset, d)

    if self.compass_change:
      self.compass_wind_offset.value += self.compass_change

      
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
      
  def process(self, reset):
    ap = self.ap

    if type(ap.sensors.wind.value) == type(False):
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
    wind = ap.sensors.wind.value
        
    headingrate = ap.boatimu.SensorValues['headingrate_lowpass'].value
    headingraterate = ap.boatimu.SensorValues['headingraterate_lowpass'].value
    gain_values = {'P': self.heading_error.value,
                   'I': self.heading_error_int.value,
                   'D': headingrate,      
                   'DD': headingraterate}

    command = self.Compute(gain_values)

    if ap.enabled.value:
      ap.servo.command.set(command)
