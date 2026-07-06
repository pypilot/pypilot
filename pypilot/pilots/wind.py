#!/usr/bin/env python
#
#   Copyright (C) 2026 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

from pilot import AutopilotPilot

from pypilot.resolv import resolv
from pypilot.values import *

# the wind pilot does not require a compass but does require a wind sensor.
# it does not rely on the compass or calibration (unless in compass mode)
# and is not affected by magnetic changes.   Even in compass mode this pilot
# will follow wind shifts and only hold the general compass course

class WindPilot(AutopilotPilot):
    def __init__(self, ap):
        super().__init__('wind', ap)

        # create filters
        self.gps_wind_offset = self.register(HeadingOffset, 'gps_wind_offset')

        self.last_wind_speed = 0

        # create simple pid filter
        self.PosGain('P', .003, .02) # position (heading error)
#        self.PosGain('I', 0, .1)     # integral
        self.PosGain('D', .1, 1.0)   # derivative (gyro)
        self.PosGain('DD', .05, 1.0) # rate of derivative
        self.Gain('WG', 0, -.1, .1)  # wind gust

    def compute_heading(self):
        ap = self.ap
        sensors = self.ap.sensors
        wind = sensors.wind.filtered_direction.value

        if sensors.gps.source.value != 'none':
            gps_track  = sensors.gps.track.value
            # difference from gps to wind
            if ap.gps_speed > 1 and gps_track is not False:
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
            ap.heading.set(sensors.truewind.filtered_direction.value)
        elif mode == 'wind':
            ap.heading.set(wind)

    def best_pilot(self):
        return self if 'wind' in self.ap.modes.value else self.ap.pilots['basic']

    mode_fallbacks = {'nav': 'gps', 'gps': 'wind', 'true wind': 'wind'}
    def best_mode(self, mode):
        # Compass is still always listed in ap.modes. Wind can steer without it.
        if mode == 'compass' and self.ap.boatimu.SensorValues['compass'].value is False:
            return 'wind'
        return super().best_mode(mode)

    def process(self):
        ap = self.ap

        # compute command
        headingrate = ap.boatimu.SensorValues['headingrate_lowpass'].value
        headingraterate = ap.boatimu.SensorValues['headingraterate_lowpass'].value
        windgust = ap.sensors.wind.speed.value - self.last_wind_speed
        self.last_wind_speed = ap.sensors.wind.speed.value
        if ap.sensors.wind.direction.value < 0:
            windgust = -windgust

        gain_values = {'P': ap.heading_error.value,
#                       'I': ap.heading_error_int.value,
                       'D': headingrate,
                       'DD': headingraterate,
                       'WG': windgust}

        command = self.Compute(gain_values)

        if ap.enabled.value:
            ap.servo.command.command(command)

pilot = WindPilot

