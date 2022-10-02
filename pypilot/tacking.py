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
            return

        self.time = t

        # if lagged by second or more, reset
        if dt > 1:
            self.log = []
            return

        if len(self.log) < 20:
            self.log.append(value)
            return

        self.log = self.log[1:] + [value]
        port, starboard = True, True
        avg = 0
        for d in self.log:
            if d <= self.threshold:
                starboard = False
            if d >= -self.threshold:
                port = False
            avg += d

        avg /= len(self.log)

        if starboard:
            return 'starboard'
        if port:
            return 'port'


class TackDirection(EnumProperty):
    def __init__(self, name):
        super(TackDirection, self).__init__(name, 'port', ['port', 'starboard'])
        self.auto = True

    def set(self, value):
        super(TackDirection, self).set(value)
        self.auto = False

    def auto_update(self, value):
        if self.auto and self.value != value:
            super(TackDirection, self).set(value)

    def toggle(self):
        super(TackDirection, self).set('port' if self.value == 'starboard' else 'starboard')
        self.auto = True # allow automatic tack direction again


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
        self.rate = self.register(RangeSetting, 'rate', 15, 1, 100, 'deg/s')
        self.threshold = self.register(RangeSetting, 'threshold', 50, 10, 100, '%')
        self.count = self.register(ResettableValue, 'count', 0, persistent=True)

        self.direction = self.register(TackDirection, 'direction')
        self.current_direction = 'port'  # so user can't change while tacking
        self.time = time.monotonic()

        self.wind_log = TackSensorLog(12)
        self.heel_log = TackSensorLog(5)
        self.tack_angle = self.angle.value

    def register(self, _type, name, *args, **kwargs):
        return self.ap.client.register(_type(*(['ap.tack.' + name] + list(args)), **kwargs))

    def process(self):
        t = time.monotonic()
        ap = self.ap

        # disengage cancels any tacking
        if not ap.enabled.value:
            self.state.update('none')

            # if we have wind data, use it to determine the tacking direction
            r = False
            if ap.sensors.wind.source.value != 'none':
                d = resolv(ap.sensors.wind.direction.value)
                r = self.wind_log.update(d)
            elif t - self.time > 30:
                r = self.heel_log.update(ap.boatimu.heel)

            if r:
                self.direction.auto_update(r)
            return # done

        # tacking initiated, enter waiting state
        if self.state.value == 'begin':
            self.time = t
            self.current_direction = self.direction.value
            self.state.update('waiting')

        # waiting to tack, update timeout
        if self.state.value == 'waiting':
            remaining = round(self.delay.value - (t - self.time), 1)
            if remaining > 0:
                self.timeout.set(remaining)
                return

            self.timeout.set(0)
            self.state.update('tacking')
            self.tack_angle = self.angle.value

            
        # tacking, moving rudder continuously at tack rate
        if self.state.value == 'tacking':
            # command servo to turn boat at tack rate
            command = ap.heading_command.value
            heading = ap.boatimu.SensorValues['heading_lowpass'].value
            headingrate = ap.boatimu.SensorValues['headingrate_lowpass'].value
            headingraterate = ap.boatimu.SensorValues['headingraterate_lowpass'].value

            if 'wind' in ap.mode.value:
                d = .5 - 2*heading / command
                tack_heading = -command
                direction = 1 if command < 0 else -1
            else:
                direction = 1 if self.current_direction == 'port' else -1
                tack_heading = command - direction * self.tack_angle
                d = direction * (command - resolv(heading, command)) / self.tack_angle

            # see if we passed the tack user defined tack threshold
            if 100*d > self.threshold.value:
                self.state.update('none')
                self.direction.toggle()
                ap.heading_command.set(tack_heading)
                return

            # for now very simple filter based on turn rate for tacking
            command = (headingrate + headingraterate/2)/self.rate.value + direction
            
            command = min(max(command, -1), 1) # do not cause integrator windup
            ap.servo.command.command(command)
            return True # ensure current pilot is overridden
