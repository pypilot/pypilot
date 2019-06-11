#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from __future__ import print_function
import math
from signalk.values import *
from sensors import Sensor

class Rudder(Sensor):
    def __init__(self, server):
        super(Rudder, self).__init__(server, 'rudder')

        timestamp = server.TimeStamp('rudder')
        self.angle = self.Register(SensorValue, 'angle', timestamp)
        self.speed = self.Register(SensorValue, 'speed', timestamp)
        self.last = 0
        self.last_time = time.time()
        self.offset = self.Register(Value, 'offset', -100, persistent=True)
        self.scale = self.Register(Value, 'scale', 220, persistent=True)
        self.nonlinearity = self.Register(Value, 'nonlinearity',  0, persistent=True)
        self.calibration_state = self.Register(EnumProperty, 'calibration_state', 'idle', ['idle', 'reset', 'centered', 'starboard range', 'port range', 'auto gain'])
        self.calibration_raw = {}
        self.range = self.Register(RangeProperty, 'range',  45, 10, 100, persistent=True)
        self.lastrange = 0
        self.minmax = 0, 1
        self.autogain_state = 'idle'
        self.raw = 0

    def update_minmax(self):        
        scale = self.scale.value
        offset = self.offset.value
        range = self.range.value
        oldminmax = self.minmax
        self.minmax = (-range - offset)/scale, (range - offset)/scale

        if self.lastrange != self.range.value:
            # compute and update scale and offset if range changes
            nonlinearity = self.nonlinearity.value

            min, max = oldminmax
            B = scale -nonlinearity*(min+max)
            C = offset + nonlinearity*min*max

            min, max = self.minmax
            scale = B + nonlinearity*(min+max)
            offset = C - nonlinearity*min*max

            self.scale.update(scale)
            self.offset.update(offset)

        self.lastrange = self.range.value
        
    def calibration(self, command):
        if command == 'reset':
            self.nonlinearity.update(0.0)
            self.scale.update(220.0)
            self.offset.update(-100.0)
            self.update_minmax()
            self.calibration_raw = {}
            return

        elif command == 'centered':
            true_angle = 0
        elif command == 'port range':
            true_angle = self.range.value
        elif command == 'starboard range':
            true_angle = -self.range.value
        else:
            print('unhandled rudder_calibration', command)
            return
        
        # raw range 0 to 1
        self.calibration_raw[command] = {'raw': self.raw,
                                         'rudder': true_angle}
        scale = self.scale.value
        offset = self.offset.value
        nonlinearity = self.nonlinearity.value

        # rudder = (nonlinearity * raw + scale) * raw + offset
        p = []
        for c in ['starboard range', 'centered', 'port range']:
            if c in self.calibration_raw:
                p.append(self.calibration_raw[c])

        l = len(p)
        # 1 point, estimate offset
        if l == 1:
            rudder= p[0]['rudder']
            raw = p[0]['raw']
            offset = rudder - scale*raw - nonlinearity*(self.minmax[0]-raw)*(self.minmax[1]-raw)

        # 2 points, estimate scale and offset
        elif l == 2:
            rudder0, rudder1 = p[0]['rudder'], p[1]['rudder']
            raw0, raw1 = p[0]['raw'], p[1]['raw']
            if abs(raw1-raw0) > .001:
                scale = (rudder1 - rudder0)/(raw1 - raw0)
            offset = rudder1 - scale*raw1
            nonlinearity = 0
            
        # 3 points, estimate nonlinearity scale and offset
        if l == 3:
            rudder0, rudder1, rudder2 = p[0]['rudder'], p[1]['rudder'], p[2]['rudder']
            raw0, raw1, raw2 = p[0]['raw'], p[1]['raw'], p[2]['raw']

            if min(abs(raw1 - raw0), abs(raw2 - raw0), abs(raw2 - raw1)) > .001:
                scale = (rudder2 - rudder0)/(raw2 - raw0)
                offset = rudder0 - scale*raw0
                nonlinearity = (rudder1 - scale*raw1 - offset)/(raw0-raw1)/(raw2-raw1)
                self.minmax = raw0, raw1
                self.lastrange = 0 # force update minmax
            else:
                print('bad rudder calibration', self.calibration_raw)
            
        if abs(scale) <= .01:
            # bad update, trash an other reading
            print('bad servo rudder calibration', scale, nonlinearity)
            while len(self.calibration_raw) > 1:
                for c in self.calibration_raw:
                    if c != command:
                        del self.calibration_raw[c]
                        break
            return

        self.offset.update(offset)
        self.scale.update(scale)
        self.nonlinearity.update(nonlinearity)
        self.update_minmax()

    def invalid(self):
        return type(self.angle.value) == type(False)

    def poll(self):
        if self.lastrange != self.range.value:
            self.update_minmax()
        
        if self.calibration_state.value == 'idle':
            return

        if self.calibration_state.value == 'auto gain':
            def idle():
                self.autogain_state='idle'
                self.calibration_state.set('idle')

            t = time.time()
            if self.autogain_state=='idle':
                self.gain.set(1)
                self.autogain_state='fwd'
                self.autogain_movetime = t

            # must have rudder readings
            if type(self.value) == type(False):
                idle()

            rng = self.range.value
            #print(self.autogain_state, self.angle.value, rng)

            if self.autogain_state=='fwd':
                self.command.set(1)
                if abs(self.angle.value) >= rng:
                    self.autogain_state='center'
                    self.autogain_time = t

            if self.autogain_state=='center':
                self.command.set(-1)
                if abs(self.angle.value) < rng - 1:
                    self.autogain_state='rev'
                                        
            if self.autogain_state=='rev':
                self.command.set(-1)
                if abs(self.value) >= rng:
                    dt = time.time() - self.autogain_time
                    #print('hardover', dt, 'with', rng/dt, 'deg/s')
                    # 5 deg/s is gain of 1

                    gain = min(max(5*dt/rng, .5), 2)
                    if self.angle.value < 0:
#                            print('negative gain detected')
                        gain = -gain
                    self.gain.set(gain)
                    idle()

            if self.current.value:
                self.autogain_movetime = t

            if t - self.autogain_movetime > 3:
                print('servo rudder autogain failed')
                idle()
        else: # perform calibration
            self.calibration(self.calibration_state.value)
            self.calibration_state.set('idle')

    def update(self, data):
        if not data:
            self.angle.update(False)
            return
        
        self.raw = data['angle']
        if math.isnan(self.raw):
            self.angle.update(False)
            return

        # rudder = ((nonlinearity*self.raw + 1  + offset)*self.raw + offset)*scale
        #angle = ((self.nonlinearity.value*self.raw + 1)*self.raw)*self.scale.value + self.offset.value
        scale = self.scale.value
        offset = self.offset.value
        nonlinearity = self.nonlinearity.value
        raw = self.raw
        
        angle = scale*raw + offset + nonlinearity*(self.minmax[0]-raw)*(self.minmax[1]-raw)
        angle = round(angle, 2) # 2 decimal for rudder angle is enough
        self.angle.set(angle)

        t = time.time()
        dt = t - self.last_time

        if dt > 1:
            dt = 1
        if dt > 0:
            speed = (self.angle.value - self.last) / dt
            self.last_time = t
            self.last = self.angle.value
            self.speed.set(.9*self.speed.value + .1*speed)

    def reset(self):
        self.angle.set(False)
