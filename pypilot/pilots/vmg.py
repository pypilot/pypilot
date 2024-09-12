#!/usr/bin/env python3
#
#   Copyright (C) 2022 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from pilot import AutopilotPilot
disabled = True

from resolv import resolv
from pypilot.values import *

# the vmg pilot optimizes velocity made good over tacks or jibes
# once it has sufficient data on both tacks, it can determine the optimal
# angle to sail on that particular tack and adjust to it to get the best speed
# toward the direction of both tacks
#
# gps is required and an estimation of true wind over water is determined
# as well as the current speed and direction


class vmgTable(object):
    def __init__(self):
        self.t = 0
        self.reset()

    def reset(self):
        self.command = 400 # impossible, will test false comparison
        self.table = {}

        self.previous = 400 # heading from other tack
        self.previous_table = {}


    def update_time(self):
        t = time.monotonic()
        if t - self.t > 0.5:  # vmg pilot switched off, mode changed, etc
            self.reset()

        self.t = t
        
    def add_measurement(self, heading, command, speed, track):
        t = time.monotonic()
        dt = t - self.current_command_time
        if dt < 60:
            return # ignore first minute on a new command
        
        if command != self.command:
            self.table = {}

            diff = resolv(command - self.command)

            if diff > 60: # 
                self.previous = self.command
                self.previous_table = self.table
            self.current_command_time = t


        headingi = int(round(heading))

        # convert gps speed/track into vector
        rtrack = math.radians(track)
        vn = speed*math.cos(rtrack)
        ve = speed*math.sin(rtrack)
        t = time.monotonic()
        
        if not headingi in self.table:
            vnt, vet, count, tt = vn, ve, 1, t
        else:
            vnt, vet, count, tt = self.table[headingi]
            if t - tt > 30:
                vnt, vet, count, tt = vn, ve, 1, t

        # put current speed and track into table
        vnt = vn/(count + 1) + vnt/count
        vet = ve/(count + 1) + vet/count
        count += 1

        self.table[headingi] = vnt, vet, count, t


    def updated_command(self, heading_command):
        t = time.monotonic()
        dt = t - self.current_command_time

        if dt > 120: # after 2 minutes allow course adjustment
            if self.table and self.previous_table:
                # find to optimal direction using the tables

                pass


class VMGPilot(AutopilotPilot):
    def __init__(self, ap, name='vmg'):
        super(VMGPilot, self).__init__(name, ap)

        # create extended pid filter
        self.PosGain('P', .003, .02)   # position (heading error)
        self.PosGain('D',  .09, .8)   # derivative (gyro)
        self.PosGain('DD',  .075, 0.8) # rate of derivative

        # for each mode, build up a table
        self.vmg = {}
        for mode in ap.mode.info['choices']:
            self.vmg[mode] = vmgTable()

        self.noise = 1


    def process(self):
        t = time.monotonic()
        ap = self.ap
        vmg = self.vmg[ap.mode.value]

        vmg.update_time()

        if ap.sensors.wind.source.value == 'none':
            ap.pilot.set('basic') # fall back to basic pilot if gps input fails
            return
        
        # compute command
        headingrate = ap.boatimu.SensorValues['headingrate_lowpass'].value
        headingraterate = ap.boatimu.SensorValues['headingraterate_lowpass'].value
        #reactive_value = self.servocommand_queue.take(t - self.reactive_time.value)
        #self.reactive_value.update(reactive_value)
    
        gain_values = {'P': ap.heading_error.value,
                       'D': headingrate,      
                       'DD': headingraterate}

        command = self.Compute(gain_values)

        if not ap.enabled.value:
            return
        ap.servo.command.command(command)
        
        # log into table
        accel = ap.boatimu.SensorValues['accel'].value
        noise = vector.dist(accel, self.accel)
        self.accel = accel
        self.noise = .1*noise + .9*self.noise

        # for now, only  use measurements taken without much acceleration,
        # would have to estimate acceleration compensation for polar speed otherwise
        if self.noise < .01: # good enough
            # now log our speed over time and compute

            vmg.add_measurement(ap.heading.value, ap.heading_comand.value, ap.sensors.gps.filtered.speed.value, ap.sensors.gps.filtered.track.value)

        vmg.update_command(ap.heading_command)
            

pilot = VMGPilot
