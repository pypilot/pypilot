#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# autopilot base handles reading from the imu (boatimu)

import sys, getopt, os

pypilot_dir = os.getenv('HOME') + '/.pypilot/'

sys.path.append('.')

import os.path
from signalk.server import *
from signalk.values import *
from boatimu import BoatIMU

import gpspoller
import nmea_bridge
import servo
import math


class Filter:
    def __init__(self,filtered, lowpass):
        self.filtered = filtered
        self.lowpass = lowpass
        self.lastfilter_time = time.time()
        
    def update(self, value):
        t = time.time()
        timediff = t - self.lastfilter_time
        self.lastfilter_time = t
        #        rate = self.lowpass.value * timediff
        rate = self.lowpass.value
        if rate > 1:
            rate = 1

        filtered = self.filtered.value
        self.filtered.set(filtered + rate * (value-filtered))
        
class FilterHeading():
  def __init__(self, *args, **keywords):
    super(FilterHeading, self).__init__(*args, **keywords)
    self.variables = []

    def update(self, value):
        # detect passing the wrong way around
        if value - self.value > 180:
            self.value = self.value + 360
        elif self.value - value > 180:
            self.value = self.value - 360
        super(FilterHeading, self).update(value)

class AutopilotGain(RangeProperty):
  def __init__(self, *cargs):
      super(AutopilotGain, self).__init__(*cargs)

  def type(self):
      d = super(AutopilotGain, self).type()
      d['AutopilotGain'] = True
      return d


class AutopilotBase(object):
  def __init__(self, *args, **keywords):
    super(AutopilotBase, self).__init__(*args, **keywords)
    self.server = SignalKServer()
    self.boatimu = BoatIMU(self.server)
    self.servo = servo.Servo(self.server)
    self.gps = gpspoller.GpsPoller(self.server)

    self.heading_command = self.Register(HeadingProperty, 'heading_command', 0)
    self.mode = self.Register(EnumProperty, 'mode', 'disabled', ['disabled', 'compass', 'gps', 'wind'])
    self.last_heading = False

    self.gps_heading_offset = self.Register(Value, 'gps_heading_offset', 0)

    # read initial value from imu as this takes time
    while not self.boatimu.IMURead():
        time.sleep(.1)

  def __del__(self):
      print 'autopilot __del__ called'
      self.gps.process.terminate()
      self.boatimu.__del__()

  def Register(self, _type, name, *args):
    return self.server.Register(apply(_type, ['ap/' + name] + list(args)))

  def CreateFilter(self, name, initial, initial_constant):
    return Filter(self.Register(SensorValue, name+'_filtered', self.boatimu, initial),
                  self.Register(RangeProperty, name+'_lp_constant', initial_constant, 0, 1))
      
  def run(self):
          while True:
              self.ap_iteration()

  def ap_iteration(self):
      period = .1
      data = False
      while True:
          t0 = time.time()
          data = self.boatimu.IMURead()
          if data:
              break
          time.sleep(period/2)
              
      dt1 = time.time() - t0

      if 'calupdate' in data and self.last_heading:
          # with compass calibration updates, adjust the autopilot heading_command
          # to prevent actual course change

          heading_off = data['heading'] - self.last_heading
          new_command = self.heading_command.value + heading_off
          while new_command > 360:
              new_command -= 360
          while new_command < 0:
              new_command += 360
          self.heading_command.set(new_command)

      dt2 = time.time() - t0
      self.last_heading = data['heading']

      # calibration or other mode, disable autopilot
      if self.servo.drive.value == 'raw':
          self.mode.set('disabled')

      dt3 = time.time() - t0
      magnetic_heading_command = self.heading_command.value
      self.gps.poll()
      if self.gps.fix and self.gps.fix.speed > 1:
          track = self.gps.fix.track
          heading = self.boatimu.SensorValues['heading_lowpass'].value
          d = .01
          self.gps_heading_offset.set((1-d)*self.gps_heading_offset.value + d*(track - heading))
          if self.mode.value == 'gps':
              magnetic_heading_command += self.gps_heading_offset.value

      if self.mode.value != 'disabled':
          self.process_imu_data(self.boatimu, magnetic_heading_command)

      t1 = time.time()
      if t1-t0 > period/2:
          print 'Autopilot routine is running too _slowly_', t1-t0, period/2

      self.servo.poll()
      self.servo.send_command()

      t2 = time.time()
      if t2-t1 > period/2:
          print 'servo is running too _slowly_', t2-t1
          
      t3 = time.time()
      if t3 - t2 > period/2:
          print 'gps is running too _slowly_', t2-t1
          
      self.server.HandleRequests(period - (t3 - t0))


if __name__ == '__main__':
  print 'You must run an actual autopilot implementation, eg: simple_autopilot.py'
