#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# autopilot base handles reading from the imu (boatimu)

import sys, getopt, os
import math

pypilot_dir = os.getenv('HOME') + '/.pypilot/'

sys.path.append('.')

import os.path
from signalk.server import *
from signalk.pipeserver import SignalKPipeServer
from signalk.values import *
from boatimu import *

import serialprobe
from gpspoller import GpsPoller
from nmea_bridge import NmeaBridge
import wind, servo

def resolv(angle, offset=0):
    while offset - angle > 180:
        angle += 360
    while offset - angle < -180:
        angle -= 360
    return angle

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
      super(AutopilotGain, self).__init__(*cargs, persistent=True)

  def type(self):
      d = super(AutopilotGain, self).type()
      d['AutopilotGain'] = True
      return d

class AutopilotBase(object):
  def __init__(self, *args, **keywords):
    super(AutopilotBase, self).__init__(*args, **keywords)

    # setup all processes to exit on any signal
    def cleanup(signal_number, frame):
        exit(1)

    import signal
    for s in range(1, 16):
        if s != 9:
            signal.signal(s, cleanup)

    serial_probe = serialprobe.serialprobe()
    #self.server = SignalKServer()
    self.server = SignalKPipeServer()
    self.boatimu = BoatIMU(self.server)
    self.servo = servo.Servo(self.server, serial_probe)
    self.gps = GpsPoller(self.server, serial_probe)
    self.wind = wind.Wind(self.server, serial_probe)
    self.nmea_bridge = NmeaBridge(self.server, self.gps, self.wind)

    self.heading_command = self.Register(HeadingProperty, 'heading_command', 0)
    self.enabled = self.Register(BooleanProperty, 'enabled', False)
    self.mode = self.Register(EnumProperty, 'mode', 'compass', ['compass', 'gps', 'wind'], persistent=True)
    self.lastmode = False
    self.last_heading = False

    self.heading = self.Register(Value, 'heading', 0)

    self.gps_heading_offset = self.Register(Value, 'gps_heading_offset', 0)
    self.wind_heading_offset = self.Register(Value, 'wind_heading_offset', 0)

    self.runtime = self.Register(AgeValue, 'runtime') #, persistent=True)

    device = '/dev/watchdog0'
    self.watchdog_device = False
    try:
        self.watchdog_device = open(device, 'w')
    except:
        print 'failed to open special file', device, 'for writing:'
        print 'autopilot cannot strobe the watchdog'


    if os.system('sudo chrt -pf 1 %d 2>&1 > /dev/null' % os.getpid()):
      print 'warning, failed to make autopilot process realtime'

        
    # read initial value from imu as this takes time
#    while not self.boatimu.IMURead():
#        time.sleep(.1)

  def __del__(self):
      if self.gps.process:
          self.gps.process.terminate()
      self.nmea_bridge.process.terminate()
      if self.watchdog_device:
          print 'close watchdog'
          self.watchdog_device.write('V')
          self.watchdog_device.close()

  def Register(self, _type, name, *args, **kwargs):
    return self.server.Register(_type(*(['ap/' + name] + list(args)), **kwargs))

  def run(self):
      while True:
          self.ap_iteration()

  def ap_iteration(self):
      period = .1 # 10hz
      data = False
      # try 3 times to read data within the period
      for i in range(3):
          t0 = time.time()
          data = self.boatimu.IMURead()
          if data:
              break
          time.sleep(period/3)
          #self.server.HandleRequests(period/3)

      dt1 = time.time() - t0

      if data and 'calupdate' in data and self.last_heading:
          # with compass calibration updates, adjust the autopilot heading_command
          # to prevent actual course change

          heading_off = data['heading'] - self.last_heading
          new_command = self.heading_command.value + heading_off
          while new_command > 360:
              new_command -= 360
          while new_command < 0:
              new_command += 360
          self.heading_command.set(new_command)
      if data:
          self.last_heading = data['heading']
              
      dt2 = time.time() - t0

      # calibration or other mode, disable autopilot
      if not data or self.servo.rawcommand.value:
          self.enabled.update(False)

      dt3 = time.time() - t0
      
      #compass_heading = self.boatimu.SensorValues['heading_lowpass'].value
      compass_heading = self.boatimu.SensorValues['heading'].value
      if self.mode.value == 'compass':
          self.heading.set(compass_heading)
      elif self.mode.value == 'gps':
        if self.gps.speed.value > 1:
            offset = self.gps_heading_offset.value
            diff = resolv(compass_heading - self.gps.track.value, offset)
            d = .01
            self.gps_heading_offset.set(resolv((1-d)*offset + d*diff))
        self.heading.set(resolv(compass_heading - self.gps_heading_offset.value, 180))
      elif self.mode.value == 'wind':
        offset = self.wind_heading_offset.value
        diff = resolv(compass_heading - self.wind.direction.value, offset)
        d = .1
        d = 1 # for now, don't even use compass (raw wind)
        self.wind_heading_offset.set(resolv((1-d)*offset + d*diff))
        self.heading.set(resolv(compass_heading - self.wind_heading_offset.value, 180))

#        self.heading.set(self.wind.direction.value)

      if self.enabled.value:
          if self.mode.value != self.lastmode:
              self.heading_command.set(self.heading.value)
          self.process_imu_data(self.boatimu)

          self.runtime.update()

      self.lastmode = self.mode.value

      t1 = time.time()
      if t1-t0 > period/2:
          print 'Autopilot routine is running too _slowly_', t1-t0, period/2

      self.servo.poll()
      t12 = time.time()
      self.servo.send_command()

      t2 = time.time()
      if t2-t1 > period/2:
          print 'servo is running too _slowly_', t2-t1, t12-t1

      self.gps.poll()

      t3 = time.time()
      if t3 - t2 > period/2:
          print 'gps is running too _slowly_', t3-t2

      self.wind.poll()

      t4 = time.time()
      if t4 - t3 > period/2:
          print 'wind is running too _slowly_', t4-t3

      self.nmea_bridge.poll()

      t5 = time.time()
      if t5 - t4 > period/2:
          print 'nmea is running too _slowly_', t5-t4
          
      t = max(period - (t5 - t0), .02)
      self.server.HandleRequests(t)

      if self.watchdog_device:
          self.watchdog_device.write('c')

if __name__ == '__main__':
  print 'You must run an actual autopilot implementation, eg: simple_autopilot.py'
