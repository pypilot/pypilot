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
from nmea import Nmea
import servo

def resolv(angle, offset=0):
    while offset - angle > 180:
        angle += 360
    while offset - angle < -180:
        angle -= 360
    return angle

def minmax(value, r):
  return min(max(value, -r), r)

class Filter(object):
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
        
class FilterHeading(object):
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
  def __init__(self, name):
    super(AutopilotBase, self).__init__()

    if True:  # disabled debugging from keyboard interrupt
        # setup all processes to exit on any signal
        def cleanup(signal_number, frame):
           # if signal_number == 2:
            raise KeyboardInterrupt # to get backtrace on all processes
            exit(1)

        import signal
        for s in range(1, 16):
            if s != 9 and s!=2:
                signal.signal(s, cleanup)
            pass

    serial_probe = serialprobe.SerialProbe()
#    self.server = SignalKServer()
    self.server = SignalKPipeServer()
    self.boatimu = BoatIMU(self.server)
    self.servo = servo.Servo(self.server, serial_probe)
    self.nmea = Nmea(self.server, serial_probe)
    self.version = self.Register(JSONValue, 'version', name + ' ' + 'pypilot' + ' ' + str(0.1))
    self.heading_command = self.Register(HeadingProperty, 'heading_command', 0)
    self.enabled = self.Register(BooleanProperty, 'enabled', False)
    self.mode = self.Register(EnumProperty, 'mode', 'compass', ['compass', 'gps', 'wind', 'true wind'], persistent=True)
    self.lastmode = False
    self.last_heading = False
    self.last_heading_off = self.boatimu.heading_off.value

    timestamp = self.server.TimeStamp('ap')
    self.heading = self.Register(SensorValue, 'heading', timestamp, directional=True)
    self.heading_error = self.Register(SensorValue, 'heading_error', timestamp)
    self.heading_error_int = self.Register(SensorValue, 'heading_error_int', timestamp)
    self.heading_error_int_time = time.time()

    self.gps_heading = self.Register(SensorValue, 'gps_heading', timestamp, directional=True)
    self.gps_speed = self.Register(SensorValue, 'gps_speed', timestamp)

    self.wind_direction = self.Register(SensorValue, 'wind_direction', timestamp, directional=True)
    self.wind_speed = self.Register(SensorValue, 'wind_speed', timestamp)

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

    self.starttime = time.time()
    self.times = 4*[0]
    # read initial value from imu as this takes time
#    while not self.boatimu.IMURead():
#        time.sleep(.1)

  def __del__(self):
      print 'closing autopilot'
      self.server.__del__()

      if self.watchdog_device:
          print 'close watchdog'
          self.watchdog_device.write('V')
          self.watchdog_device.close()

  def Register(self, _type, name, *args, **kwargs):
    return self.server.Register(_type(*(['ap/' + name] + list(args)), **kwargs))

  def run(self):
      self.lasttime = time.time()
      while True:
          self.ap_iteration()

  def ap_iteration(self):
      period = .1 # 10hz
      data = False
      t00 = time.time()
      for tries in range(14): # try 14 times to read from imu 
          data = self.boatimu.IMURead()
          if data:
              break
          time.sleep(period/10)

      if not data:
          print 'autopilot failed to read imu at time:', time.time()
      t0 = time.time()
      dt = t0 - self.lasttime
      self.lasttime = t0

      #compass_heading = self.boatimu.SensorValues['heading_lowpass'].value
      self.server.TimeStamp('ap', time.time()-self.starttime)
      compass_heading = self.boatimu.SensorValues['heading_lowpass'].value
      headingrate = self.boatimu.SensorValues['headingrate_lowpass'].value
      gps_heading_ap = self.gps_heading.value + headingrate*dt
      wind_direction_ap = self.wind_direction.value - headingrate*dt

      #update wind and gps offsets
      if self.nmea.values['gps']['source'].value != 'none':
          d = .01
          gps_speed = self.nmea.values['gps']['speed'].value
          self.gps_speed.set((1-d)*self.gps_speed.value + d*gps_speed)
          if gps_speed > 1: # don't update gps offset below 1 knot
              d = .001
              gps_heading = resolv(self.nmea.values['gps']['track'].value, gps_heading_ap)
              gps_heading_ap = (1-d)*gps_heading_ap * d*gps_heading
          self.gps_heading.set(resolv(compass_heading + self.gps_heading_offset, 180))
      if self.nmea.values['wind']['source'].value != 'none':
          d = .01
          wind_speed = self.nmea.values['wind']['speed'].value
          self.wind_speed.set((1-d)*self.wind_speed.value + d*wind_speed)
          # weight wind direction more with higher wind speed
          d = .01*math.log(self.wind_speed.value/5.0 + .2)
          if d > 0: # below 4 knots of wind, can't even use it
              wind_direction = resolv(self.nmea.values['wind']['direction'].value, wind_direction_ap)
              wind_direction_ap = (1-d)*wind_direction_ap + d*wind_direction

      self.gps_heading.set(resolv(gps_heading_ap, 180))
      self.wind_direction.set(resolv(wind_direction_ap, 180))

      if self.mode.value == 'true wind':
          # for true wind, we must have both wind and gps
          if self.nmea.values['wind']['source'].value == 'none':
              self.mode.set('gps')
          elif self.nmea.values['gps']['source'].value == 'none':
              self.mode.set('wind')

          wind_speed = self.wind_speed.value
          if wind_speed < 3: # below 3 knots wind speed, not reliable
              self.mode.set('wind')

          rd = math.radians(self.wind_direction.value)
          windv = wind_speed*math.sin(rd), wind_speed*math.cos(rd)
          truewindv = windv[0], windv[1] - self.gps_speed.value
          truewindd = math.degrees(math.atan2(*truewindv))
          self.heading.set(resolv(truewindd, 180))
      if self.mode.value == 'wind':
        # if wind sensor drops out, switch to compass
        if self.nmea.values['wind']['source'].value == 'none':
            self.mode.set('compass')
        self.heading.set(self.wind_direction.value)
      if self.mode.value == 'gps':
        # if gps drops out or speed too slow, switch to compass
        if self.nmea.values['gps']['source'].value == 'none' or \
           self.gps_speed.value < 1:
            self.mode.set('compass')
        self.heading.set(self.gps_heading.value)
      if self.mode.value == 'compass':
          if data:
              if 'calupdate' in data and self.last_heading:
                  # with compass calibration updates, adjust the autopilot heading_command
                  # to prevent actual course change
                  self.last_heading = resolv(self.last_heading, data['heading'])
                  heading_off = data['heading'] - headingrate*dt - self.last_heading
                  self.heading_command.set(resolv(self.heading_command.value + heading_off, 180))
              self.last_heading = data['heading']
          # if heading offset alignment changed, keep same course
          if self.last_heading_off != self.boatimu.heading_off.value:
              self.last_heading_off = resolv(self.last_heading_off, self.boatimu.heading_off.value)
              heading_off = self.boatimu.heading_off.value - self.last_heading_off
              self.heading_command.set(resolv(self.heading_command.value + heading_off, 180))
              self.last_heading_off = self.boatimu.heading_off.value
          self.heading.set(compass_heading)

      if self.enabled.value:
          if self.mode.value != self.lastmode: # mode changed?
              err = self.heading_error.value
              if 'wind' in self.mode.value:
                  err = -err
              self.heading_command.set(resolv(self.heading.value - err, 180))
          self.runtime.update()
          self.servo.servo_calibration.stop()

      # filter the incoming heading and gyro heading
      # error +- 60 degrees
      heading = self.heading.value
      heading_command = self.heading_command.value
      err = minmax(resolv(heading - heading_command), 60)
      # since wind direction is where the wind is from, the sign is reversed
      if 'wind' in self.mode.value:
          err = -err
      self.heading_error.set(err)

      t = time.time()
      self.server.TimeStamp('ap', t)
      dt = t - self.heading_error_int_time
      dt = max(min(dt, 1), 0) # ensure dt is from 0 to 1
      self.heading_error_int_time = t
      # int error +- 1, from 0 to 500 deg/s
      self.heading_error_int.set(minmax(self.heading_error_int.value + \
                                        (self.heading_error.value/500)*dt, 1))
      self.process_imu_data() # implementation specific process

      # servo can only disengauge under manual control
      self.servo.force_engauged = self.enabled.value
      self.lastmode = self.mode.value

      t1 = time.time()
      if t1-t0 > period/2:
          print 'Autopilot routine is running too _slowly_', t1-t0, period/2

      self.servo.poll()
      t2 = time.time()
      if t2-t1 > period/2:
          print 'servo is running too _slowly_', t2-t1

      self.nmea.poll()

      t4 = time.time()
      if t4 - t2 > period/2:
          print 'nmea is running too _slowly_', t4-t2

      self.server.HandleRequests()
      t5 = time.time()
      if t5 - t4 > period/2:
          print 'server is running too _slowly_', t5-t4

      times = t1-t0, t2-t1, t4-t2
      #self.times = map(lambda x, y : .975*x + .025*y, self.times, times)
      #print 'times', map(lambda t : '%.2f' % (t*1000), self.times)
      
      if self.watchdog_device:
          self.watchdog_device.write('c')

      while True:
          dt = period - (time.time() - t00)
          if dt <= 0:
              break
          time.sleep(dt)


if __name__ == '__main__':
  print 'You must run an actual autopilot implementation, eg: simple_autopilot.py'
