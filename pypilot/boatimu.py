#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# Boat imu is built on top of RTIMU

# it is an enhanced imu with special knowledge of boat dynamics
# giving it the ability to auto-calibrate the inertial sensors


import os.path
from sys import stdout
import json, time, math, multiprocessing

import autopilot
from calibration_fit import MagnetometerAutomaticCalibration
import vector
import quaternion
from signalk.server import SignalKServer
from signalk.values import *

try:
  import RTIMU
except ImportError:
  print "RTIMU library not detected, please install it"

def imu_process(queue, cal_queue):
    SETTINGS_FILE = "RTIMULib"
    s = RTIMU.Settings(SETTINGS_FILE)
    s.FusionType = 1
    s.CompassCalValid = False
    s.CompassCalEllipsoidValid = True
    s.MPU9255AccelFsr = 0 # +- 2g
    s.MPU9255GyroFsr = 0 # +- 250 deg/s
    s.MPU9255GyroAccelSampleRate = 10
    s.MPU9255CompassSampleRate = 10

    s.GyroBiasValid = True
    s.GyroBias = (-.003, 0.021000, -.01)

    rtimu = RTIMU.RTIMU(s)    

    print("Using settings file " + SETTINGS_FILE + ".ini")
    if not os.path.exists(SETTINGS_FILE + ".ini"):
        print("Settings file does not exist, will be created")

    print("IMU Name: " + rtimu.IMUName())

    while True:
      if not rtimu.IMUInit():
        print("ERROR: IMU Init Failed, no inertial data available")
        time.sleep(3)

      # this is a good time to set any fusion parameters
      rtimu.setSlerpPower(.01)
      rtimu.setGyroEnable(True)
      rtimu.setAccelEnable(True)
      rtimu.setCompassEnable(True)

      poll_interval = rtimu.IMUGetPollInterval()

      c = 0
      calupdates = 0
      t1 = time.time()
      while True:
        t0 = time.time()
        if t0 - t1 > 2:
          break;
        
        if rtimu.IMURead():
          t1 = t0
          data = rtimu.getIMUData()

          if cal_queue.qsize() > 0:
            new_cal = cal_queue.get()
            s.CompassCalEllipsoidValid = True
            s.CompassCalEllipsoidOffset = new_cal
            #rtimu.resetFusion()
            calupdates+=1

          data['gyrobias'] = s.GyroBias
          data['calupdates'] = calupdates
          queue.put(data)
        
        dt = time.time() - t0
        t = poll_interval/1000.0 - dt

        if t > 0:
          time.sleep(t)

def except_imu_process(queue, cal_queue):
  try:
    imu_process(queue, cal_queue)
  except KeyboardInterrupt:
    print 'Keyboard interrupt, boatimu process exit'
    pass

class LoopFreqValue(Value):
    def __init__(self, name, initial):
        super(LoopFreqValue, self).__init__(name, initial)
        self.loopc = 0
        self.t0 = time.time()

    def strobe(self):
        self.loopc += 1
        if self.loopc == 10:
            t1 = time.time()
            self.set(self.loopc/(t1-self.t0))
            self.t0 = t1
            self.loopc = 0


class AgeValue(Value):
    def __init__(self, name):
        super(AgeValue, self).__init__(name, False)
        self.timestamp = os.times()[4]

    @staticmethod
    def readable_timespan(time):
        mods = [('s', 1), ('m', 60), ('h', 60), ('d', 24), ('y', 365.24)]
            
        def loop(i, mod):
            if i == len(mods) or (int(time / (mods[i][1]*mod)) == 0 and i > 0):
                return ''
            if i < len(mods) - 1:
                div = mods[i][1]*mods[i+1][1]*mod
                t = int(time%int(div))
            else:
                t = time
            return loop(i+1, mods[i][1]*mod) + (('%d' + mods[i][0] + ' ') % (t/(mods[i][1]*mod)))
        return loop(0, 1)

    def update(self):
        self.set(AgeValue.readable_timespan(os.times()[4] - self.timestamp))
        
    def strobe(self):
        self.timestamp = os.times()[4]

class QuaternionProperty(Property):
    def __init__(self, name, initial, **kwargs):
        super(QuaternionProperty, self).__init__(name, initial, **kwargs)

    def set(self, value):
        return super(QuaternionProperty, self).set(quaternion.normalize(value))


class BoatIMU(object):
  def __init__(self, server, *args, **keywords):
    self.server = server
    self.heading_off = self.Register(RangeProperty, 'heading_offset', 0, 0, 360, persistent=True)

    self.loopfreq = self.Register(LoopFreqValue, 'loopfreq', 0)
    self.alignmentQ = self.Register(QuaternionProperty, 'alignmentQ', [1, 0, 0, 0], persistent=True)
    self.alignmentCounter = self.Register(Property, 'alignmentCounter', 0)
    self.alignmentType = self.Register(EnumProperty, 'alignmentType', 'level', ['level', 'port', 'starbord'])
    self.last_alignmentCounter = False
    self.port_down = self.starboard_down = False
    
    self.timestamp = 0

    self.runtime = self.Register(AgeValue, 'runtime')
    self.compass_calibration_age = self.Register(AgeValue, 'compass_calibration_age')
    self.SensorValues = {}

    self.compass_calibration = self.Register(Value, 'compass_calibration', [[0, 0, 0, 30], 0], persistent=True)
    self.compass_calibration_sigmapoints = self.Register(Value, 'compass_calibration_sigmapoints', False)

    self.imu_queue = multiprocessing.Queue()
    imu_cal_queue = multiprocessing.Queue()

#    if self.load_calibration():
#      imu_cal_queue.put(tuple(self.compass_calibration.value[0][:3]))
    
    self.imu_process = multiprocessing.Process(target=except_imu_process, args=(self.imu_queue,imu_cal_queue))
    self.imu_process.start()

    self.compass_auto_cal = MagnetometerAutomaticCalibration(imu_cal_queue)
    
    self.lastqpose = False

    self.FirstTimeStamp = False

    self.DataBank = []
    self.heel = 0

    self.calupdates = 0

    self.heading_lowpass3 = self.heading_lowpass3a = self.heading_lowpass3b = False

    for name in ['timestamp', 'fusionQPose', 'accel', 'gyro', 'compass', 'gyrobias', 'heading_lowpass', 'pitch', 'roll', 'heading', 'pitchrate', 'rollrate', 'headingrate', 'heel', 'calupdate']:
        if not name in self.SensorValues:
            self.SensorValues[name] = self.Register(SensorValue, name, self)

    
  def __del__(self):
    self.imu_process.terminate()
    self.compass_auto_cal.process.terminate()

  def Register(self, _type, name, *args, **kwargs):
    return self.server.Register(_type(*(['imu/' + name] + list(args)), **kwargs))
      
  def IMURead(self):
    if self.imu_queue.qsize() == 0:
      if time.time() - self.loopfreq.t0 > 1 and self.loopfreq.value:
        self.loopfreq.set(0)
        for name in self.SensorValues:
          self.SensorValues[name].set(False)
      return False

    # flush queue
    while self.imu_queue.qsize() > 0:
      data = self.imu_queue.get()

    fixtime = True
    if fixtime: # override buggy imu library timestamp with current time here
      data['timestamp'] = time.time()
    
    if not self.FirstTimeStamp:
      self.FirstTimeStamp = data['timestamp']

    data['timestamp'] -= self.FirstTimeStamp
    if not fixtime:
      data['timestamp'] /= 1000000.0

    self.DataBank.append(data)

    self.loopfreq.strobe()
    
    data = {}

    def avgsensor(sensor, n=3):
      data[sensor] = [0]*len(self.DataBank[0][sensor])
      for i in range(n):
        for d in self.DataBank:
          data[sensor][i] += d[sensor][i]
        data[sensor][i] /= len(self.DataBank)

    avgsensor('accel')
    avgsensor('gyro')
    avgsensor('compass')

    avgsensor('gyrobias')
    
    # when the calibration updates, we cannot average fusion pose
    # so we just take the last one
    calupdates = self.DataBank[len(self.DataBank)-1]['calupdates']
    if self.calupdates != calupdates:
      data['fusionQPose'] = self.DataBank[len(self.DataBank)-1]['fusionQPose']
      data['calupdate'] = True
      self.calupdates = calupdates
    else:
      avgsensor('fusionQPose', 4)

    # apply alignment calibration
    data['fusionQPose'] = list(quaternion.multiply(data['fusionQPose'], self.alignmentQ.value))

    # count down to alignment
    if (self.alignmentCounter.value, self.alignmentType.value) != self.last_alignmentCounter:
      self.alignmentPose = [0, 0, 0, 0]

    if self.alignmentCounter.value > 0:
      self.alignmentPose = map(lambda x, y : x + y, self.alignmentPose, data['fusionQPose'])
      self.alignmentCounter.set(self.alignmentCounter.value-1)

      if self.alignmentCounter.value == 0:
        self.alignmentPose = quaternion.normalize(self.alignmentPose)
        down = quaternion.rotvecquat([0, 0, 1], quaternion.conjugate(self.alignmentPose))
        alignment = []
        if self.alignmentType.value == 'level':
          alignment = quaternion.vec2vec2quat([0, 0, 1], down)
          alignment = quaternion.multiply(self.alignmentQ.value, alignment)
        elif self.alignmentType.value == 'starboard':
          self.starboard_down = list(down)
        elif self.alignmentType.value == 'port':
          self.port_down = list(down)
        
        if self.starboard_down and self.port_down:
          ang = math.atan2(self.port_down[0] - self.starboard_down[0], \
                           self.starboard_down[1] - self.port_down[1])
          alignment = quaternion.angvec2quat(ang, [0, 0, 1])
          print 'downs:', self.starboard_down, self.port_down, ang*180/math.pi, alignment
          alignment = quaternion.multiply(self.alignmentQ.value, alignment)

        if len(alignment):
          print 'self.alignmentQ', self.alignmentQ.value, alignment
          self.alignmentQ.set(alignment)

    self.last_alignmentCounter = self.alignmentCounter.value, self.alignmentType.value

    self.compass_auto_cal.AddPoint(data['compass'] + vector.normalize(data['accel']))
    if vector.norm(data['accel']) == 0:
      print 'vector n', data['accel']
    
    data['roll'], data['pitch'], data['heading'] = map(math.degrees, quaternion.toeuler(data['fusionQPose']))

    data['heading'] -= self.heading_off.value

    if data['heading'] < 0:
      data['heading'] += 360

    gyro_q = quaternion.rotvecquat(data['gyro'], data['fusionQPose'])
    data['pitchrate'] = math.degrees(gyro_q[0])
    data['rollrate'] = math.degrees(gyro_q[1])
    data['headingrate'] = math.degrees(gyro_q[2])

    data['heel'] = self.heel = data['roll']*.05 + self.heel*.95

    filtername = 'heading'
    def heading_filter(lp, a, b):
      if not a:
        return b
      if not b:
        return a
      if a - b > 180:
        a -= 360
      elif b - a > 180:
        b -= 360
      result = llp*a + (1-llp)*b
      if result < 0:
        result += 360
      return result

    # third order lowpass
    llp = .18
    self.heading_lowpass3a = heading_filter(llp, data[filtername], self.heading_lowpass3a)
    self.heading_lowpass3b = heading_filter(llp, self.heading_lowpass3a, self.heading_lowpass3b)
    self.heading_lowpass3 = heading_filter(llp, self.heading_lowpass3b, self.heading_lowpass3)

    data['heading_lowpass'] = self.heading_lowpass3
    data['gyro'] = map(math.degrees, data['gyro'])
    data['gyrobias'] = map(math.degrees, data['gyrobias'])
    data['timestamp'] = self.DataBank[len(self.DataBank)-1]['timestamp']

    self.DataBank = []

    dt = data['timestamp'] - self.timestamp
    self.timestamp = data['timestamp']

    for name in data:
      self.SensorValues[name].set(data[name])

    self.runtime.update()

    result = self.compass_auto_cal.UpdatedCalibration()
    if result:
      self.compass_calibration_sigmapoints.set(result[1])

      if result[0]:
        self.compass_calibration_age.strobe()
        self.compass_calibration.set(result[0])

    self.compass_calibration_age.update()

    return data

if __name__ == "__main__":
  server = SignalKServer()
  boatimu = BoatIMU(server)

  heading_lp = 0

  while True:
    data = boatimu.IMURead()
    if data:
      print 'pitch', data['pitch'], 'roll', data['roll'], 'heading', data['heading']

    server.HandleRequests(.002)
