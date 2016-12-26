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
    rtimu = RTIMU.RTIMU(s)    

    print("Using settings file " + SETTINGS_FILE + ".ini")
    if not os.path.exists(SETTINGS_FILE + ".ini"):
        print("Settings file does not exist, will be created")

    print("IMU Name: " + rtimu.IMUName())
      
    if not rtimu.IMUInit():
        print("IMU Init Failed")
        exit(1)

    # this is a good time to set any fusion parameters
    rtimu.setSlerpPower(.01)
    rtimu.setGyroEnable(True)
    rtimu.setAccelEnable(True)
    rtimu.setCompassEnable(True)

    poll_interval = rtimu.IMUGetPollInterval()

    c = 0
    calupdates = 0
    while True:
      t0 = time.time()
      if rtimu.IMURead():
        data = rtimu.getIMUData()

        if cal_queue.qsize() > 0:
          new_cal = cal_queue.get()
          s.CompassCalEllipsoidValid = True
          s.CompassCalEllipsoidOffset = new_cal
          #rtimu.resetFusion()
          calupdates+=1
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
        self.timestamp = time.time()

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
        self.set(AgeValue.readable_timespan(time.time() - self.timestamp))
        
    def strobe(self):
        self.timestamp = time.time()

class QuaternionProperty(Property):
    def __init__(self, name, initial):
        super(QuaternionProperty, self).__init__(name, initial)

    def set(self, value):
        return super(QuaternionProperty, self).set(quaternion.normalize(value))

class BoatIMU(object):
  def __init__(self, server, *args, **keywords):
    self.server = server
    self.heading_off = self.Register(RangeProperty, 'heading_offset', 0, 0, 360)

    self.loopfreq = self.Register(LoopFreqValue, 'loopfreq', 0)
    self.alignmentQ = self.Register(QuaternionProperty, 'alignmentQ', [1, 0, 0, 0])
    self.timestamp = 0

    self.runtime = self.Register(AgeValue, 'runtime')
    self.compass_calibration_age = self.Register(AgeValue, 'compass_calibration_age')
    self.SensorValues = {}

    self.compass_calibration = self.Register(Value, 'compass_calibration', [[0, 0, 0, 30], 0])

    self.compass_calibration_sigmapoints = self.Register(Value, 'compass_calibration_sigmapoints', False)

    self.imu_queue = multiprocessing.Queue()
    imu_cal_queue = multiprocessing.Queue()

    if self.load_calibration():
      imu_cal_queue.put(tuple(self.compass_calibration.value[0][:3]))
    
    self.imu_process = multiprocessing.Process(target=except_imu_process, args=(self.imu_queue,imu_cal_queue))
    self.imu_process.start()

    self.compass_auto_cal = MagnetometerAutomaticCalibration(imu_cal_queue)
    
    self.lastqpose = False

    self.FirstTimeStamp = False
    self.LastDataTimestamp = 0

    # average sensor data down to 20hz
    # this reduces bandwidth processing load.
    # we don't need higher frequency for boat motion do we??
    self.Period = .05 # 10hz
    self.DataBank = []
    self.heel = 0
    self.rollog = []

    self.calupdates = 0

    self.heading_lowpass3 = self.heading_lowpass3a = self.heading_lowpass3b = False
    
  def __del__(self):
    self.save_calibration()

  def Register(self, _type, name, *args):
    return self.server.Register(apply(_type, ['imu/' + name] + list(args)))
      
  def IMURead(self):
    if self.imu_queue.qsize() == 0:
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
    if data['timestamp'] - self.LastDataTimestamp < self.Period:
      return False
    
#    self.LastDataTimestamp += self.Period
    self.LastDataTimestamp = data['timestamp']

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

    self.compass_auto_cal.AddPoint(data['compass'] + vector.normalize(data['accel']))
    
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
    self.rollog.append([data[filtername]])
    if len(self.rollog) > 600/self.Period:
      self.rollog = self.rollog[1:]
    x = self.rollog

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

    llp = .18
    self.heading_lowpass3a = heading_filter(llp, data[filtername], self.heading_lowpass3a)
    self.heading_lowpass3b = heading_filter(llp, self.heading_lowpass3a, self.heading_lowpass3b)
    self.heading_lowpass3 = heading_filter(llp, self.heading_lowpass3b, self.heading_lowpass3)

    data['heading_lowpass'] = self.heading_lowpass3
    data['gyro'] = map(math.degrees, data['gyro'])
    data['timestamp'] = self.DataBank[len(self.DataBank)-1]['timestamp']

    self.DataBank = []

    dt = data['timestamp'] - self.timestamp
    self.timestamp = data['timestamp']

    for name in data:
      if not name in self.SensorValues:
        self.SensorValues[name] = self.Register(SensorValue, name, self)
      self.SensorValues[name].set(data[name])

    self.runtime.update()

    result = self.compass_auto_cal.UpdatedCalibration()
    if result:
      self.compass_calibration_sigmapoints.set(result[1])

      if result[0]:
        self.compass_calibration_age.strobe()
        self.compass_calibration.set(result[0])
        self.save_calibration()
    self.compass_calibration_age.update()

    return data

  def load_calibration(self):
    try:
      f = open(autopilot.pypilot_dir + 'boatimu_calibration.json', 'r')
    except:
      print 'boatimu_calibration.json doesn\'t exist, no calibration'
      return False
    cal = json.loads(f.readline())
    self.compass_calibration.set(cal['compass_calibration'])
    self.alignmentQ.set(cal['alignmentQ'])
    self.heading_off.set(cal['heading_off'])
    return True

  def save_calibration(self):
    cal = {'compass_calibration' : self.compass_calibration.value, \
           'alignmentQ' : self.alignmentQ.value, \
           'heading_off' : self.heading_off.value}
#    print 'saving calibration:', cal
    f = open(autopilot.pypilot_dir + 'boatimu_calibration.json', 'w')
    f.write(json.dumps(cal))

if __name__ == "__main__":
  server = SignalKServer()
  boatimu = BoatIMU(server)

  heading_lp = 0

  while True:
    data = boatimu.IMURead()
    if data:
      print 'pitch', data['pitch'], 'roll', data['roll'], 'heading', data['heading']

    server.HandleRequests(.002)
