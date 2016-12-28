#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# Boat imu is built on top of RTIMU

# it is an enhanced imu with special knowledge of boat dynamics
# giving it the ability to auto-calibrate the inertial sensors

import os
from sys import stdout
import json, time, math, multiprocessing

import autopilot
from calibration_fit import MagnetometerAutomaticCalibration
import vector
import quaternion
from signalk.server import SignalKServer
from signalk.pipeserver import SignalKPipeServer
from signalk.values import *

try:
  import RTIMU
except ImportError:
  print "RTIMU library not detected, please install it"

def imu_process(pipe, cal_queue, compass_cal, gyrobias):
    if os.system('sudo chrt -pf 2 %d 2>&1 > /dev/null' % os.getpid()):
      print 'warning, failed to make imu process realtime'

    #os.system("sudo renice -10 %d" % os.getpid())
    SETTINGS_FILE = "RTIMULib"
    s = RTIMU.Settings(SETTINGS_FILE)
    s.FusionType = 1
    s.CompassCalValid = False

    s.CompassCalEllipsoidOffset = tuple(compass_cal[:3])
    
    s.CompassCalEllipsoidValid = True
    s.MPU9255AccelFsr = 0 # +- 2g
    s.MPU9255GyroFsr = 0 # +- 250 deg/s
    # compass noise by rate 10=.043, 20=.033, 40=.024, 80=.017, 100=.015
    rate = 100
    s.MPU9255GyroAccelSampleRate = rate
    s.MPU9255CompassSampleRate = rate

    s.GyroBiasValid = True
    if gyrobias:
      s.GyroBias = tuple(map(math.radians, gyrobias))
    else:
      s.GyroBias = (0, 0, 0)

    s.KalmanRk, s.KalmanQ = .002, .001
#    s.KalmanRk, s.KalmanQ = .0005, .001

    rtimu = RTIMU.RTIMU(s)    

    print("Using settings file " + SETTINGS_FILE + ".ini")
    if not os.path.exists(SETTINGS_FILE + ".ini"):
        print("Settings file does not exist, will be created")

    print("IMU Name: " + rtimu.IMUName())

    while True:
      if not rtimu.IMUInit():
        print("ERROR: IMU Init Failed, no inertial data available")
        time.sleep(3)
        continue

      # this is a good time to set any fusion parameters
      rtimu.setSlerpPower(.01)
      rtimu.setGyroEnable(True)
      rtimu.setAccelEnable(True)
      rtimu.setCompassEnable(True)

      poll_interval = rtimu.IMUGetPollInterval()
      time.sleep(.1)

      c = 0
      while True:
        t0 = time.time()
        
        if rtimu.IMURead():
          data = rtimu.getIMUData()

          if cal_queue.qsize() > 0:
            new_cal = cal_queue.get()
            s.CompassCalEllipsoidValid = True
            s.CompassCalEllipsoidOffset = new_cal
            #rtimu.resetFusion()

          data['accelresiduals'] = list(rtimu.getAccelResiduals())
            
          data['gyrobias'] = s.GyroBias
          pipe.send(data)
        else:
          print 'failed to read IMU!!!!!!!!!!!!!!'
          break # reinitialize imu
        
        dt = time.time() - t0
        t = .1 - dt # 10hz

        if t > 0:
          time.sleep(t)

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


class AgeValue(StringValue):
    def __init__(self, name, **kwargs):
        super(AgeValue, self).__init__(name, '', **kwargs)
        self.time = os.times()[4]

        mods = {'s': 1, 'm': 60, 'h': 60, 'd': 24, 'y': 365.24}
        self.total = 0
        try:
          for arg in self.value.split(' '):
            if len(arg):
              total += mods[arg[-1:]] * float(arg[:-1])
        except:
          print 'invalid timespan', self.value
            
    def readable_timespan(self):
        time = self.total
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
      now = os.times()[4]
      dt = now - self.time
      if dt > 1:
        self.total += dt
        super(AgeValue, self).update(self.readable_timespan())
        self.time = now
        
    def reset(self):
        self.time = os.times()[4]
        self.total = 0

class QuaternionValue(ResettableValue):
    def __init__(self, name, initial, **kwargs):
      super(QuaternionValue, self).__init__(name, initial, **kwargs)


    def set(self, value):
      super(QuaternionValue, self).set(quaternion.normalize(value))

def nonblockingpipe():
  import _multiprocessing, socket
  s = socket.socketpair()
  map(lambda t : t.setblocking(False), s)
  p = map(lambda t : _multiprocessing.Connection(os.dup(t.fileno())), s)
  s[0].close(), s[1].close()
  return p

class BoatIMU(object):
  def __init__(self, server, *args, **keywords):
    self.server = server

    self.loopfreq = self.Register(LoopFreqValue, 'loopfreq', 0)
    self.alignmentQ = self.Register(QuaternionValue, 'alignmentQ', [1, 0, 0, 0], persistent=True)
    self.heading_off = self.Register(RangeProperty, 'heading_offset', 0, -180, 180)

    self.alignmentCounter = self.Register(Property, 'alignmentCounter', 0)
    self.last_alignmentCounter = False

    self.uptime = self.Register(AgeValue, 'uptime')
    self.compass_calibration_age = self.Register(AgeValue, 'compass_calibration_age')

    self.compass_calibration = self.Register(RoundedValue, 'compass_calibration', [[0, 0, 0, 30], 0, [0, 0, 0, 30, 1, 1], [0, 0, 0, 30, 0], [1, 0, 0, 0]], persistent=True)

    self.compass_calibration_sigmapoints = self.Register(RoundedValue, 'compass_calibration_sigmapoints', False)
    self.imu_pipe = nonblockingpipe()
    imu_cal_queue = multiprocessing.Queue()

    #if self.load_calibration():
    #  imu_cal_queue.put(tuple(self.compass_calibration.value[0][:3]))

    self.compass_auto_cal = MagnetometerAutomaticCalibration(imu_cal_queue, self.compass_calibration.value[0])

    self.lastqpose = False
    self.FirstTimeStamp = False

    self.headingrate = self.heel = 0
    self.heading_lowpass3 = self.heading_lowpass3a = self.heading_lowpass3b = False

    self.SensorValues = {}
    timestamp = server.TimeStamp('imu')
    for name in ['timestamp', 'fusionQPose', 'accel', 'gyro', 'compass', 'gyrobias', 'accelresiduals', 'heading_lowpass', 'pitch', 'roll', 'heading', 'pitchrate', 'rollrate', 'headingrate', 'headingraterate', 'heel']:
      self.SensorValues[name] = self.Register(SensorValue, name, timestamp)

    self.SensorValues['gyrobias'].make_persistent(120) # write gyrobias every 2 minutes

    self.imu_process = multiprocessing.Process(target=imu_process, args=(self.imu_pipe[1],imu_cal_queue, self.compass_calibration.value[0], self.SensorValues['gyrobias'].value))
    self.last_imuread = time.time()
    self.lasttimestamp = 0
    self.last_heading_off = 0

  def __del__(self):
    self.imu_process.terminate()
    self.compass_auto_cal.process.terminate()

  def Register(self, _type, name, *args, **kwargs):
    value = _type(*(['imu/' + name] + list(args)), **kwargs)
    return self.server.Register(value)
      
  def update_alignment(self, q):
    a2 = 2*math.atan2(q[3], q[0])
    heading_offset = a2*180/math.pi
    off = self.heading_off.value - heading_offset
    o = quaternion.angvec2quat(off*math.pi/180, [0, 0, 1])
    self.alignmentQ.update(quaternion.normalize(quaternion.multiply(q, o)))

  def IMURead(self):
    if not self.imu_process.is_alive():
      print 'launching imu process...'
      self.imu_process.start()
      return False
    
    # flush queue
    data = False

    while True: # read all the data from the non-blocking pipe
      try:
        data = self.imu_pipe[0].recv()
      except IOError:
        break

    if not data:
      if time.time() - self.last_imuread > 1 and self.loopfreq.value:
        print 'IMURead failed!'
        self.loopfreq.set(0)
        for name in self.SensorValues:
          self.SensorValues[name].set(False)
      return False
  
    self.last_imuread = time.time()
    
    self.loopfreq.strobe()

    if not self.FirstTimeStamp:
      self.FirstTimeStamp = data['timestamp']

    data['timestamp'] -= self.FirstTimeStamp
    data['timestamp'] /= 1e6

    #data['accel_comp'] = quaternion.rotvecquat(vector.sub(data['accel'], down), self.alignmentQ.value)

    # apply alignment calibration
    origfusionQPose = data['fusionQPose']
    data['fusionQPose'] = quaternion.multiply(data['fusionQPose'], self.alignmentQ.value)

    if vector.norm(data['accel']) == 0:
      print 'vector n', data['accel']
      return False

    data['roll'], data['pitch'], data['heading'] = map(math.degrees, quaternion.toeuler(data['fusionQPose']))

    if data['heading'] < 0:
      data['heading'] += 360

    gyro_q = quaternion.rotvecquat(data['gyro'], data['fusionQPose'])
    data['pitchrate'], data['rollrate'], data['headingrate'] = map(math.degrees, gyro_q)

    dt = data['timestamp'] - self.lasttimestamp
    self.lasttimestamp = data['timestamp']
    if dt > .02 and dt < .5:
      data['headingraterate'] = (data['headingrate'] - self.headingrate) / dt
    else:
      data['headingraterate'] = 0

    self.headingrate = data['headingrate']

    data['heel'] = self.heel = data['roll']*.05 + self.heel*.95

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
    llp = .2
#    self.heading_lowpass3a = heading_filter(llp, data['heading'], self.heading_lowpass3a)
#    self.heading_lowpass3b = heading_filter(llp, self.heading_lowpass3a, self.heading_lowpass3b)
#    self.heading_lowpass3 = heading_filter(llp, self.heading_lowpass3b, self.heading_lowpass3)

 #   data['heading_lowpass'] = self.heading_lowpass3
    data['heading_lowpass'] = data['heading']

    data['gyro'] = map(math.degrees, data['gyro'])
    data['gyrobias'] = map(math.degrees, data['gyrobias'])

    self.server.TimeStamp('imu', data['timestamp'])
    for name in self.SensorValues:
      self.SensorValues[name].set(data[name])

    # in main process:
    down = quaternion.rotvecquat([0, 0, 1], quaternion.conjugate(origfusionQPose))
    self.compass_auto_cal.AddPoint(list(data['compass']) + down)

    self.uptime.update()

    # count down to alignment
    if self.alignmentCounter.value != self.last_alignmentCounter:
      self.alignmentPose = [0, 0, 0, 0]

    if self.alignmentCounter.value > 0:
      self.alignmentPose = map(lambda x, y : x + y, self.alignmentPose, data['fusionQPose'])
      self.alignmentCounter.set(self.alignmentCounter.value-1)

      if self.alignmentCounter.value == 0:
        self.alignmentPose = quaternion.normalize(self.alignmentPose)
        adown = quaternion.rotvecquat([0, 0, 1], quaternion.conjugate(self.alignmentPose))
        alignment = []

        alignment = quaternion.vec2vec2quat([0, 0, 1], adown)
        alignment = quaternion.multiply(self.alignmentQ.value, alignment)
        
        if len(alignment):
          self.update_alignment(alignment)

      self.last_alignmentCounter = self.alignmentCounter.value
    if self.heading_off.value != self.last_heading_off:
      self.update_alignment(self.alignmentQ.value)
      self.last_heading_off = self.heading_off.value

    result = self.compass_auto_cal.UpdatedCalibration()
    if result:
      self.compass_calibration_sigmapoints.set(result[1])

      if result[0]:
        self.compass_calibration_age.reset()
        self.compass_calibration.set(result[0])

    self.compass_calibration_age.update()
    return data


if __name__ == "__main__":
#  server = SignalKServer()
  server = SignalKPipeServer()
  boatimu = BoatIMU(server)

  heading_lp = 0

  while True:
    t0 = time.time()
    data = boatimu.IMURead()
#    if data:
#      print 'pitch', data['pitch'], 'roll', data['roll'], 'heading', data['heading']

    dt = time.time() - t0
    t1=time.time()
#    server.HandleRequests(.1 - dt)
    server.HandleRequests(.1)

