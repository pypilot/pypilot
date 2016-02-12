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
import time, math, multiprocessing, select
from signalk.pipeserver import NonBlockingPipe

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

def imu_process(pipe, cal_pipe, compass_cal, gyrobias):
    #print 'imu on', os.getpid()
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
        exit(1) # if we run without gyros, how to inform user???
        continue

      # this is a good time to set any fusion parameters
      rtimu.setSlerpPower(.01)
      rtimu.setGyroEnable(True)
      rtimu.setAccelEnable(True)
      rtimu.setCompassEnable(True)

      poll_interval = rtimu.IMUGetPollInterval()
      time.sleep(.1)

      c = 0
      cal_poller = select.poll()
      cal_poller.register(cal_pipe, select.POLLIN)

      while True:
        t0 = time.time()
        
        if rtimu.IMURead():
          data = rtimu.getIMUData()
          data['accelresiduals'] = list(rtimu.getAccelResiduals())
          data['gyrobias'] = s.GyroBias
          data['timestamp'] = t0 # imu timestamp is perfectly accurate
          pipe.send(data, False)
        else:
          print 'failed to read IMU!!!!!!!!!!!!!!'
          break # reinitialize imu

        if cal_poller.poll(0):
          new_cal = cal_pipe.recv()
          #print '[imu process] new cal', new_cal
          s.CompassCalEllipsoidValid = True
          s.CompassCalEllipsoidOffset = new_cal
          #rtimu.resetFusion()
        
        dt = time.time() - t0
        t = .1 - dt # 10hz

        if t > 0:
          time.sleep(t)
        else:
          print 'imu process failed to keep time'

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
        self.value = time.time()
        super(AgeValue, self).__init__(name, '', **kwargs)
        self.dt = 0

    def readable_timespan(self, total):
        mods = [('s', 1), ('m', 60), ('h', 60), ('d', 24), ('y', 365.24)]          
        def loop(i, mod):
            if i == len(mods) or (int(total / (mods[i][1]*mod)) == 0 and i > 0):
                return ''
            if i < len(mods) - 1:
                div = mods[i][1]*mods[i+1][1]*mod
                t = int(total%int(div))
            else:
                t = total
            return loop(i+1, mods[i][1]*mod) + (('%d' + mods[i][0] + ' ') % (t/(mods[i][1]*mod)))
        return loop(0, 1)
        
    def reset(self):
        self.set(time.time())

    def update(self):
        self.send()

    def get_signalk(self):
        dt = time.time() - self.value
        if abs(dt - self.dt) > 1:
            self.dt = dt
            self.lastage = self.readable_timespan(dt))
        return '{"' + self.name + '": {"value": "' + self.lastage + '"}}'


class QuaternionValue(ResettableValue):
    def __init__(self, name, initial, **kwargs):
      super(QuaternionValue, self).__init__(name, initial, **kwargs)


    def set(self, value):
      super(QuaternionValue, self).set(quaternion.normalize(value))

def heading_filter(lp, a, b):
    if not a:
        return b
    if not b:
        return a
    if a - b > 180:
        a -= 360
    elif b - a > 180:
        b -= 360
    result = lp*a + (1-lp)*b
    if result < 0:
        result += 360
    return result

class BoatIMU(object):
  def __init__(self, server, *args, **keywords):
    self.server = server

    self.loopfreq = self.Register(LoopFreqValue, 'loopfreq', 0)
    self.alignmentQ = self.Register(QuaternionValue, 'alignmentQ', [1, 0, 0, 0], persistent=True)
    self.heading_off = self.Register(RangeProperty, 'heading_offset', 0, -180, 180)

    self.alignmentCounter = self.Register(Property, 'alignmentCounter', 0)
    self.last_alignmentCounter = False

    self.uptime = self.Register(AgeValue, 'uptime')
    self.compass_calibration_age = self.Register(AgeValue, 'compass_calibration_age', persistent=True)

    self.compass_calibration = self.Register(RoundedValue, 'compass_calibration', [[0, 0, 0, 30, 0], [0, 0, 0, 30], [0, 0, 0, 30, 1, 1]], persistent=True)

    self.compass_calibration_sigmapoints = self.Register(RoundedValue, 'compass_calibration_sigmapoints', False)
    self.compass_calibration_locked = self.Register(BooleanProperty, 'compass_calibration_locked', False, persistent=True)
    
    self.imu_pipe, imu_pipe = NonBlockingPipe('imu_pipe')
    imu_cal_pipe = NonBlockingPipe('imu_cal_pipe')

    self.poller = select.poll()
    self.poller.register(self.imu_pipe, select.POLLIN)

    self.compass_auto_cal = MagnetometerAutomaticCalibration(imu_cal_pipe[1], self.compass_calibration.value[0])

    self.lastqpose = False
    self.FirstTimeStamp = False

    self.headingrate = self.heel = 0
    self.heading_lowpass_constant = self.Register(RangeProperty, 'heading_lowpass_constant', .1, .01, 1)
    self.headingrate_lowpass_constant = self.Register(RangeProperty, 'headingrate_lowpass_constant', .1, .01, 1)
    self.headingraterate_lowpass_constant = self.Register(RangeProperty, 'headingraterate_lowpass_constant', .1, .01, 1)
          
    sensornames = ['fusionQPose', 'accel', 'gyro', 'compass', 'accelresiduals', 'pitch', 'roll']

    sensornames += ['pitchrate', 'rollrate', 'headingrate', 'headingraterate', 'heel']
    sensornames += ['headingrate_lowpass', 'headingraterate_lowpass']
    directional_sensornames = ['heading', 'heading_lowpass']
    sensornames += directional_sensornames
    
    self.SensorValues = {}
    timestamp = server.TimeStamp('imu')
    for name in sensornames:
      self.SensorValues[name] = self.Register(SensorValue, name, timestamp, directional = name in directional_sensornames)

    sensornames += ['gyrobias']
    self.SensorValues['gyrobias'] = self.Register(SensorValue, 'gyrobias', timestamp, persistent=True)

    self.imu_process = multiprocessing.Process(target=imu_process, args=(imu_pipe,imu_cal_pipe[0], self.compass_calibration.value[0], self.SensorValues['gyrobias'].value))
    self.imu_process.start()

    self.last_imuread = time.time()
    self.lasttimestamp = 0
    self.last_heading_off = 0

  def __del__(self):
    print 'terminate imu process'
    self.imu_process.terminate()

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
    data = False

    while self.poller.poll(0): # read all the data from the pipe
      data = self.imu_pipe.recv()

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

    data['heel'] = self.heel = data['roll']*.03 + self.heel*.97
    data['roll'] -= data['heel']

    data['gyro'] = map(math.degrees, data['gyro'])
    data['gyrobias'] = map(math.degrees, data['gyrobias'])

    # lowpass heading and rate
    llp = self.heading_lowpass_constant.value
    data['heading_lowpass'] = heading_filter(llp, data['heading'], self.SensorValues['heading_lowpass'].value)

    llp = self.headingrate_lowpass_constant.value
    data['headingrate_lowpass'] = llp*data['headingrate'] + (1-llp)*self.SensorValues['headingrate_lowpass'].value

    llp = self.headingraterate_lowpass_constant.value
    data['headingraterate_lowpass'] = llp*data['headingraterate'] + (1-llp)*self.SensorValues['headingraterate_lowpass'].value

    # set sensors
    self.server.TimeStamp('imu', data['timestamp'])
    for name in self.SensorValues:
      self.SensorValues[name].set(data[name])

    if not self.compass_calibration_locked.value:
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
    
    if result and not self.compass_calibration_locked.value:
      #print '[boatimu] cal result', result[0]
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
    if data:
      print 'pitch', data['pitch'], 'roll', data['roll'], 'heading', data['heading']
    server.HandleRequests()
    dt = time.time() - t0
    if dt > .1:
      time.sleep(dt);

