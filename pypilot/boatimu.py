#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# Boat imu is built on top of RTIMU

# it is an enhanced imu with special knowledge of boat dynamics
# giving it the ability to auto-calibrate the inertial sensors

from __future__ import print_function
import os, sys
import time, math, multiprocessing, select
from signalk.pipeserver import NonBlockingPipe

import autopilot
from calibration_fit import IMUAutomaticCalibration
import vector
import quaternion
from signalk.server import SignalKServer
from signalk.pipeserver import SignalKPipeServer
from signalk.values import *

try:
  import RTIMU
except ImportError:
  RTIMU = False
  print('RTIMU library not detected, please install it')

def imu_process(pipe, cal_pipe, accel_cal, compass_cal, gyrobias, period):
    if not RTIMU:
      while True:
        time.sleep(10)
  
    #print 'imu on', os.getpid()
    if os.system('sudo chrt -pf 2 %d 2>&1 > /dev/null' % os.getpid()):
      print('warning, failed to make imu process realtime')

    #os.system("sudo renice -10 %d" % os.getpid())
    SETTINGS_FILE = "RTIMULib"
    s = RTIMU.Settings(SETTINGS_FILE)
    s.FusionType = 1
    s.CompassCalValid = False

    s.CompassCalEllipsoidOffset = tuple(compass_cal[:3])  
    s.CompassCalEllipsoidValid = True
    s.MPU925xAccelFsr = 0 # +- 2g
    s.MPU925xGyroFsr = 0 # +- 250 deg/s
    # compass noise by rate 10=.043, 20=.033, 40=.024, 80=.017, 100=.015
    rate = 100
    s.MPU925xGyroAccelSampleRate = rate
    s.MPU925xCompassSampleRate = rate

    s.AccelCalValid = True
    if accel_cal:
      s.AccelCalMin = tuple(map(lambda x : x - accel_cal[3], accel_cal[:3]))
      s.AccelCalMax = tuple(map(lambda x : x + accel_cal[3], accel_cal[:3]))
    else:
      s.AccelCalMin = (-1, -1, -1)
      s.AccelCalMax = (1, 1, 1)

    s.GyroBiasValid = True
    if gyrobias:
      s.GyroBias = tuple(map(math.radians, gyrobias))
    else:
      s.GyroBias = (0, 0, 0)

    s.KalmanRk, s.KalmanQ = .002, .001
#    s.KalmanRk, s.KalmanQ = .0005, .001

    while True:
      print("Using settings file " + SETTINGS_FILE + ".ini")
      s.IMUType = 0 # always autodetect imu
      rtimu = RTIMU.RTIMU(s)
      if rtimu.IMUName() == 'Null IMU':
        print('no IMU detected... try again')
        time.sleep(1)
        continue
      
      print("IMU Name: " + rtimu.IMUName())

      if not rtimu.IMUInit():
        print("ERROR: IMU Init Failed, no inertial data available")
        time.sleep(1)
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
          data['accel.residuals'] = list(rtimu.getAccelResiduals())
          data['gyrobias'] = s.GyroBias
          data['timestamp'] = t0 # imu timestamp is perfectly accurate
          pipe.send(data, False)
        else:
          print('failed to read IMU!!!!!!!!!!!!!!')
          break # reinitialize imu

        if cal_poller.poll(0):
          r = cal_pipe.recv()
          
          #print('[imu process] new cal', new_cal)
          if r[0] == 'accel':
            s.AccelCalValid = True
            b, t = r[1][0][:3], r[1][0][3]
            s.AccelCalMin = b[0] - t, b[1] - t, b[2] - t
            s.AccelCalMax = b[0] + t, b[1] + t, b[2] + t
          elif r[0] == 'compass':
            s.CompassCalEllipsoidValid = True
            s.CompassCalEllipsoidOffset = tuple(r[1][0][:3])
          #rtimu.resetFusion()
        
        dt = time.time() - t0
        t = period - dt # 10hz

        if t > 0 and t < period:
          time.sleep(t)
        else:
          print('imu process failed to keep time', t)

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

def readable_timespan(total):
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

class TimeValue(StringValue):
    def __init__(self, name, **kwargs):
        super(TimeValue, self).__init__(name, 0, **kwargs)
        self.lastupdate_value = 0
        self.lastage_value = -100
        self.stopped = True
        self.total = self.value
        
    def reset(self):
        self.lastupdate_value = 0
        self.total = 0
        self.start = time.time()
        self.set(0)

    def update(self):
        t = time.time()
        if self.stopped:
            self.stopped = False
            self.start = t

        self.value = self.total + t - self.start
        if abs(self.value - self.lastupdate_value) > 1:
          self.lastupdate_value = self.value
          self.send()

    def stop(self):
      if self.stopped:
        return
      self.total += time.time() - self.start
      self.stopped = True;

    def get_signalk(self):
        if abs(self.value - self.lastage_value) > 1: # to reduce cpu, if the time didn't change by a second
            self.lastage_value = self.value
            self.lastage = readable_timespan(self.value)
        return '{"' + self.name + '": {"value": "' + self.lastage + '"}}'
      
class AgeValue(StringValue):
    def __init__(self, name, **kwargs):
        super(AgeValue, self).__init__(name, time.time(), **kwargs)
        self.dt = max(0, time.time() - self.value)
        self.lastupdate_value = -1
        self.lastage = ''

    def reset(self):
        self.set(time.time())

    def update(self):
        t = time.time()
        if abs(t - self.lastupdate_value) > 1:
          self.lastupdate_value = t
          self.send()

    def get_signalk(self):
        dt = max(0, time.time() - self.value)
        if abs(dt - self.dt) > 1:
            self.dt = dt
            self.lastage = readable_timespan(dt)
        return '{"' + self.name + '": {"value": "' + self.lastage + '"}}'

class QuaternionValue(ResettableValue):
    def __init__(self, name, initial, **kwargs):
      super(QuaternionValue, self).__init__(name, initial, **kwargs)


    def set(self, value):
      if value:
        value = quaternion.normalize(value)
      super(QuaternionValue, self).set(value)

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

    self.rate = self.Register(EnumProperty, 'rate', 10, [10, 25], persistent=True)
    self.period = 1.0/self.rate.value

    self.loopfreq = self.Register(LoopFreqValue, 'loopfreq', 0)
    self.alignmentQ = self.Register(QuaternionValue, 'alignmentQ', [1, 0, 0, 0], persistent=True)
    self.heading_off = self.Register(RangeProperty, 'heading_offset', 0, -180, 180, persistent=True)

    self.alignmentCounter = self.Register(Property, 'alignmentCounter', 0)
    self.last_alignmentCounter = False

    self.uptime = self.Register(TimeValue, 'uptime')

    def calibration(name, default):
        calibration = self.Register(RoundedValue, name+'.calibration', default, persistent=True)
        calibration.age = self.Register(AgeValue, name+'.calibration.age', persistent=True)
        calibration.locked = self.Register(BooleanProperty, name+'.calibration.locked', False, persistent=True)
        calibration.sigmapoints = self.Register(RoundedValue, name+'.calibration.sigmapoints', False)
        return calibration

    self.accel_calibration = calibration('accel', [[0, 0, 0, 1], 1])
    self.compass_calibration = calibration('compass', [[0, 0, 0, 30, 0], [1, 1], 0])
    
    self.imu_pipe, imu_pipe = NonBlockingPipe('imu_pipe')
    imu_cal_pipe = NonBlockingPipe('imu_cal_pipe')

    self.poller = select.poll()
    self.poller.register(self.imu_pipe, select.POLLIN)

    self.auto_cal = IMUAutomaticCalibration(imu_cal_pipe[1], self.accel_calibration.value[0], self.compass_calibration.value[0])

    self.lastqpose = False
    self.FirstTimeStamp = False

    self.headingrate = self.heel = 0
    self.heading_lowpass_constant = self.Register(RangeProperty, 'heading_lowpass_constant', .1, .01, 1)
    self.headingrate_lowpass_constant = self.Register(RangeProperty, 'headingrate_lowpass_constant', .1, .01, 1)
    self.headingraterate_lowpass_constant = self.Register(RangeProperty, 'headingraterate_lowpass_constant', .1, .01, 1)
          
    sensornames = ['accel', 'gyro', 'compass', 'accel.residuals', 'pitch', 'roll']

    sensornames += ['pitchrate', 'rollrate', 'headingrate', 'headingraterate', 'heel']
    sensornames += ['headingrate_lowpass', 'headingraterate_lowpass']
    directional_sensornames = ['heading', 'heading_lowpass']
    sensornames += directional_sensornames
    
    self.SensorValues = {}
    timestamp = server.TimeStamp('imu')
    for name in sensornames:
      self.SensorValues[name] = self.Register(SensorValue, name, timestamp, directional = name in directional_sensornames)

    # quaternion needs to report many more decimal places than other sensors
    sensornames += ['fusionQPose']
    self.SensorValues['fusionQPose'] = self.Register(SensorValue, 'fusionQPose', timestamp, fmt='%.7f')
    
    sensornames += ['gyrobias']
    self.SensorValues['gyrobias'] = self.Register(SensorValue, 'gyrobias', timestamp, persistent=True)

    self.imu_process = multiprocessing.Process(target=imu_process, args=(imu_pipe,imu_cal_pipe[0], self.accel_calibration.value[0], self.compass_calibration.value[0], self.SensorValues['gyrobias'].value, self.period))
    self.imu_process.start()

    self.last_imuread = time.time()
    self.lasttimestamp = 0
    self.last_heading_off = 3000 # invalid

  def __del__(self):
    print('terminate imu process')
    self.imu_process.terminate()

  def Register(self, _type, name, *args, **kwargs):
    value = _type(*(['imu.' + name] + list(args)), **kwargs)
    return self.server.Register(value)
      
  def update_alignment(self, q):
    a2 = 2*math.atan2(q[3], q[0])
    heading_offset = a2*180/math.pi
    off = self.heading_off.value - heading_offset
    o = quaternion.angvec2quat(off*math.pi/180, [0, 0, 1])
    self.alignmentQ.update(quaternion.normalize(quaternion.multiply(q, o)))
    self.auto_cal.SetNorm(quaternion.rotvecquat([0, 0, 1], self.alignmentQ.value))

  def IMURead(self):    
    data = False

    while self.poller.poll(0): # read all the data from the pipe
      data = self.imu_pipe.recv()

    if not data:
      if time.time() - self.last_imuread > 1 and self.loopfreq.value:
        print('IMURead failed!')
        self.loopfreq.set(0)
        for name in self.SensorValues:
          self.SensorValues[name].set(False)
        self.uptime.reset();
      return False
  
    if vector.norm(data['accel']) == 0:
      print('vector n', data['accel'])
      return False

    self.last_imuread = time.time()
    self.loopfreq.strobe()

    if not self.FirstTimeStamp:
      self.FirstTimeStamp = data['timestamp']

    data['timestamp'] -= self.FirstTimeStamp

    #data['accel_comp'] = quaternion.rotvecquat(vector.sub(data['accel'], down), self.alignmentQ.value)

    # apply alignment calibration
    gyro_q = quaternion.rotvecquat(data['gyro'], data['fusionQPose'])

    data['pitchrate'], data['rollrate'], data['headingrate'] = map(math.degrees, gyro_q)

    origfusionQPose = data['fusionQPose']
    aligned = quaternion.multiply(data['fusionQPose'], self.alignmentQ.value)
    data['fusionQPose'] = quaternion.normalize(aligned) # floating point precision errors

    data['roll'], data['pitch'], data['heading'] = map(math.degrees, quaternion.toeuler(data['fusionQPose']))

    if data['heading'] < 0:
      data['heading'] += 360

    dt = data['timestamp'] - self.lasttimestamp
    self.lasttimestamp = data['timestamp']
    if dt > .02 and dt < .5:
      data['headingraterate'] = (data['headingrate'] - self.headingrate) / dt
    else:
      data['headingraterate'] = 0

    self.headingrate = data['headingrate']

    data['heel'] = self.heel = data['roll']*.03 + self.heel*.97
    #data['roll'] -= data['heel']

    data['gyro'] = list(map(math.degrees, data['gyro']))
    data['gyrobias'] = list(map(math.degrees, data['gyrobias']))

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

    compass, accel, down = False, False, False
    if not self.accel_calibration.locked.value:
      accel = list(data['accel'])
    if not self.compass_calibration.locked.value:
      down = quaternion.rotvecquat([0, 0, 1], quaternion.conjugate(origfusionQPose))
      compass = list(data['compass']) + down
    if accel or compass:
      self.auto_cal.AddPoint((accel, compass, down))

    self.uptime.update()

    # count down to alignment
    if self.alignmentCounter.value != self.last_alignmentCounter:
      self.alignmentPose = [0, 0, 0, 0]

    if self.alignmentCounter.value > 0:
      self.alignmentPose = list(map(lambda x, y : x + y, self.alignmentPose, data['fusionQPose']))
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

    result = self.auto_cal.UpdatedCalibration()
    
    if result:
      if result[0] == 'accel' and not self.accel_calibration.locked.value:
        #print('[boatimu] cal result', result[0])
        self.accel_calibration.sigmapoints.set(result[2])
        if result[1]:
          self.accel_calibration.age.reset()
          self.accel_calibration.set(result[1])
      elif result[0] == 'compass' and not self.compass_calibration.locked.value:
        #print('[boatimu] cal result', result[0])
        self.compass_calibration.sigmapoints.set(result[2])
        if result[1]:
          self.compass_calibration.age.reset()
          self.compass_calibration.set(result[1])

    self.accel_calibration.age.update()
    self.compass_calibration.age.update()
    return data

class BoatIMUServer():
  def __init__(self):
    # setup all processes to exit on any signal
    self.childpids = []
    def cleanup(signal_number, frame=None):
        print('got signal', signal_number, 'cleaning up')
        while self.childpids:
            pid = self.childpids.pop()
            os.kill(pid, signal.SIGTERM) # get backtrace
        sys.stdout.flush()
        if signal_number != 'atexit':
          raise KeyboardInterrupt # to get backtrace on all processes

    # unfortunately we occasionally get this signal,
    # some sort of timing issue where python doesn't realize the pipe
    # is broken yet, so doesn't raise an exception
    def printpipewarning(signal_number, frame):
        print('got SIGPIPE, ignoring')

    import signal
    for s in range(1, 16):
        if s == 13:
            signal.signal(s, printpipewarning)
        elif s != 9:
            signal.signal(s, cleanup)

    #  server = SignalKServer()
    self.server = SignalKPipeServer()
    self.boatimu = BoatIMU(self.server)

    self.childpids = [self.boatimu.imu_process.pid, self.boatimu.auto_cal.process.pid,
                      self.server.process.pid]
    signal.signal(signal.SIGCHLD, cleanup)
    import atexit
    atexit.register(lambda : cleanup('atexit'))
    
    self.t00 = time.time()

  def iteration(self):
    self.server.HandleRequests()
    self.data = self.boatimu.IMURead()

    while True:
      dt = self.boatimu.period - (time.time() - self.t00)
      if dt <= 0 or dt >= self.boatimu.period:
        break
      time.sleep(dt)
    self.t00 = time.time()

def main():
  boatimu = BoatIMUServer()
  quiet = '-q' in sys.argv

  while True:
    boatimu.iteration()
    data = boatimu.data
    if data and not quiet:
      def line(*args):
        for a in args:
          sys.stdout.write(str(a))
          sys.stdout.write(' ')
        sys.stdout.write('\r')
        sys.stdout.flush()
      line('pitch', data['pitch'], 'roll', data['roll'], 'heading', data['heading'])

if __name__ == '__main__':
    main()
