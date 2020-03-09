#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
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

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import calibration_fit, vector, quaternion
from pypilot.server import pypilotServer
from pypilot.pipeserver import pypilotPipeServer, NonBlockingPipe
from pypilot.values import *

try:
    import RTIMU
except ImportError:
    RTIMU = False
    print('RTIMU library not detected, please install it')

class IMU(object):
    def __init__(self, mp):
        if mp:
            self.pipe, pipe = NonBlockingPipe('imu_pipe')
            process = multiprocessing.Process(target=self.process, args=(pipe))
            self.process.start()
        else:
            self.setup()
          
    def process(self, pipe):
        if not RTIMU:
            while True:
                time.sleep(10)

        if os.system('sudo chrt -pf 99 %d 2>&1 > /dev/null' % os.getpid()):
            print('warning, failed to make imu process realtime')

        self.setup()
        while True:
            self.init()
            data = self.read()
            pipe.send(data, not data)
            self.poll()
            dt = time.monotonic() - t0
            t = period.value - dt
            if t > 0 and t < period:
                time.sleep(t)
            else:
                print('imu process failed to keep time', t)

    def setup(self):
        self.client = pypilotClient('localhost')
        self.client.watch('imu.accel.calibration')
        self.client.watch('imu.compass.calibration')

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

        s.AccelCalValid = True # will be updated later
        s.AccelCalMin = (-1, -1, -1)
        s.AccelCalMax = (1, 1, 1)

        s.GyroBiasValid = True
        s.GyroBias = (0, 0, 0)

        s.KalmanRk, s.KalmanQ = .002, .001
        self.s = s

    def init(self):
        print("Using settings file " + SETTINGS_FILE + ".ini")
        self.s.IMUType = 0 # always autodetect imu
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

        self.avggyro = [0, 0, 0]
        self.compass_calibration_updated = False

    def read(self):
        t0 = time.monotonic()
        if not rtimu.IMURead():
            print('failed to read IMU!')
            self.init() # reinitialize imu
            return False 
         
        data = rtimu.getIMUData()
        data['accel.residuals'] = list(rtimu.getAccelResiduals())
        data['gyrobias'] = s.GyroBias
        #data['timestamp'] = t0 # imu timestamp is perfectly accurate
        
        if self.compass_calibration_updated:
            data['compass_calibration_updated'] = True
            self.compass_calibration_updated = False

        return data

    def poll(self):
        # see if gyro is out of range, sometimes the sensors read
        # very high gyro readings and the sensors need to be reset by software
        # this is probably a bug in the underlying driver with fifo misalignment
        d = .05*period.value # filter constant
        for i in range(3): # filter gyro vector
            avggyro[i] = (1-d)*avggyro[i] + d*data['gyro'][i]
        if vector.norm(avggyro) > .8: # 55 degrees/s
            print('too high standing gyro bias, resetting sensors', data['gyro'], avggyro)
            break
        # detects the problem even faster:
        if any(map(lambda x : abs(x) > 1000, data['compass'])):
            print('compass out of range, resetting', data['compass'])
            break

        msgs = client.receive()
        for name in msgs:
            value = msgs[name]
            if name == 'imu.accel.calibration':
                self.s.AccelCalValid = True
                b, t = value[0][:3], value[0][3]
                self.s.AccelCalMin = b[0] - t, b[1] - t, b[2] - t
                self.s.AccelCalMax = b[0] + t, b[1] + t, b[2] + t
            elif name == 'imu.compass.calibration':
                self.compass_calibration_updated = True
                self.s.CompassCalEllipsoidValid = True
                self.s.CompassCalEllipsoidOffset = tuple(value[0][:3])
                #rtimu.resetFusion()

class LoopFreqValue(Value):
    def __init__(self, name, initial):
        super(LoopFreqValue, self).__init__(name, initial)
        self.loopc = 0
        self.t0 = time.monotonic()

    def strobe(self):
        self.loopc += 1
        if self.loopc == 10:
            t1 = time.monotonic()
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
        self.start = time.monotonic()
        self.set(0)

    def update(self):
        t = time.monotonic()
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
      self.total += time.monotonic() - self.start
      self.stopped = True

    def get_pypilot(self):
        if abs(self.value - self.lastage_value) > 1: # to reduce cpu, if the time didn't change by a second
            self.lastage_value = self.value
            self.lastage = readable_timespan(self.value)
        return self.lastage

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
    def __init__(self, client, *args, **keywords):
        self.starttime = time.monotonic()
        self.client = client

        self.timestamp = client.register(SensorValue('timestamp', 0))
        self.rate = self.register(EnumProperty, 'rate', 10, [10, 25], persistent=True)
        self.period = 1.0/self.rate.value

        self.loopfreq = self.register(LoopFreqValue, 'loopfreq', 0)
        self.alignmentQ = self.register(QuaternionValue, 'alignmentQ', [2**.5/2, -2**.5/2, 0, 0], persistent=True)
        self.alignmentQ.last = False
        self.heading_off = self.register(RangeProperty, 'heading_offset', 0, -180, 180, persistent=True)
        self.heading_off.last = 3000 # invalid

        self.alignmentCounter = self.register(Property, 'alignmentCounter', 0)
        self.last_alignmentCounter = False

        self.uptime = self.register(TimeValue, 'uptime')
    
        self.poller = select.poll()
        self.poller.register(self.imu_pipe, select.POLLIN)

        self.lasttimestamp = 0

        self.headingrate = self.heel = 0
        self.heading_lowpass_constant = self.register(RangeProperty, 'heading_lowpass_constant', .1, .01, 1)
        self.headingrate_lowpass_constant = self.register(RangeProperty, 'headingrate_lowpass_constant', .1, .01, 1)
        self.headingraterate_lowpass_constant = self.register(RangeProperty, 'headingraterate_lowpass_constant', .1, .01, 1)

        sensornames = ['accel', 'gyro', 'compass', 'accel.residuals', 'pitch', 'roll']
        sensornames += ['pitchrate', 'rollrate', 'headingrate', 'headingraterate', 'heel']
        sensornames += ['headingrate_lowpass', 'headingraterate_lowpass']
        directional_sensornames = ['heading', 'heading_lowpass']
        sensornames += directional_sensornames
    
        self.SensorValues = {}
        for name in sensornames:
            self.SensorValues[name] = self.register(SensorValue, name, directional = name in directional_sensornames)

        # quaternion needs to report many more decimal places than other sensors
        sensornames += ['fusionQPose']
        self.SensorValues['fusionQPose'] = self.register(SensorValue, 'fusionQPose', fmt='%.7f')
    
        sensornames += ['gyrobias']
        self.SensorValues['gyrobias'] = self.register(SensorValue, 'gyrobias', persistent=True)

        self.imu = IMU(False)

        self.last_imuread = time.monotonic()

    def __del__(self):
        print('terminate imu process')
        self.imu_process.terminate()

    def register(self, _type, name, *args, **kwargs):
        value = _type(*(['imu.' + name] + list(args)), **kwargs)
        return self.server.register(value)
      
    def update_alignment(self, q):
        a2 = 2*math.atan2(q[3], q[0])
        heading_offset = a2*180/math.pi
        off = self.heading_off.value - heading_offset
        o = quaternion.angvec2quat(off*math.pi/180, [0, 0, 1])
        self.alignmentQ.update(quaternion.normalize(quaternion.multiply(q, o)))

    def IMUread(self):
        if self.imu.mp:
            data = False
            while self.poller.poll(0): # read all the data from the pipe
                data = self.imu_pipe.recv()
            return data
        return self.imu.read()

    def poll(self):
        if not self.imu.mp:
            self.imu.poll()

    def read(self):
        data = self.IMUread()
        if not data:
            if time.monotonic() - self.last_imuread > 1 and self.loopfreq.value:
                print('IMURead failed!')
                self.loopfreq.set(0)
                for name in self.SensorValues:
                    self.SensorValues[name].set(False)
                self.uptime.reset()
            return False
  
        if vector.norm(data['accel']) == 0:
            print('accel values invalid', data['accel'])
            return False
  
        t = time.monotonic()
        self.timestamp.set(t-self.starttime)
  
        self.last_imuread = t
        self.loopfreq.strobe()
  
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
        for name in self.SensorValues:
            self.SensorValues[name].set(data[name])

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

        # if alignment or heading offset changed:
        if self.heading_off.value != self.heading_off.last or \
           self.alignmentQ.value != self.alignmentQ.last:
            self.update_alignment(self.alignmentQ.value)
            self.heading_off.last = self.heading_off.value
            self.alignmentQ.last = self.alignmentQ.value

        return data

      '''
    cal_data = {}
    if not self.accel_calibration.locked.value:
        cal_data['accel'] = list(data['accel'])
    if not self.compass_calibration.locked.value:
        cal_data['compass'] = list(data['compass'])
        cal_data['down'] = quaternion.rotvecquat([0, 0, 1], quaternion.conjugate(origfusionQPose))

    if cal_data:
        self.auto_cal.cal_pipe.send(cal_data)
      '''

def main():
    server = pypilotServer()
    client = pypilotClient(server.pipe())
    boatimu = BoatIMU(client)

    t00 = time.monotonic()
    quiet = '-q' in sys.argv

    while True:
        self.server.poll()
        self.client.poll()
        data = boatimu.read()
        if data and not quiet:
            def line(*args):
                for a in args:
                    sys.stdout.write(str(a))
                    sys.stdout.write(' ')
                sys.stdout.write('\r')
                sys.stdout.flush()
            line('pitch', data['pitch'], 'roll', data['roll'], 'heading', data['heading'])
        data = boatimu.poll()

        while True:
            dt = boatimu.period - (time.monotonic() - self.t00)
            if dt >= boatimu.period:
                break
            time.sleep(dt)
        self.t00 = time.monotonic()
            
if __name__ == '__main__':
    main()
