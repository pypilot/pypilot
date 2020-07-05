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

import os, sys
import time, math, multiprocessing, select

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
t0=time.monotonic()
import vector, quaternion
from client import pypilotClient
from values import *

from nonblockingpipe import NonBlockingPipe

try:
    import RTIMU
except ImportError:
    RTIMU = False
    print('RTIMU library not detected, please install it')

class IMU(object):
    def __init__(self, server):
        self.client = pypilotClient(server)
        self.multiprocessing = server.multiprocessing
        if self.multiprocessing:
            self.pipe, pipe = NonBlockingPipe('imu_pipe', self.multiprocessing)
            self.process = multiprocessing.Process(target=self.process, args=(pipe,), daemon=True)
            self.process.start()
            return
        self.setup()
          
    def setup(self):
        self.client.watch('imu.accel.calibration')
        self.client.watch('imu.compass.calibration')
        self.client.watch('imu.rate')

        SETTINGS_FILE = "RTIMULib"
        print("Using settings file " + SETTINGS_FILE + ".ini")
        s = RTIMU.Settings(SETTINGS_FILE)
        s.FusionType = 1
        s.CompassCalValid = False

        s.CompassCalEllipsoidOffset = (0, 0, 0)
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
        while not self.init():
            time.sleep(1)
        self.lastdata = False
        self.rate = 10

    def init(self):
        self.s.IMUType = 0 # always autodetect imu
        rtimu = RTIMU.RTIMU(self.s)
        if rtimu.IMUName() == 'Null IMU':
            print('no IMU detected... try again')
            return False
      
        print("IMU Name: " + rtimu.IMUName())

        if not rtimu.IMUInit():
            print("ERROR: IMU Init Failed, no inertial data available")
            return False

        # this is a good time to set any fusion parameters
        rtimu.setSlerpPower(.01)
        rtimu.setGyroEnable(True)
        rtimu.setAccelEnable(True)
        rtimu.setCompassEnable(True)
        time.sleep(.1)
        self.rtimu = rtimu

        self.avggyro = [0, 0, 0]
        self.compass_calibration_updated = False
        return True

    def process(self, pipe):
        if not RTIMU:
            while True:
                time.sleep(10) # do nothing

        if os.system('sudo chrt -pf 99 %d 2>&1 > /dev/null' % os.getpid()):
            print('warning, failed to make imu process realtime')
        else:
            print('made imu process realtime')

        self.setup()
        while True:
            t0 = time.monotonic()
            data = self.read()
            pipe.send(data, not data)
            self.poll()
            dt = time.monotonic() - t0
            period = 1/self.rate
            t = period - dt
            if t > 0 and t < period:
                time.sleep(t)
            else:
                print('imu process failed to keep time', t)

    def read(self):
        t0 = time.monotonic()
        if not self.rtimu.IMURead():
            print('failed to read IMU!')
            self.init() # reinitialize imu
            return False 
         
        data = self.rtimu.getIMUData()
        data['accel.residuals'] = list(self.rtimu.getAccelResiduals())
        data['gyrobias'] = self.s.GyroBias
        #data['timestamp'] = t0 # imu timestamp is perfectly accurate
        
        if self.compass_calibration_updated:
            data['compass_calibration_updated'] = True
            self.compass_calibration_updated = False

        self.lastdata = list(data['gyro']), list(data['compass'])
        return data

    def poll(self):
        msgs = self.client.receive()
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
            elif name == 'imu.rate':
                self.rate = value

        if not self.lastdata:
            return
        gyro, compass = self.lastdata

        # see if gyro is out of range, sometimes the sensors read
        # very high gyro readings and the sensors need to be reset by software
        # this is probably a bug in the underlying driver with fifo misalignment
        d = .05/self.rate # filter constant
        for i in range(3): # filter gyro vector
            self.avggyro[i] = (1-d)*self.avggyro[i] + d*gyro[i]
        if vector.norm(self.avggyro) > .8: # 55 degrees/s
            print('too high standing gyro bias, resetting sensors', gyro, self.avggyro)
            self.init()

        # detects the problem even faster:
        if any(map(lambda x : abs(x) > 1000, compass)):
            print('compass out of range, resetting', compass)
            self.init()

class FrequencyValue(SensorValue):
    def __init__(self, name):
        super(FrequencyValue, self).__init__(name)
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
          self.set(self.value)

    def stop(self):
      if self.stopped:
        return
      self.total += time.monotonic() - self.start
      self.stopped = True

    def get_msg(self):
        if abs(self.value - self.lastage_value) > 1: # to reduce cpu, if the time didn't change by a second
            self.lastage_value = self.value
            self.lastage = readable_timespan(self.value)
        return '"'+self.lastage+'"'

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

def CalibrationProcess(cal_pipe, client):
    import calibration_fit
    calibration_fit.CalibrationProcess(cal_pipe, client)

class AutomaticCalibrationProcess(multiprocessing.Process):
    def __init__(self, server):
        #self.cal_pipe, cal_pipe = NonBlockingPipe('cal pipe', True)
        self.cal_pipe, cal_pipe = False, False # use client
        client = pypilotClient(server)
        super(AutomaticCalibrationProcess, self).__init__(target=CalibrationProcess, args=(cal_pipe, client), daemon=True)
        self.start()

    def __del__(self):
        print('terminate calibration process')
        self.terminate()


class BoatIMU(object):
    def __init__(self, client):
        self.client = client

        self.rate = self.register(EnumProperty, 'rate', 10, [10, 25], persistent=True)

        self.frequency = self.register(FrequencyValue, 'frequency')
        self.alignmentQ = self.register(QuaternionValue, 'alignmentQ', [2**.5/2, -2**.5/2, 0, 0], persistent=True)
        self.alignmentQ.last = False
        self.heading_off = self.register(RangeProperty, 'heading_offset', 0, -180, 180, persistent=True)
        self.heading_off.last = 3000 # invalid

        self.alignmentCounter = self.register(Property, 'alignmentCounter', 0)
        self.last_alignmentCounter = False

        self.uptime = self.register(TimeValue, 'uptime')
    
        self.auto_cal = AutomaticCalibrationProcess(client.server)

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

        self.imu = IMU(client.server)

        self.last_imuread = time.monotonic()

    def __del__(self):
        #print('terminate imu process')
        #self.imu.process.terminate()
        pass

    def register(self, _type, name, *args, **kwargs):
        value = _type(*(['imu.' + name] + list(args)), **kwargs)
        return self.client.register(value)
      
    def update_alignment(self, q):
        a2 = 2*math.atan2(q[3], q[0])
        heading_offset = a2*180/math.pi
        off = self.heading_off.value - heading_offset
        o = quaternion.angvec2quat(off*math.pi/180, [0, 0, 1])
        self.alignmentQ.update(quaternion.normalize(quaternion.multiply(q, o)))

    def IMUread(self):
        if self.imu.multiprocessing:
            lastdata = False
            while True:
                data = self.imu.pipe.recv()
                if not data:
                    return lastdata
                lastdata = data
        return self.imu.read()

    def poll(self):
        if not self.imu.multiprocessing:
            self.imu.poll()

    def read(self):
        data = self.IMUread()
        if not data:
            if time.monotonic() - self.last_imuread > 1 and self.frequency.value:
                print('IMURead failed!')
                self.frequency.set(False)
                for name in self.SensorValues:
                    self.SensorValues[name].set(False)
                self.uptime.reset()
            return False
  
        if vector.norm(data['accel']) == 0:
            print('accel values invalid', data['accel'])
            return False
  
        self.last_imuread = time.monotonic()
        self.frequency.strobe()
  
        # apply alignment calibration
        gyro_q = quaternion.rotvecquat(data['gyro'], data['fusionQPose'])
    
        data['pitchrate'], data['rollrate'], data['headingrate'] = map(math.degrees, gyro_q)
        
        aligned = quaternion.multiply(data['fusionQPose'], self.alignmentQ.value)
        aligned = quaternion.normalize(aligned) # floating point precision errors
    
        data['roll'], data['pitch'], data['heading'] = map(math.degrees, quaternion.toeuler(aligned))
  
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
            self.alignmentPose = list(map(lambda x, y : x + y, self.alignmentPose, aligned))
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


        if self.auto_cal.cal_pipe:
            print('warning, cal pipe always sending despite locks')
            cal_data = {}
            #how to check this here??  if not 'imu.accel.calibration.locked'
            cal_data['accel'] = list(data['accel'])
            
            #how to check this here??  if not 'imu.compass.calibration.locked'
            cal_data['compass'] = list(data['compass'])
            cal_data['down'] = quaternion.rotvecquat([0, 0, 1], quaternion.conjugate(data['fusionQPose']))

            if cal_data:
                self.auto_cal.cal_pipe.send(cal_data)

        return data

# print line without newline
def printline(*args):
    for a in args:
        sys.stdout.write(str(a))
        sys.stdout.write(' ')
    sys.stdout.write('\r')
    sys.stdout.flush()
    
def main():
    from server import pypilotServer
    server = pypilotServer()
    client = pypilotClient(server)
    boatimu = BoatIMU(client)

    quiet = '-q' in sys.argv

    while True:
        t0 = time.monotonic()
        server.poll()
        client.poll()
        data = boatimu.read()
        if data and not quiet:
            printline('pitch', data['pitch'], 'roll', data['roll'], 'heading', data['heading'])
        boatimu.poll()
        while True:
            dt = 1/boatimu.rate.value - (time.monotonic() - t0)
            if dt < 0:
                break
            if dt > 0:
                time.sleep(dt)
            
if __name__ == '__main__':
    main()
