#!/usr/bin/env python
#
#   Copyright (C) 2023 Sean D'Epagnier
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

try:
    import vector, quaternion
    from client import pypilotClient
    from values import *

    from nonblockingpipe import NonBlockingPipe
except:
    import failedimports

try:
    import RTIMU
except ImportError:
    RTIMU = False
    print(_('RTIMU library not detected, please install it'))

class IMU(object):
    def __init__(self, server):
        self.client = pypilotClient(server)
        self.multiprocessing = server.multiprocessing
        if self.multiprocessing:
            self.pipe, pipe = NonBlockingPipe('imu pipe', self.multiprocessing)
            self.process = multiprocessing.Process(target=self.process, args=(pipe,), daemon=True)
            self.process.start()
            return
        self.process = False
        self.setup()
          
    def setup(self):
        self.client.watch('imu.accel.calibration')
        self.client.watch('imu.compass.calibration')
        self.client.watch('imu.rate')

        self.gyrobias = self.client.register(SensorValue('imu.gyrobias', persistent=True))
        self.warning = self.client.register(StringValue('imu.warning', ''))
        self.lastgyrobiastime = time.monotonic()

        SETTINGS_FILE = "RTIMULib"
        print(_('Using settings file') + ' ' + SETTINGS_FILE + '.ini')
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

        s.GyroBiasValid = False
        s.GyroBias = (0, 0, 0)

        s.KalmanRk, s.KalmanQ = .002, .001
        self.s = s
        self.imu_detect_time = 0
        self.rtimu = True
        self.init()
        self.lastdata = False
        self.rate = 10

    def init(self):
        t0 = time.monotonic()
        self.s.IMUType = 0 # always autodetect imu
        # avoid detecting so often filling log file
        if t0 - self.imu_detect_time < 1:
            return
        self.imu_detect_time = t0

        rtimu = RTIMU.RTIMU(self.s)
        if rtimu.IMUName() == 'Null IMU':
            if self.rtimu:
                print(_('ERROR: No IMU Detected'), t0)
                self.warning.set('No IMU')
            self.s.IMUType = 0
            self.rtimu = False
            return

        print('IMU Name: ' + rtimu.IMUName())

        if not rtimu.IMUInit():
            print(_('ERROR: IMU Init Failed, no inertial data available'), t0)
            self.warning.set('IMU Failed')
            self.s.IMUType = 0
            return

        # this is a good time to set any fusion parameters
        rtimu.setSlerpPower(.01)
        rtimu.setGyroEnable(True)
        rtimu.setAccelEnable(True)
        rtimu.setCompassEnable(True)
        time.sleep(.1)
        self.rtimu = rtimu

        self.avggyro = [0, 0, 0]
        self.compass_calibration_updated = False
        self.axes_test = [False]*9
        self.last_axes = False
        self.warning.set('IMU not initialized')

    def process(self, pipe):
        print('imu process', os.getpid())
        if not RTIMU:
            while True:
                time.sleep(10) # do nothing

        if os.system('sudo chrt -pf 2 %d 2>&1 > /dev/null' % os.getpid()):
            print(_('warning, failed to make imu process realtime'))
        else:
            print(_('made imu process realtime'))

        self.setup()
        while True:
            t0 = time.monotonic()
            data = self.read()
            t1 = time.monotonic()
            pipe.send(data, not data)
            t2 = time.monotonic()

            if not self.s.GyroBiasValid:
                if self.gyrobias.value:
                    print(_('setting initial gyro bias'), self.gyrobias.value)
                    self.s.GyroBias = tuple(map(math.radians, self.gyrobias.value))
                    self.s.GyroBiasValid = True
            if t0-self.lastgyrobiastime > 30:
                self.gyrobias.set(list(map(math.degrees, self.s.GyroBias)))
                self.lastgyrobiastime = t0
                self.s.GyroBiasValid = True
            
            self.poll()
            t3 = time.monotonic()
            dt = time.monotonic() - t0
            period = 1/self.rate
            t = period - dt
            if t > 0 and t < period:
                time.sleep(t)
            else:
                print(_('imu process failed to keep time'), dt, t0, t1, t2, t3)

    def read(self):
        t0 = time.monotonic()
        if not self.s.IMUType:
            self.init()
            return False
        if not self.rtimu.IMURead():
            print(_('failed to read IMU!'), t0)
            self.init() # reinitialize imu
            return False 
         
        data = self.rtimu.getIMUData()
        data['accel.residuals'] = list(self.rtimu.getAccelResiduals())

        data['timestamp'] = t0 # imu timestamp is perfectly accurate
        
        if self.compass_calibration_updated:
            data['compass_calibration_updated'] = True
            self.compass_calibration_updated = False

        self.lastdata = list(data['accel']), list(data['gyro']), list(data['compass'])
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
                self.rtimu.resetFusion()
            elif name == 'imu.rate':
                self.rate = value
                print(_('imu rate set to rate'), value)

        if not self.lastdata:
            return

        accel, gyro, compass = self.lastdata
        self.lastdata = False

        if self.axes_test: # test to see if all axes are producing changing outputs
            axes = accel + gyro + compass
            if self.last_axes:
                self.axes_test = map(lambda a, b, p: p or (a != b), axes, self.last_axes, self.axes_test)
            self.last_axes = axes

            if not all(self.axes_test):
                self.warning.set('IMU waiting on axes')
            else:
                print('IMU all sensor axes verified')
                self.warning.set('')
                self.axes_test = False
                
        # see if gyro is out of range, sometimes the sensors read
        # very high gyro readings and the sensors need to be reset by software
        # this is probably a bug in the underlying driver with fifo misalignment
        d = .05/self.rate # filter constant
        for i in range(3): # filter gyro vector
            self.avggyro[i] = (1-d)*self.avggyro[i] + d*gyro[i]
        if vector.norm(self.avggyro) > .8: # 55 degrees/s
            print(_('too high standing gyro bias, resetting sensors'), gyro, self.avggyro)
            self.init()

        # detects the problem even faster:
        if any(map(lambda x : abs(x) > 1000, compass)):
            print(_('compass out of range, resetting'), compass)
            self.init()

class FrequencyValue(SensorValue):
    def __init__(self, name):
        super(FrequencyValue, self).__init__(name)
        self.loopc = 0
        self.t0 = time.monotonic()

    def strobe(self):
        self.loopc += 1
        if self.loopc == 4: # update timing from 5 cycles
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
    if os.system('sudo chrt -po 0 %d 2> /dev/null > /dev/null' % os.getpid()):
        print(_('warning, failed to make calibration process other'))
    if os.system('sudo chrt -pi 0 %d 2> /dev/null > /dev/null' % os.getpid()):
        print(_('warning, failed to make calibration process idle, trying renice'))
        if os.system("renice 20 %d" % os.getpid()):
            print(_('warning, failed to renice calibration process'))

    time.sleep(4)

    while True:
        try:
            import calibration_fit
            print(_('calibration loaded, starting'), os.getpid())
            cal_pipe.send('ready')
            break
        except Exception as e:
            print(_('failed import calibration fit'), e)
            time.sleep(30) # maybe numpy or scipy isn't ready yet
    calibration_fit.CalibrationProcess(cal_pipe, client) # does not return

class AutomaticCalibrationProcess():
    def __init__(self, server):
        if True:
            # direct connection to send raw sensors to calibration process is more
            # efficient than routing through server (save up to 2% cpu on rpi zero)
            self.cal_pipe, self.cal_pipe_process = NonBlockingPipe('cal pipe', True, sendfailok=True)
        else:
            self.cal_pipe, self.cal_pipe_process = False, False # use client
        self.client = pypilotClient(server)
        self.process = multiprocessing.Process(target=CalibrationProcess, args=(self.cal_pipe_process, self.client), daemon=True)
        self.process.start()
        self.cal_ready = False

    def calibration_ready(self):
        if self.cal_ready:
            return True
        if self.cal_pipe.recv():
            self.cal_ready = True
            return True
        return False

    def __del__(self):
        #print(_('terminate calibration process'))
        self.process.terminate()


class BoatIMU(object):
    def __init__(self, client):
        self.client = client

        self.rate = self.register(EnumProperty, 'rate', 20, [10, 20], persistent=True)

        self.frequency = self.register(FrequencyValue, 'frequency')
        self.alignmentQ = self.register(QuaternionValue, 'alignmentQ', [1, 0, 0, 0], persistent=True)
        self.alignmentQ.last = False
        self.heading_off = self.register(RangeProperty, 'heading_offset', 0, -180, 180, persistent=True)
        self.heading_off.last = 3000 # invalid

        self.alignmentCounter = self.register(Property, 'alignmentCounter', 0)
        self.last_alignmentCounter = False

        self.uptime = self.register(TimeValue, 'uptime')
        
        self.auto_cal = AutomaticCalibrationProcess(client.server)

        self.lasttimestamp = 0

        self.headingrate = self.heel = 0
        self.heading_lowpass_constant = self.register(RangeProperty, 'heading_lowpass_constant', .2, .05, .3)
        self.headingrate_lowpass_constant = self.register(RangeProperty, 'headingrate_lowpass_constant', .2, .05, .3)
        self.headingraterate_lowpass_constant = self.register(RangeProperty, 'headingraterate_lowpass_constant', .1, .05, .3)

        sensornames = ['accel', 'gyro', 'compass', 'accel.residuals', 'pitch', 'roll']
        sensornames += ['pitchrate', 'rollrate', 'headingrate', 'headingraterate', 'heel']
        sensornames += ['headingrate_lowpass', 'headingraterate_lowpass']
        directional_sensornames = ['heading', 'heading_lowpass']
        sensornames += directional_sensornames
    
        self.SensorValues = {}
        for name in sensornames:
            self.SensorValues[name] = self.register(SensorValue, name, directional = name in directional_sensornames)

        # quaternion needs to report many more decimal places than other sensors
        #sensornames += ['fusionQPose']
        self.SensorValues['fusionQPose'] = self.register(SensorValue, 'fusionQPose', fmt='%.8f')
    
        self.imu = IMU(client.server)

        self.last_imuread = time.monotonic() + 4 # ignore failed readings at startup
        self.cal_data = False
        self.reset_alignment = False

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
        self.reset_alignment = True

    def IMUread(self):
        if self.imu.multiprocessing:
            lastdata = False
            while True:
                data = self.imu.pipe.recv()
                if not data:
                    return lastdata
                lastdata = data
        return self.imu.read()

    def read(self):
        if not self.imu.multiprocessing:
            self.imu.poll()

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
            print(_('accel values invalid'), data['accel'])
            return False
  
        self.last_imuread = time.monotonic()
        self.frequency.strobe()
  
        # apply alignment calibration                                       
        aligned = quaternion.multiply(data['fusionQPose'], self.alignmentQ.value)
        aligned = quaternion.normalize(aligned) # floating point precision errors
    
        data['roll'], data['pitch'], data['heading'] = map(math.degrees, quaternion.toeuler(aligned))
        if data['heading'] < 0:
            data['heading'] += 360

        gyro_q = quaternion.rotvecquat(data['gyro'], data['fusionQPose'])

        # optimized below
        #gyro_q = quaternion.rotvecquat(gyro_q, quaternion.angvec2quat(-math.radians(data['heading']), [0, 0, 1]))
        ur, vr, data['headingrate'] = map(math.degrees, gyro_q)
        rh = math.radians(data['heading'])
        srh = math.sin(rh)
        crh = math.cos(rh)
        data['rollrate'] = ur*crh + vr*srh
        data['pitchrate'] = vr*crh - ur*srh
  
        dt = data['timestamp'] - self.lasttimestamp
        self.lasttimestamp = data['timestamp']
        if dt > .01 and dt < .2:
            data['headingraterate'] = (data['headingrate'] - self.headingrate) / dt
        else:
            data['headingraterate'] = 0
  
        self.headingrate = data['headingrate']
  
        data['heel'] = self.heel = data['roll']*.03 + self.heel*.97
        #data['roll'] -= data['heel']
  
        data['gyro'] = list(map(math.degrees, data['gyro']))
  
        # lowpass heading and rate
        llp = self.heading_lowpass_constant.value
        if self.reset_alignment or data.get('compass_calibration_updated'):
            llp = 1
            self.reset_alignment = False
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
        if self.alignmentCounter.value > 0:
            if self.alignmentCounter.value != self.last_alignmentCounter:
                self.alignmentPose = [0, 0, 0, 0]
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

        #  warning, cal pipe always sending despite locks
        #how to check this here??  if not 'imu.accel.calibration.locked'
        #how to check this here??  if not 'imu.compass.calibration.locked'
        self.cal_data = {'accel': data['accel'],
                         'compass': data['compass'],
                         'fusionQPose': data['fusionQPose']}
        return data

    def send_cal_data(self):
        if self.auto_cal.calibration_ready() and self.cal_data:
            self.auto_cal.cal_pipe.send(self.cal_data)


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

    lastprint = 0
    while True:
        t0 = time.monotonic()
        server.poll()
        client.poll()
        data = boatimu.read()
        if data and not quiet:
            if t0-lastprint > .25:
                printline('pitch', data['pitch'], 'roll', data['roll'], 'heading', data['heading'])
                lastprint = t0
        while True:
            dt = 1/boatimu.rate.value - (time.monotonic() - t0)
            if dt < 0:
                break
            if dt > 0:
                time.sleep(dt)
            
if __name__ == '__main__':
    main()
