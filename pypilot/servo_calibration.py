#!/usr/bin/env python
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time, sys, json, os
from signalk.client import SignalKClient
from servo import *

import threading
import numpy, scipy.optimize

# fit to order n
def fit(x, n):
    def func(b, x, n):
        res = -x[1]
        for o in range(n+1):
            res = res + b[o]*x[0]**o
        return res
    a = numpy.array(x)
    b = scipy.optimize.leastsq(func, [0]*(n+1), (a, n))[0]
    t = 0
    for v in func(b, a, n):
        t += v**2
    return list(b), t**.5

def fit_str(fit):
    s = ''
    for o in range(len(fit)):
        if o:
            s += ' + '
        s += '%.3f*x**'%fit[o]+str(o)
    return s

def FitCalibration(cal):
    speeds = []
    commands = []
    for speed in cal:
        # only care about speed and raw command for now...
        raw_cmd, idle_current, stall_current, idle_voltage, dt, power = cal[speed]
        speeds.append(speed)
        commands.append(raw_cmd)
    
    fits = {}
    print 'speeds', len(speeds)
    print 'plot'
    for val in zip(speeds, commands):
        print val[0], val[1]

    for n in [1, 3, 5]:  # linear, 3rd and 5th order fit
        if len(speeds) > n+1:
            fits[n] = fit([speeds, commands], n)
        else:
            fits[n] = False
            
        print 'fit order', n, fits[n]
        if fits[n]:
            print fit_str(fits[n][0])

    # did third order help much?
    #if fits[3][1] / fits[1][1] < .9:
    #    print 'warning non-linear servo!'
    if fits[1]:
        return map(float, fits[1][0]) # first order, good enough
    else:
        return False

period_speed = .6
printconsole = False

def ServoCalibrationThread(calibration):
    servo = calibration.servo

    def console(*text):
        c = ''
        for t in text:
            c += t + ' '
        calibration.console.set(c)
        if printconsole:
            print c
    
    def command(value):
        if self.fwd_fault and value < 0:
            servo.fwd_fault = False
        elif self.rev_fault and value > 0:
            servo.rev_fault = False
        engauge()
        calibration.raw_command.set(value)

    def stop():
        command(0)

    def reset(self):
        self.stop()
        if not self.waitnofault(7):
            print 'servo reset failed', self.fault(), self.data['servo/flags']
            exit(1)

    def average_power(self, timeout):
        start = time.time()
        self.log = []
        time.sleep(timeout)
        power, avgc, avgv = 0, 0, 0
        if self.log.len():
            lasttime = log[0][2]
            for l in self.log:
                voltage, current, time = l
                dt = time - lasttime
                avgc += dt*current
                avgv += dt*voltage
                power+= dt*voltage*current
            avgv /= len(self.log)
            avgc /= len(self.log)
                
        return avgc, avgv, power

    def calibrate_period(raw_cmd, period, idle_current):
        #print 'reset, and cool down'
        reset()
        for t in range(20):
            sys.stdout.write('%d ' % t)
            sys.stdout.flush()
            self.average_power(1) # wait to reset and cool
        print 'start'
        t0 = time.time()
        transitions = 0
        #print 'run', raw_cmd, period
        while True:
            def period_command(raw_cmd):
                t = time.time()
                while time.time() - t <= period - .05:
                    command(raw_cmd)
                    if self.fault() and time.time() - t0 > 3:
                        return True
                    #print 'idle current', idle_current, self.data['servo/current']
                    if time.time() - t0 > 3 and self.data['servo/current'] > 1.6 * idle_current:
                        #print 'max current', self.data['servo/current'], idle_current
                        return True
                    time.sleep(.1)
                return False

            t1 = time.time()
            transitions += 1
            if period_command(raw_cmd):
                break
            period_command(0)

        dt = t1-t0 + (time.time() - t1)*2
        current, voltage, t = self.average_power(0)
        power = current*voltage*dt
        truespeed = 1/dt
        print transitions, truespeed, 'plota' if raw_cmd > 0 else 'plotb'
        return current, voltage, transitions, dt, power

    
    def calibrate_speed(raw_cmd):
        self.reset()
        for t in range(10):
            sys.stdout.write('%d ' % t)
            sys.stdout.flush()
            self.average_power(1) # wait to reset and cool
        servo.max_current.set(10)

        #console('calibration raw command:', raw_cmd)
        t0 = False

        idle_current = idle_voltage = False
        stall_current = False
        prevcurrent = False
        lp_current = 0
        count = 0
        power = 0
        while True:
            command(raw_cmd)

            avgtime = .3

            voltage, current, p = average_power(avgtime)
            power += p
            
            if not t0:
                t0 = t
            if t-t0 >= self.timeout:
                break

            #print 'current', current, idle_current, lp_current
            if idle_current and (current > idle_current*1.3 or lp_current > idle_current * 1.1):
                console('found stall current', current, lp_current)

                voltage, current, t1 = self.average_power(.1)
                console('settled to stall current', current)

                stall_current = current
            elif fault():
                if not idle_current:
                    console('motor fault without finding idle current:', raw_cmd)
                    return False
                console('stall current above max current for raw_cmd:', raw_cmd)
                stall_current = servo.max_current
                
            elif idle_current and current > servo.max_current:
                console('servo failed to stop overcurrent!!!!', current)
                stall_current = servo.max_current.value

            if stall_current:
                return [raw_cmd, idle_current, stall_current, idle_voltage, t - t0, power]

            if not idle_current and count > 4 and current > 0 and prevcurrent and \
               abs(prevcurrent - current) < .03:
                console('found idle current', current)
                servo.max_current(current*1.8)

                idle_current = current
                idle_voltage = voltage
                
            prevcurrent = current

            lp_current = .9*lp_current + .1*current
            count += 1

        print 'timeout calibrating raw_cmd', raw_cmd
        return False


    def safe_raw_cmd(self, d):
        if self.brake_hack and d > 0:
            d *= 1.4
        return period_speed*d # 60% of full speed should always work??
    
    def search_end(sign):
        self.reset()
        self.waitfault(1)
        console('moving away from end')
        self.reset()
        self.command(self.safe_raw_cmd(-sign))
        self.waitfault(4)
        self.reset()
        self.waitfault(1)
        console('continuing to end at', self.safe_raw_cmd(sign))
        calibration = self.calibrate_speed(self.safe_raw_cmd(sign))
        if not calibration:
            console('failed to reach end at safe speed')
            return False
        console('reached end')
        return calibration

    # calibrate!
    brake_hack = False
    servo.brake_hack.set(brake_hack)
    servo.max_current.set(10)
    servo.disengauge_on_timeout.set(False)
    calibration.raw_command(0)

    timeout = 40 # max time to move end to end

    cal = self.search_end(-1)
    if not cal:
        console('failed to reset servo position to start')
        console('Trying with brake hack')
        self.brake_hack = True
        self.client.set('servo/brake_hack', self.brake_hack)
        cal = self.search_end(-1)
        if not cal:
            console('failed to reset servo position to start')
            exit(1)
        
    print 'initial cal', cal
    command, idle_current, stall_current, cal_voltage, dt, power = cal
    truespeed = 1/dt
    max_current = idle_current + .75*(stall_current - idle_current)

    reset()        
    #self.client.set('servo/max_current', max_current)
                        
    console('max current found', max_current)
    console('found start')

    if False:
        period = .2
        for d in range(12):
            print 'period', period
            cal = self.calibrate_period(period_speed, period, idle_current)
            print 'fwd', cal
            cal = self.calibrate_period(-period_speed, period, idle_current)
            print 'rev', cal
            period *= 1.5
                                          
        exit(0)
        
    calibration = {} # clear old cal
        
    complete = [False, False]
    lastspeed = [0, 0]
    steps = 14 # speeds 20 speeds
    mincmd = 400
    maxcmd = 750
    stepi = 0
    for abs_raw_cmd in range(mincmd, maxcmd, (maxcmd - mincmd)/(steps-1)):
        for signi in [0, 1]:
            sign = 1-2*signi
            raw_cmd = sign*abs_raw_cmd/1000.0
            #if signi == 0:
            #    raw_cmd += .03
            self.stop()
            #console('flags', self.data['servo/flags'], 'fault', self.fault())
            console('%.1f%%' % (stepi*100.0/2/steps), 'step', stepi, 'of', 2*steps, 'raw command', raw_cmd)
            stepi += 1
            cal = self.calibrate_speed(raw_cmd)
            if cal:
                command, idle_current, stall_current, cal_voltage, dt, power = cal
                truespeed = 1/dt
                print 'truespeed', truespeed, 'lastspeed', lastspeed[signi], 'power', power
                if lastspeed[signi] and truespeed / lastspeed[signi] < 1+.1/steps:
                    complete[signi] += 1
                    console('completed this direction when counter >= 3:', complete[signi])
                else:
                    complete[signi] = 0

#                    c = map(lambda x : sign*x, calibration)
#                    if len(c) == 0 or truespeed > sorted(c)[len(c)-1]:
#                        print 'update cal', cal
                    calibration[sign*truespeed] = cal
                    lastspeed[signi] = max(truespeed, lastspeed[signi])
            else:
                if not self.search_end(sign):
                    print 'failed to find end'
                    exit(0)

        if complete[0] >= 3 and complete[1] >= 3:
            console('higher commands do not yield higher speeds: finished')
            break

    if complete[0] < 3 or complete[1] < 3:
        console('did not reach highest speed')

    self.stop()
    self.client.set('servo/raw_command', 0)

    #print 'calibration:', calibration
    # normalize calibration speed from 0 to 1
    speeds = list(calibration)
    max_fwd_speed = max(speeds)
    max_rev_speed = -min(speeds)

    ratio = max_fwd_speed / max_rev_speed
    print 'fwd/rev', ratio
    if ratio > 1.1 or ratio < .9:
        print 'warning: very unbalanced ratio in forward/reverse speed'

    max_speed = min(max_rev_speed, max_fwd_speed)
    print 'max speed', max_speed

    fwd_calibration = {}
    rev_calibration = {}
    for truespeed in calibration:
        # discard speeds above 85% full speed for calibration
        # to avoid using saturated results in the calibration fit
        if abs(truespeed) <= max_speed*.99 or True:
            speed = truespeed / max_speed
            cal = calibration[truespeed]
            if truespeed > 0:
                fwd_calibration[speed] = cal
            else:
                rev_calibration[-speed] = [-cal[0]] + cal[1:]
        else:
            print 'discarding speed, outside of range', truespeed

    min_fwd_speed = min(fwd_calibration)
    min_rev_speed = min(rev_calibration)
    min_speed = max(min_fwd_speed, min_rev_speed)

    #print 'forward calibration', fwd_calibration
    fwdfit = FitCalibration(fwd_calibration)
    #print 'reverse calibration', rev_calibration
    revfit = FitCalibration(rev_calibration)        
    cal = {'forward': fwdfit, 'reverse': revfit, 'Min Speed': min_speed, 'Brake Hack': self.brake_hack}

    f = open(os.getenv('HOME') + '/.pypilot/servocalibration', 'w')
    f.write(json.dumps(cal))
    console('calibration complete')

        
class ServoCalibration(object):
    def __init__(self, servo):
        self.server = servo.server
        self.run = self.Register(BooleanProperty, 'run', False)
        self.rawcommand = self.Register(SensorValue, 'raw_command', self.server.TimeStamp('servo'))
        self.console = self.Register(Value, 'console', '')
        self.current_total = self.voltage_total = 0, 0
        self.servo = servo
        self.thread = threading.Thread(target=ServoCalibrationThread, args=(self, ))
        self.rawcommand.set(0)

    def raw_command(self, value):
        self.rawcommand.set(value)

    def Register(self, _type, name, *args, **kwargs):
        return self.server.Register(_type(*(['servo/calibration/' + name] + list(args)), **kwargs))

        def fault(self):
            return (ServoFlags.OVERCURRENT | ServoFlags.FALTPIN) & self.servo.flags.value or \
                not self.servo.engauged.value
    
    def poll(self):
        if not self.thread.is_alive():
            if self.run.value: # calibration should start
                self.command = self.servo.command.value
                self.raw_command = self.servo.raw_command.value
                self.servo.brake_hack.set(False)
                self.log = []
                self.state = 0
                self.thread = thread.start()
            else:
                return

        if not self.run.value or self.ap.enabled.value:
            self.thread.exit()
            return
        
        if self.command != self.servo.command.value:
            console('servo command received, aborting')
            console('ensure the autopilot is not active and')
            console('no manual servo commands during calibration!')
            self.command = self.servo.command.value
            self.thread.exit(0)

        self.log.append([self.servo.voltage.value, self.servo.current.value, self.servo.current.time])

        if self.fwd_fault and self.rawcommand.value < 0:
            self.fwd_fault = False
        elif self.rev_fault and self.rawcommand.value > 0:
            self.rev_fault = False
        self.servo.engauge()
        self.servo.raw_command(self.rawcommand.value)

    def stop(self):
        if self.thread.is_alive():
            self.thread.exit()

def round_any(x, n):
    if type(x) == type({}):
        r = {}
        for v in x:
            r[v] = round_any(x[v], n)
        return r
    elif type(x) == type([]):
        return map(lambda v : round_any(v, n), x)
    elif type(x) == type(0.0):
        return round(x, n)
    else:
        return x

if __name__ == '__main__':
    import serialprobe
    print 'Servo Server'
    printconsole = True
    server = SignalKServer()
    serial_probe = serialprobe.SerialProbe()
    servo = Servo(server, serial_probe)
    servo.servo_calibration.run = True

    while True:
        servo.poll()
        servo.send_command()
        server.HandleRequests()
        time.sleep(.1)

