#!/usr/bin/env python
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time, sys, json, os
from signalk.client import SignalKClient

def console(*text):
    for t in text:
        sys.stdout.write(str(t))
        sys.stdout.write(' ')
    print ''

import numpy
import scipy.optimize

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
    
class ServoClient(object):
    def __init__(self):
        watches = ['servo/voltage', 'servo/current', 'servo/engauged', 'servo/command', 'servo/raw_command', 'servo/max_current', 'servo/flags']
        def on_con(client):
            for name in watches:
                client.watch(name)
        host = False
        if len(sys.argv) > 1:
            host = sys.argv[1]
        self.client = SignalKClient(on_con, host=host
)
                
        # should only run one instance, for now hardcode local connection
        self.data = {'servo/flags': '', 'servo/engauged': False}
        self.current_total = self.voltage_total = 0, 0

        self.first_command = False
        self.lastcommand = self.lastlastcommand = 0

    def fault(self):
        return 'OVERCURRENT' in self.data['servo/flags'] or \
            'OVERTEMP' in self.data['servo/flags'] or \
            'FAULTPIN' in self.data['servo/flags'] or \
            not self.data['servo/engauged']

    def process_messages(self):
        data = self.client.receive()
        for name in data:
            if name == 'servo/command':
                if self.first_command:
                    console('servo command received, aborting')
                    console('ensure the autopilot is not active and')
                    console('no manual servo commands during calibration!')
                    exit(1)
                self.first_command = True
            elif name == 'servo/raw_command':
                cmd = data[name]['value']
                if abs(cmd - self.lastcommand) > .01 and abs(cmd - self.lastlastcommand) > .01:
                    print cmd, self.lastcommand, self.lastlastcommand,  cmd-self.lastcommand, cmd - self.lastlastcommand
                    console('raw servo command received, aborting')
                    console('ensure no other calibration client is running')
                    exit(1)
            elif name == 'servo/voltage':
                #print 'voltage', data[name]['value'], data[name]['timestamp']
                self.voltage_total = self.voltage_total[0] + data[name]['value'], \
                                     self.voltage_total[1] + 1
            elif name == 'servo/current':
                self.current_total = self.current_total[0] + data[name]['value'], \
                                     self.current_total[1] + 1

                self.currenttimestamp = data[name]['timestamp']
            self.data[name] = data[name]['value']

    
    def waitnofault(self, timeout):
        return self.waitfault(timeout, True)

    def waitfault(self, timeout, waitnofault=False):
        t0 = time.time()
        while waitnofault == self.fault():
            self.process_messages()

            if time.time() - t0 >= timeout:
                return False
            time.sleep(.1)
            self.client.set('servo/raw_command', self.lastcommand)
        return True

    def command(self, value):
        self.client.set('servo/raw_command', value)
        if value != self.lastcommand:
            self.lastlastcommand = self.lastcommand
        self.lastcommand = value

    def stop(self):
        self.command(0)
        
    def reset(self):
        self.stop()
        if not self.waitnofault(7):
            print 'servo reset failed', self.fault(), self.data['servo/flags']
            exit(1)

    def average_power(self, timeout):
        start = time.time()
        self.currenttimestamp = False
        while time.time() - start < timeout or self.current_total[1] == 0:
            self.process_messages()
            if self.fault():
                #print 'fault in avg power, set the servo current higher!'
                dt = timeout - (time.time() - start)
                if dt > 0:
                    time.sleep(dt)
                break

            time.sleep(.01)

        if self.voltage_total[1] == 0:
            voltage = self.data['servo/voltage']
        else:
            voltage = self.voltage_total[0] / self.voltage_total[1]
        if self.current_total[1] == 0:
            current = 0
        else:
            current = self.current_total[0] / self.current_total[1]
        self.voltage_total = 0, 0
        self.current_total = 0, 0
        return voltage, current, self.currenttimestamp

    def calibrate_period(self, raw_cmd, period, idle_current):
        #print 'reset, and cool down'
        self.reset()
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
                    self.command(raw_cmd)
                    self.process_messages()
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

    
    def calibrate_speed(self, raw_cmd):
        self.reset()
        for t in range(10):
            sys.stdout.write('%d ' % t)
            sys.stdout.flush()
            self.average_power(1) # wait to reset and cool
        self.client.set('servo/max_current', 10)

        #console('calibration raw command:', raw_cmd)
        t0 = False

        idle_current = idle_voltage = False
        stall_current = False
        prevcurrent = False
        lp_current = 0
        count = 0
        power = 0
        lastt = self.currenttimestamp
        while True:
            self.command(raw_cmd)

            avgtime = .3

            voltage, current, t = self.average_power(avgtime)
            dt = t - lastt
            lastt = t

            power += voltage * current * dt
            
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
            elif self.fault():
                if not idle_current:
                    console('motor fault without finding idle current:', raw_cmd)
                    return False
                console('stall current above max current for raw_cmd:', raw_cmd)
                stall_current = self.data['servo/max_current']
                
            elif idle_current and current > self.data['servo/max_current']:
                console('servo failed to stop overcurrent!!!!', current)
                stall_current = self.data['servo/max_current']

            if stall_current:
                return [raw_cmd, idle_current, stall_current, idle_voltage, t - t0, power]

            if not idle_current and count > 4 and current > 0 and prevcurrent and \
               abs(prevcurrent - current) < .03:
                console('found idle current', current)
                self.client.set('servo/max_current', current*1.8)

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
    
    def search_end(self, sign):
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

    def calibrate(self):
        self.brake_hack = False
        self.client.set('ap/enabled', False)
        self.client.set('servo/brake_hack', self.brake_hack)
        self.client.set('servo/max_current', 10)
        self.client.set('servo/disengauge_on_timeout', False)
        self.client.set('servo/raw_command', 0)

        self.timeout = 40 # max time to move end to end

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

        self.reset()        
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
        return {'forward': fwdfit, 'reverse': revfit, 'Min Speed': min_speed, 'Brake Hack': self.brake_hack}

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
    cal = ServoClient().calibrate()
    cal = round_any(cal, 5)
    print 'result', cal

    f = open(os.getenv('HOME') + '/.pypilot/servocalibration', 'w')
    f.write(json.dumps(cal))
    console('calibration complete')
