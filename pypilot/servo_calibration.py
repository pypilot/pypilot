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
        raw_cmd, idle_current, stall_current, idle_voltage, dt = cal[speed]
        speeds.append(speed)
        commands.append(raw_cmd)
    
    fits = {}
    print 'speeds', len(speeds)
    print 'speeds', 'commands', [speeds, commands]
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
    
class ServoClient(object):
    def __init__(self):
        watches = ['servo/voltage', 'servo/current', 'servo/engauged', 'servo/command', 'servo/raw_command', 'servo/Max Current', 'servo/flags']
        def on_con(client):
            for name in watches:
                client.watch(name)
        self.client = SignalKClient(on_con)
                
        # should only run one instance, for now hardcode local connection
        self.data = {'servo/engauged': False}
        self.current_total = self.voltage_total = 0, 0

        self.first_command = False
        self.lastcommand = self.lastlastcommand = 0

    def fault(self):
        return not self.data['servo/engauged']

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
            print 'servo reset failed'
            exit(1)

    def average_power(self, timeout):
        start = time.time()
        self.currenttimestamp = False
        while time.time() - start < timeout or self.current_total[1] == 0:
            self.process_messages()
            if self.fault():
                print 'fault in avg power, set the servo current higher!'
                break
            time.sleep(.01)

        if self.voltage_total[1] == 0:
            voltage = self.data['servo/voltage']
        else:
            voltage = self.voltage_total[0] / self.voltage_total[1]
        current = self.current_total[0] / self.current_total[1]
        self.voltage_total = 0, 0
        self.current_total = 0, 0
        return voltage, current, self.currenttimestamp

    def calibrate_speed(self, raw_cmd):
        self.reset()
        self.average_power(1) # wait a second
        #console('calibration raw command:', raw_cmd)
        t0 = False

        idle_current = idle_voltage = False
        stall_current = False
        prevcurrent = False
        lp_current = 0
        count = 0
        while True:
            self.command(raw_cmd)

            if idle_current:
                avgtime = .2
            else:
                avgtime = .5

            voltage, current, t = self.average_power(avgtime)
            if not t0:
                t0 = t
            if t-t0 >= self.timeout:
                break

            #print 'current', current, idle_current, lp_current
            if idle_current and (current > idle_current*1.3 or lp_current > idle_current * 1.1):
                #console('found stall current', current, lp_current)

                voltage, current, t1 = self.average_power(.1)
                #console('settled to stall current', current)

                stall_current = current
            elif self.fault():
                if not idle_current:
                    console('motor fault without finding idle current:', raw_cmd)
                    return False
                console('stall current above max current for raw_cmd:', raw_cmd)
                stall_current = self.data['servo/Max Current']
            elif idle_current and current > self.data['servo/Max Current']:
                console('servo failed to stop overcurrent!!!!', current)
                stall_current = self.data['servo/Max Current']

            if stall_current:
                return [raw_cmd, idle_current, stall_current, idle_voltage, t - t0]

            if not idle_current and count > 1 and current > 0 and prevcurrent and \
               abs(prevcurrent - current) < .03:
                #console('found idle current', current)
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
        return .3*d
    
    def search_end(self, sign):
        self.reset()
        self.waitfault(1)
        console('moving away from end')
        self.reset()
        self.command(self.safe_raw_cmd(-sign))
        self.waitfault(5)
        self.reset()
        self.waitfault(1)
        console('continuing to end at', self.safe_raw_cmd(sign))
        if not self.calibrate_speed(self.safe_raw_cmd(sign)):
            console('failed to reach end at safe speed')
            return False
        console('reached end')
        return True

    def calibrate(self):
        self.brake_hack = False
        
        self.client.set('ap/enabled', False)
        self.client.set('servo/Brake Hack', self.brake_hack)
        self.client.set('servo/Max Current', 10)
        self.client.set('servo/raw_command', 0)

        self.timeout = 40 # max time to move end to end

        if not self.search_end(-1):
            console('failed to reset servo position to start')
            console('Trying with brake hack')
            self.brake_hack = True
            self.client.set('servo/Brake Hack', self.brake_hack)
            if not self.search_end(-1):
                console('failed to reset servo position to start')
                exit(1)

        console('found start')
        calibration = {} # clear old cal
        
        complete = [False, False]
        lastspeed = [0, 0]
        steps = 5 # speeds 20 speeds
        mincmd = 320
        maxcmd = 600
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
                    command, idle_current, stall_current, cal_voltage, dt = cal
                    truespeed = 1/dt
                    print 'truespeed', truespeed, 'lastspeed', lastspeed[signi]
                    if truespeed - lastspeed[signi] < .05/steps:
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
