#!/usr/bin/env python
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time, sys, json
from signalk.client import SignalKClient
from servo import ArduinoServoFlags

def console(*text):
    for t in text:
        sys.stdout.write(str(t))
        sys.stdout.write(' ')
    print ''

class ServoClient(SignalKClient):
    def __init__(self):
        watches = ['servo/voltage', 'servo/current', 'servo/engauged', 'servo/command', 'servo/raw_command', 'servo/Max Current', 'servo/flags']
        def on_con(client):
            for name in watches:
                client.watch(name)
                
        # should only run one instance, for now hardcode local connection
        super(ServoClient, self).__init__(on_con, sys.argv[1])
        self.data = {'servo/engauged': False}
        self.current_total = self.voltage_total = 0, 0

        self.first_command = False
        self.lastcommand = False
        self.lastlastcommand = 0

    def fault(self):
        return not self.data['servo/engauged']

    def process_messages(self):
        data = self.receive()
        for name in data:
            if name == 'servo/command':
                if self.first_command:
                    console('servo command received, aborting')
                    console('ensure no other client is commanding the servo during calibration!!')
                    exit(1)
                self.first_command = True
            elif name == 'servo/raw_command':
                cmd = data[name]['value']
                if cmd != self.lastcommand and cmd != self.lastlastcommand:
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
            self.set('servo/raw_command', self.lastcommand)
        return True

    def command(self, value):
        self.set('servo/raw_command', value)
        self.lastlastcommand = self.lastcommand
        self.lastcommand = value

    def stop(self):
        self.command(0)
        
    def reset(self):
        self.stop()
        if not self.waitnofault(7):
            raise 'servo reset failed'

    def average_power(self, timeout):
        start = time.time()
        self.currenttimestamp = False
        while time.time() - start < timeout or self.current_total[1] == 0:
            self.process_messages()
            if self.fault():
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

    def calibrate_speed(self, speed, timeout):
        self.average_power(0)
        self.reset()
        console('calibration speed:', speed)
        t0 = False

        idle_current = idle_voltage = False
        stall_current = False
        prevcurrent = False
        lp_current = 0
        count = 0
        while True:
            self.command(speed)

            if idle_current:
                avgtime = .2
            else:
                avgtime = .5

            voltage, current, t = self.average_power(avgtime)
            if not t0:
                t0 = t
            if t-t0 >= timeout:
                break

            if idle_current and (current > idle_current*2 or lp_current > idle_current * 1.4):
                console('found stall current', current)

                voltage, current, t1 = self.average_power(.1)
                console('settled to stall current', current)

                stall_current = current
            elif self.fault():
                if not idle_current:
                    console('motor fault without finding idle current:', speed)
                    return False
                console('stall current above max current for speed:', speed)
                stall_current = self.data['servo/Max Current']
            elif idle_current and current > self.data['servo/Max Current']:
                console('servo failed to stop overcurrent!!!!', current)
                stall_current = self.data['servo/Max Current']

            if stall_current:
                return speed, idle_current, stall_current, idle_voltage, t - t0

            if not idle_current and count > 1 and current > 0 and prevcurrent and \
               abs(prevcurrent - current) < .03:
                console('found idle current', current)
                idle_current = current
                idle_voltage = voltage
                
            prevcurrent = current

            lp_current = .9*lp_current + .1*current
            count += 1

        print 'timeout calibrating speed', speed
        return False

    def Calibrate(self):
        self.set('ap/enabled', False)
        self.set('servo/Brake Hack', True)

        self.set('servo/raw_command', 0)
        if not self.waitnofault(7):
            raise 'servo reset failed'
        self.stop()
        self.waitfault(1)

        safe_speed = .15 # starting at relatively slow speed
        timeout = 40 # max time to move end to end

        console('moving away from end')
        self.reset()
    
        self.command(safe_speed)
        self.waitfault(5)
        self.reset()

        console('searching start')
        self.process_messages()

        if not self.calibrate_speed(-safe_speed, timeout):
            console('failed to reset servo position to start')
            console('Trying with brake hack')

#            self.set('servo/Brake Hack', True)
            self.command(safe_speed)
            self.waitfault(.5)
            self.reset()
            if not self.calibrate_speed(-safe_speed, timeout):
                console('failed to reset servo position to start')
                exit(1)
           
        console('found start')
        calibration = {} # clear old cal
        
        steps = 40 # calibrate 20 speeds
        complete = [False, False]
        lastspeed = [0, 0]
        for abs_speed in range(120, 230, 200/steps):
            for signi in [0, 1]:
                sign = 1-2*signi
                speed = sign*abs_speed/1000.0
                if signi == 0:
                    speed += .03
                self.stop()
                console('flags', self.data['servo/flags'], 'fault', self.fault())
                cal = self.calibrate_speed(speed, timeout)
                if cal:
                    command, idle_current, stall_current, cal_voltage, dt = cal
                    truespeed = 1/dt
                    console('speed', speed, 'truespeed', truespeed, 'cal', cal, 'dt', dt)
                    if truespeed - lastspeed[signi] < .1/steps:
                        complete[signi] += 1
                    else:
                        complete[signi] = 0

                    c = map(lambda x : sign*x, calibration)
                    if len(c) == 0 or truespeed > sorted(c)[len(c)-1]:
                        print 'update cal', cal
                        calibration[sign*truespeed] = cal
                    lastspeed[signi] = truespeed
                else:
                    console('speed', speed, ' failed')
                    continue
                    console('speed', speed, ' failed, continuing to end at speed', sign*safe_speed)
                    self.stop()
                    self.command(safe_speed*sign)
                    if not self.calibrate_speed(sign*safe_speed, timeout):
                        raise 'failed to reach end'
                    console('reached end')

            if complete[0] > 3 and complete[1] > 3:
                break

        self.stop()
        self.set('servo/raw_command', False)
        calibration[0] = 0, 0, 0, 12, 0
        return calibration

if __name__ == '__main__':
    cal = ServoClient().Calibrate()
#    file = open('cal_raspberry')
#    cal = json.loads(file.readline())

    f = open('servo_calibration', 'w')
    f.write(json.dumps(cal))
    console('calibration complete')
