#!/usr/bin/env python
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import os, math
import time, json

from signalk.server import SignalKServer
from signalk.values import *
import autopilot
import select
import serial

import fcntl
# these are not defined in python module
TIOCEXCL = 0x540C
TIOCNXCL = 0x540D

def sign(x):
    if x > 0:
        return 1
    if x < 0:
        return -1
    return 0

def interpolate(x, x0, x1, y0, y1):
    d = (x-x0)/(x1-x0)
    return (1-x) * y0 + d * y1

# the raspberry hw pwm servo driver uses
# the pwm0 output from the raspberry pi directly to
# a servo or motor controller
# there is no current feedback, instead a fault pin is used
class RaspberryHWPWMServoDriver:
    def __init__(self):
        import wiringpi
        wiringpi.wiringPiSetup()
        self.engauged = False

    def command(self, command, max_current):
        if command == 0:
            stop()
            return

        if not self.engauged:
            wiringpi.pinMode(1, wiringpi.GPIO.PWM_OUTPUT)
            wiringpi.pwmSetMode( wiringpi.GPIO.PWM_MODE_MS )

            # fix this to make it higher resolution!!!
            wiringpi.pwmSetRange( 1000 )
            wiringpi.pwmSetClock( 400 )
            self.engauged = True
            
        clockcmd = 60 + 30*command
        clockcmd = int(min(110, max(36, clockcmd)))
        wiringpi.pwmWrite(1, clockcmd)

    def stop():
        wiringpi.pinMode(1, wiringpi.GPIO.PWM_INPUT)
        self.engauged = False
        
    def fault(self):
        return wiringpi.digitalRead(self.fault_pin)

    def errorpin_interrupt(self):
        if self.fault():
            self.stop()


# the arduino pypilot servo sketch is used and communication
# over serial port controllers the servo motor controller
# as well as voltage and current feedback
class ServoFlags(Value):
    SYNC = 1
    FAULTPIN = 2
    OVERCURRENT = 4
    ENGAUGED = 8
    
    def __init__(self, name):
        super(ServoFlags, self).__init__(name, 0)
            
    def strvalue(self):
        ret = ''
        if self.value & self.SYNC:
            ret += 'SYNC '
        if self.value & self.FAULTPIN:
            ret += 'FAULTPIN '
        if self.value & self.OVERCURRENT:
            ret += 'OVERCURRENT '
        if self.value & self.ENGAUGED:
            ret += 'ENGAUGED '
        return ret
        
    def get_signalk(self):
        return '{"' + self.name + '": {"value": "' + self.strvalue() + '"}}'

class ServoTelemetry:
    CURRENT = 1
    VOLTAGE = 2
    TEMPERATURE = 4
    SPEED = 8
    POSITION = 16
    FLAGS = 32


# special case for servo calibration, and to convert the map key
class CalibrationProperty(Property):
    def __init__(self, name, initial):
        super(CalibrationProperty, self).__init__(name, initial)

    def set(self, value):
        nvalue = {}
        for cal in value:
            nvalue[round(1000*float(cal))/1000.0] = map(lambda x : round(1000*x)/1000.0, value[cal])
        if not 0 in nvalue: #remove?
            nvalue[0] = 0, 0, 0, 12, 0

        return super(CalibrationProperty, self).set(nvalue)

    def get_signalk(self):
        strkey = {}
        for key in self.value:
            strkey[str(key)] = self.value[key]
        return '{"' + self.name + '": {"value": ' + json.dumps(strkey) + '}}'


# a property which records the time when it is updated
class TimedProperty(Property):
    def __init__(self, name, initial):
        super(TimedProperty, self).__init__(name, initial)
        self.time = 0

    def set(self, value):
        self.time = time.time()
        return super(TimedProperty, self).set(value)
    
class Servo:
    calibration_filename = autopilot.pypilot_dir + 'servocalibration'

    def __init__(self, server, serialprobe):
        self.server = server
        self.serialprobe = serialprobe
        self.fwd_fault = self.rev_fault = False
        self.min_speed = self.Register(RangeProperty, 'Min Speed', .3, 0, 1, persistent=True)
        self.max_speed = self.Register(RangeProperty, 'Max Speed', 1, 0, 1, persistent=True)
        self.brake_hack = self.Register(BooleanProperty, 'Brake Hack', True, persistent=True)
        self.brake_hack_state = 0

        # power usage
        self.command = self.Register(TimedProperty, 'command', 0)
        self.rawcommand = self.Register(TimedProperty, 'raw_command', 0)
        self.timestamp = time.time()
        timestamp = server.TimeStamp('servo')
        self.voltage = self.Register(SensorValue, 'voltage', timestamp)
        self.current = self.Register(SensorValue, 'current', timestamp)
        self.engauged = self.Register(BooleanValue, 'engauged', False)
        self.max_current = self.Register(RangeProperty, 'Max Current', 2, 0, 10, persistent=True)
        self.slow_period = self.Register(RangeProperty, 'Slow Period', 1.5, .1, 10, persistent=True)
        self.compensate_current = self.Register(BooleanProperty, 'Compensate Current', False, persistent=True)
        self.compensate_voltage = self.Register(BooleanProperty, 'Compensate Voltage', False, persistent=True)
        self.amphours = self.Register(ResettableValue, 'Amp Hours', 0, persistent=True, persistent_timeout=300)
        self.watts = self.Register(RoundedValue, 'Watts', 0)

        self.calibration = self.Register(CalibrationProperty, 'calibration', {})
        self.load_calibration()

        self.position = .5
        self.speed = 0
        self.lastpositiontime = time.time()
        self.lastpositionamphours = 0

        self.last_zero_command_time = self.command_timeout = time.time()

        self.mode = self.Register(StringValue, 'mode', 'none')
        self.controller = self.Register(StringValue, 'controller', 'none')
        self.flags = self.Register(ServoFlags, 'flags')

        self.driver = False

    def Register(self, _type, name, *args, **kwargs):
        return self.server.Register(_type(*(['servo/' + name] + list(args)), **kwargs))

    def send_command(self):
        timeout = 1 # command will expire after 1 second
        if self.rawcommand.value:
            if time.time() - self.rawcommand.time > timeout:
                self.rawcommand.set(0)
            else:
                self.raw_command(self.rawcommand.value)
        else:
            if time.time() - self.command.time > timeout:
                command = 0
            else:
                command = self.command.value
            self.velocity_command(command)

    def velocity_command(self, speed):
        if speed == 0: # optimization
            self.raw_command(0)
            return

        # complete integration from previous step
        t = time.time()
        dt = t - self.lastpositiontime
        self.lastpositiontime = t
        #        print 'integrate pos', self.position, self.speed, speed, dt

        self.position += self.speed * dt
        self.position = min(max(self.position, 0), 1)

        # get current
        ampseconds = 3600*(self.amphours.value - self.lastpositionamphours)
        current = ampseconds / dt
        self.lastpositionamphours = self.amphours.value

        speed = max(min(speed, self.max_speed.value),-self.max_speed.value)

        # apply calibration
        cal0 = cal1 = False
        for calspeed in sorted(self.calibration.value):
            if calspeed != 0 and abs(calspeed) < self.min_speed.value:
                continue
            
            cal = self.calibration.value[calspeed]
            command, idle_current, stall_current, cal_voltage, dt = cal
            if self.compensate_current.value:
                #1 = m*idle_current + b
                #0 = m*stall_current + b

                if idle_current  - stall_current == 0:
                    factor = 1
                else:
                    m = 1/(idle_current  - stall_current)
                    b = -m*stall_current
                    factor = m*self.current.value + b
                    print "factor", factor, idle_current, self.current.value, calspeed
                calspeed *= factor

            if self.compensate_voltage.value:
                calspeed *= cal_voltage / self.voltage.value

#            print 'speed', speed, calspeed
            if speed < calspeed:
                calspeed1 = calspeed
                cal1 = cal
                break

            calspeed0 = calspeed
            cal0 = cal

        duty = 1
        if not cal0: # minimum calibrated speed
            cal = cal1
            speed = calspeed1
        elif not cal1: # maximum calibrated speed
            cal = cal0
            speed = calspeed0
        else:
            if False: # interpolate
#            print 'speed', cal0, cal1, speed, calspeed0, calspeed1
                cal = map(lambda x, y :
                          interpolate(speed, calspeed0, calspeed1, x, y), cal0, cal1)
            else:
                duty = (speed - calspeed0)/(calspeed1 - calspeed0)
                slow_period = self.slow_period.value

                # move at least 1/2th of a second
                minduty = .35 / slow_period

                # double slow period until we get a duty so the motor will run
                # at least 1/3rd of a second
                if duty > 0 and duty < 1:
                    while duty < minduty or duty > 1-minduty:
                        slow_period *= 2
                        minduty /= 2
                #print 'duty', minduty, duty, slow_period, calspeed0, calspeed1

                if (time.time() % slow_period) / slow_period > duty:
                    speed = calspeed0
                    cal = cal0
                else:
                    speed = calspeed1
                    cal = cal1
#            print 'duty', duty, speed, (time.time() % self.slow_period.value) / self.slow_period.value
        command, idle_current, stall_current, voltage, dt = cal

#        max_current = stall_current
        max_current = self.max_current.value
        if self.compensate_voltage.value:
            max_current *= self.voltage.value/voltage

        if self.fault():
            #print 'fault, should stop and reset', self.fault(), self.current.value, current, max_current, idle_current, stall_current
            if self.speed > 0:
                self.fwd_fault = True
                self.position = 1
            elif self.speed < 0:
                self.rev_fault = True
                self.position = -1

            self.stop()
            return

        if self.position < .9:
            self.fwd_fault = False
        if self.position > .1:
            self.rev_fault = False

        if self.fwd_fault and command > 0 or \
           self.rev_fault and command < 0:
            self.raw_command(0)
            return # abort

        self.speed = speed
        self.raw_command(command)

    def raw_command(self, command):
        if self.brake_hack_state == 1:
            if self.fault():
                return

            if self.driver:
                self.driver.command(0)
            self.mode.update('brake')
            self.brake_hack_state = 0
            return

        if command <= 0:
            if self.brake_hack.value and self.mode.value == 'forward':
                if not self.fault():
                    if self.driver:
                        self.driver.stop()
                        self.driver.command(-.18)
                    self.brake_hack_state = 1
                return
            if command < 0:
                if self.mode != 'reverse':
                    self.mode.update('reverse')
            else:
#                if self.mode.value == 'idle':
#                    return
                self.mode.update('idle')
        else:
            self.mode.update('forward')

        if not self.driver:
            t0 = time.time()
            device_path = self.serialprobe.probe('servo', [115200])
            if device_path:
                #from arduino_servo.arduino_servo_python import ArduinoServo
                from arduino_servo.arduino_servo import ArduinoServo
                device = serial.Serial(*device_path)
                device.timeout=0 #nonblocking
                fcntl.ioctl(device.fileno(), TIOCEXCL) #exclusive
                self.driver = ArduinoServo(device.fileno())

                t0 = time.time()
                if self.driver.initialize():
                    self.device = device
                    print 'arduino servo found on', device_path
                    self.serialprobe.probe_success('servo')
                    self.driver.max_current(self.max_current.value)
                    self.controller.set('arduino')

                    if self.brake_hack.value:
                        self.driver.command(-.2) # flush any brake
                    self.driver.command(0)
                    self.lastpolltime = time.time()
                else:
                    print 'failed in ', time.time()-t0
                    device.close()
                    self.driver = False
                    print 'failed to initialize servo on', device


        # only send at .5 seconds when command is zero
        t = time.time()
        if command == 0:
            if t > self.command_timeout and t - self.last_zero_command_time < .5:
                return
            self.last_zero_command_time = t
        else:
            self.command_timeout = t+1
                
        if self.driver:
            self.driver.max_current(self.max_current.value)
            self.driver.command(command)

    def stop(self):
        if self.driver:
            self.driver.stop()
         
        if self.brake_hack.value and self.mode.value == 'forward':
            if self.driver:
                self.driver.command(-.18)
            self.brake_hack_state = 1

        self.mode.set('stop')
        self.speed = 0

    def close_driver(self):
        print 'servo lost connection'
        self.controller.set('none')
        self.device.close()
        self.driver = False


    def poll(self):
        if not self.driver:
            return
        result = self.driver.poll()
        if result == -1:
            print 'poll -1'
            self.close_driver()
            return

        if result == 0:
            d = time.time() - self.lastpolltime
            #print 'i', i, d
            if d > 10: # correct for clock skew
                self.lastpolltime = time.time()
            elif d > 8:
                print 'd', d
                self.close_driver()
            return
        self.lastpolltime = time.time()

        if self.fault():
            if self.speed > 0:
                self.fwd_fault = True
            elif self.speed < 0:
                self.rev_fault = True

        self.timestamp = time.time()
        self.server.TimeStamp('servo', self.timestamp)
        lasttimestamp = self.timestamp
        if result & ServoTelemetry.VOLTAGE:
            self.voltage.set(self.driver.voltage)
        if result & ServoTelemetry.CURRENT:
            self.current.set(self.driver.current)
            # integrate power consumption
            dt = (self.timestamp-lasttimestamp)
            amphours = self.current.value*dt/3600
            self.amphours.set(self.amphours.value + amphours)
            lp = .003
            self.watts.set((1-lp)*self.watts.value + lp*self.voltage.value*self.current.value)
        if result & ServoTelemetry.FLAGS:
            self.flags.update(self.driver.flags)
            self.engauged.update(not not self.driver.flags & ServoFlags.ENGAUGED)

    def fault(self):
        if not self.driver:
            return False
        return self.driver.fault()

    def load_calibration(self):
        try:
            filename = Servo.calibration_filename
            print 'loading servo calibration', filename
            file = open(filename)
            self.calibration.set(json.loads(file.readline()))
        except:
            print 'no servo calibration!!'

    def save_calibration(self):
        file = open(Servo.calibration_filename, 'w')
        file.write(json.dumps(self.calibration))


if __name__ == '__main__':
    print 'Servo Server'
    server = SignalKServer()
    servo = Servo(server)

    while True:
        servo.send_command()
        servo.poll()
        print servo.voltage.value
        server.HandleRequests(.05)
