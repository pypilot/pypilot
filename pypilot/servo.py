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
from servo_calibration import *

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
class RaspberryHWPWMServoDriver(object):
    def __init__(self):
        import wiringpi
        wiringpi.wiringPiSetup()
        self.engauged = False

    def raw_command(self, command):
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
    OVERTEMP = 2
    OVERCURRENT = 4
    ENGAUGED = 8

    INVALID=16*1
    FAULTPIN=16*2
    
    def __init__(self, name):
        super(ServoFlags, self).__init__(name, 0)
            
    def strvalue(self):
        ret = ''
        if self.value & self.SYNC:
            ret += 'SYNC '
        if self.value & self.OVERTEMP:
            ret += 'OVERTEMP '
        if self.value & self.OVERCURRENT:
            ret += 'OVERCURRENT '
        if self.value & self.ENGAUGED:
            ret += 'ENGAUGED '
        if self.value & self.INVALID:
            ret += 'INVALID '
        if self.value & self.FAULTPIN:
            ret += 'FAULTPIN '
        return ret
        
    def get_signalk(self):
        return '{"' + self.name + '": {"value": "' + self.strvalue() + '"}}'

class ServoTelemetry(object):
    FLAGS = 1
    CURRENT = 2
    VOLTAGE = 4
    SPEED = 8
    POSITION = 16
    CONTROLLER_TEMP = 32

# a property which records the time when it is updated
class TimedProperty(Property):
    def __init__(self, name, initial):
        super(TimedProperty, self).__init__(name, initial)
        self.time = 0

    def set(self, value):
        self.time = time.time()
        return super(TimedProperty, self).set(value)
    
class Servo(object):
    calibration_filename = autopilot.pypilot_dir + 'servocalibration'

    def __init__(self, server, serialprobe):
        self.server = server
        self.serialprobe = serialprobe
        self.fwd_fault = self.rev_fault = False

        self.servo_calibration = ServoCalibration(self)
        self.calibration = self.Register(JSONValue, 'calibration', {})
        self.load_calibration()

        self.min_speed = self.Register(RangeProperty, 'min_speed', .5, 0, 1, persistent=True)
        self.max_speed = self.Register(RangeProperty, 'max_speed', 1, 0, 1, persistent=True)
        brake_hack = 'brake_hack' in self.calibration.value and self.calibration.value['brake_hack']
        self.brake_hack = self.Register(BooleanProperty, 'brake_hack', brake_hack, persistent=True)
        self.brake_hack_state = 0

        self.faults = self.Register(Property, 'faults', 0)

        # power usage
        self.command = self.Register(TimedProperty, 'command', 0)
        self.timestamp = time.time()
        timestamp = server.TimeStamp('servo')
        self.voltage = self.Register(SensorValue, 'voltage', timestamp)
        self.current = self.Register(SensorValue, 'current', timestamp)
        self.temperature = self.Register(SensorValue, 'temperature', timestamp)
        self.engauged = self.Register(BooleanValue, 'engauged', False)
        self.max_current = self.Register(RangeProperty, 'max_current', 2, 0, 10, persistent=True)
        self.max_controller_temp = self.Register(RangeProperty, 'max_controller_temp', 65, 30, 80, persistent=True)
        self.period = self.Register(RangeProperty, 'period', .7, .1, 3, persistent=True)
        self.compensate_current = self.Register(BooleanProperty, 'compensate_current', False, persistent=True)
        self.compensate_voltage = self.Register(BooleanProperty, 'compensate_voltage', False, persistent=True)
        self.amphours = self.Register(ResettableValue, 'amp_hours', 0, persistent=True)
        self.watts = self.Register(SensorValue, 'watts', timestamp)

        self.position = .5
        self.speed = 0
        self.lastpositiontime = time.time()
        self.lastpositionamphours = 0

        self.windup = 0
        self.windup_change = 0

        self.disengauged = True
        self.disengauge_on_timeout = self.Register(BooleanValue, 'disengauge_on_timeout', False)
        self.force_engauged = False

        self.last_zero_command_time = self.command_timeout = time.time()

        self.mode = self.Register(StringValue, 'mode', 'none')
        self.controller = self.Register(StringValue, 'controller', 'none')
        self.flags = self.Register(ServoFlags, 'flags')

        self.driver = False
        self.raw_command(0)

    def Register(self, _type, name, *args, **kwargs):
        return self.server.Register(_type(*(['servo/' + name] + list(args)), **kwargs))

    def send_command(self):
        t = time.time()

        def engauge():
            if self.disengauged:
                self.disengauged = False
                
        if not self.disengauge_on_timeout.value:
            engauge()

        if self.servo_calibration.thread.is_alive():
            return

        timeout = 1 # command will expire after 1 second
        if self.command.value:
            if time.time() - self.command.time > timeout:
                self.disengauged = True
                self.command.update(0)
            else:
                engauge()
                self.velocity_command(self.command.value)
        else:
            #print 'timeout', t - self.command_timeout
            if self.disengauge_on_timeout.value and \
               not self.servo.force_engauged and \
               t - self.command_timeout > self.period.value*3:
                self.disengauged = True
            self.raw_command(0)

    def velocity_command(self, speed):
        # complete integration from previous step
        t = time.time()
        dt = t - self.lastpositiontime
        self.lastpositiontime = t

        if speed == 0 and self.speed == 0: # optimization
            self.raw_command(0)
            return

        if self.fwd_fault and speed > 0 or \
           self.rev_fault and speed < 0:
            self.speed = 0
            self.raw_command(0)
            return # abort

        #        print 'integrate pos', self.position, self.speed, speed, dt

        self.position += self.speed * dt
        self.position = min(max(self.position, 0), 1)
        if self.position < .9:
            self.fwd_fault = False
        if self.position > .1:
            self.rev_fault = False

        if False: # don't keep moving too long in same direction.....
            rng = 5;
            if self.position > 1 + rng:
                self.fwd_fault = True
            if self.position < -rng:
                self.rev_fault = True
            
        if self.compensate_voltage.value:
            speed *= 12 / self.voltage.value

        if self.compensate_current.value:
            # get current
            ampseconds = 3600*(self.amphours.value - self.lastpositionamphours)
            current = ampseconds / dt
            self.lastpositionamphours = self.amphours.value
            pass #todo fix this
        # allow higher current with higher voltage???
        #max_current = self.max_current.value
        #if self.compensate_voltage.value:
        #    max_current *= self.voltage.value/voltage
        
        min_speed = self.min_speed.value
        
        # ensure max_speed is at least min_speed
        #if min_speed > self.max_speed.value:
        #    self.max_speed.set(min_speed)

        # integrate windup
        self.windup += (speed - self.speed) * dt

        # if windup overflows, move at minimum speed
        if abs(self.windup) > self.period.value*min_speed / 1.5:
            if abs(speed) < min_speed:
                speed = min_speed if self.windup > 0 else -min_speed
        else:
            speed = 0
            
        # don't let windup overflow
        self.windup = min(max(self.windup, -self.period.value), self.period.value)
        #print 'windup', self.windup, dt, self.windup / dt, speed, self.speed

        if speed * self.speed <= 0: # switched direction or stopped?
            if t - self.windup_change < self.period.value:
                # less than period, keep previous direction, but use minimum speed
                if self.speed > 0:
                    speed = min_speed
                elif self.speed < 0:
                    speed = -min_speed
                else:
                    speed = 0
            else:
                self.windup_change = t

        # clamp to max speed
        speed = min(max(speed, -self.max_speed.value), self.max_speed.value)
        if True:
            self.speed = speed
        else:
            # estimate true speed from voltage, current, and last command
            # TODO
            pass

        if speed > 0:
            cal = self.calibration.value['forward']
        elif speed < 0:
            cal = self.calibration.value['reverse']
        else:
            self.raw_command(0)
            return

        raw_speed = cal[0] + abs(speed)*cal[1]
        if speed < 0:
            raw_speed = - raw_speed
        command = raw_speed

        self.raw_command(command)

    def raw_command(self, command):
        if self.fault():
            if self.speed > 0:
                self.fwd_fault = True
                self.position = 1
            elif self.speed < 0:
                self.rev_fault = True
                self.position = -1

            self.stop()
            return
        
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
            device_path = self.serialprobe.probe('servo', [38400], 1)
            if device_path:
                #from arduino_servo.arduino_servo_python import ArduinoServo
                from arduino_servo.arduino_servo import ArduinoServo
                device = serial.Serial(*device_path)
                device.timeout=0 #nonblocking
                fcntl.ioctl(device.fileno(), TIOCEXCL) #exclusive
                self.driver = ArduinoServo(device.fileno())
                self.driver.max_values(self.max_current.value, self.max_controller_temp.value)

                t0 = time.time()
                if self.driver.initialize(device_path[1]):
                    self.device = device
                    print 'arduino servo found on', device_path
                    self.serialprobe.probe_success('servo')
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
            #if t > self.command_timeout and t - self.last_zero_command_time < .5:
            #    return
            self.last_zero_command_time = t
        else:
            self.command_timeout = t
                
        if self.driver:
            if self.disengauged: # keep sending disengauge to keep sync
                self.driver.disengauge()
            else:
                max_current = self.max_current.value
                if self.fwd_fault or self.rev_fault: # allow more current to "unstuck" ram
                    max_current *= 2
                self.driver.max_values(max_current, self.max_controller_temp.value)
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
        #print 'servo poll'
        result = self.driver.poll()
        if result == -1:
            print 'servo poll -1'
            self.close_driver()
            return

        if result == 0:
            d = time.time() - self.lastpolltime
            if d > 10: # correct for clock skew
                self.lastpolltime = time.time()
            elif d > 8:
                print 'servo timeout', d
                self.close_driver()
            return
        self.lastpolltime = time.time()

        if self.fault():
            if not self.fwd_fault and not self.rev_fault:
                self.faults.set(self.faults.value + 1)
            
            if self.speed > 0:
                self.fwd_fault = True
            elif self.speed < 0:
                self.rev_fault = True

        lasttimestamp = self.timestamp
        self.timestamp = time.time()
        self.server.TimeStamp('servo', self.timestamp)
        if result & ServoTelemetry.VOLTAGE:
            self.voltage.set(self.driver.voltage)
        if result & ServoTelemetry.CONTROLLER_TEMP:
            self.temperature.set(self.driver.controller_temp)
        if result & ServoTelemetry.CURRENT:
            self.current.set(self.driver.current)
            # integrate power consumption
            dt = (self.timestamp-lasttimestamp)
            amphours = self.current.value*dt/3600
            self.amphours.set(self.amphours.value + amphours)
            lp = .003*dt # 5 minute time constant to average wattage
            self.watts.set((1-lp)*self.watts.value + lp*self.voltage.value*self.current.value)
        if result & ServoTelemetry.FLAGS:
            self.flags.update(self.driver.flags)
            self.engauged.update(not not self.driver.flags & ServoFlags.ENGAUGED)
        self.servo_calibration.poll()

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
            print 'WARNING: using default servo calibration!!'
            self.calibration.set({'forward': [.2, .6], 'reverse': [.2, .6]})

    def save_calibration(self):
        file = open(Servo.calibration_filename, 'w')
        file.write(json.dumps(self.calibration))


if __name__ == '__main__':
    import serialprobe
    print 'Servo Server'
    server = SignalKServer()
    serial_probe = serialprobe.SerialProbe()
    servo = Servo(server, serial_probe)

    while True:
        servo.poll()
        servo.send_command()
        if servo.voltage.value:
            print 'voltage:', servo.voltage.value, 'current', servo.current.value, 'temp', servo.temperature.value
        server.HandleRequests()
        time.sleep(.1)

