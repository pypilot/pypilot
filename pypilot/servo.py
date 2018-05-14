#!/usr/bin/env python
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import os, math, sys
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
    FWD_FAULTPIN=16*2
    REV_FAULTPIN=16*4

    MIN_RUDDER=256*1
    MAX_RUDDER=256*2    

    DRIVER_MASK = 2047 # bits used for driver flags

    FWD_FAULT=2048*1 # overcurrent faults
    REV_FAULT=2048*2
    DRIVER_TIMEOUT = 2048*4

    def __init__(self, name):
        super(ServoFlags, self).__init__(name, 0)

    def updatedriver(self, driverflags):
        self.update(self.value & ~self.DRIVER_MASK | driverflags)
            
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
        if self.value & self.FWD_FAULTPIN:
            ret += 'FWD_FAULTPIN '
        if self.value & self.REV_FAULTPIN:
            ret += 'REV_FAULTPIN '
        if self.value & self.MIN_RUDDER:
            ret += 'MIN_RUDDER '
        if self.value & self.MAX_RUDDER:
            ret += 'MAX_RUDDER '
        if self.value & self.FWD_FAULT:
            ret += 'FWD_FAULT '
        if self.value & self.REV_FAULT:
            ret += 'REV_FAULT '
        if self.value & self.DRIVER_TIMEOUT:
            ret += 'DRIVER_TIMEOUT '
        return ret

    def setbit(self, bit, t=True):
        if t:
            self.update(self.value | bit)
        else:
            self.update(self.value & ~bit)

    def clearbit(self, bit):
        self.setbit(bit, False)
            
    def fwd_fault(self):
        self.update((self.value | ServoFlags.FWD_FAULT) \
                    & ~ServoFlags.REV_FAULT)

    def rev_fault(self):
        self.update((self.value | ServoFlags.REV_FAULT) \
                    & ~ServoFlags.FWD_FAULT)
        
    def get_signalk(self):
        return '{"' + self.name + '": {"value": "' + self.strvalue() + '"}}'

class ServoTelemetry(object):
    FLAGS = 1
    CURRENT = 2
    VOLTAGE = 4
    SPEED = 8
    POSITION = 16
    CONTROLLER_TEMP = 32
    MOTOR_TEMP = 64
    RUDDER_POS = 128

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
        self.lastdir = 0 # doesn't matter

        self.servo_calibration = ServoCalibration(self)
        self.calibration = self.Register(JSONValue, 'calibration', {})
        self.load_calibration()

        self.min_speed = self.Register(RangeProperty, 'min_speed', 1, 0, 1, persistent=True)
        self.max_speed = self.Register(RangeProperty, 'max_speed', 1, 0, 1, persistent=True)

        self.faults = self.Register(Property, 'faults', 0)

        # power usage
        self.command = self.Register(TimedProperty, 'command', 0)
        self.current_timestamp = time.time()
        timestamp = server.TimeStamp('servo')
        self.voltage = self.Register(SensorValue, 'voltage', timestamp)
        self.current = self.Register(SensorValue, 'current', timestamp)
        self.controller_temp = self.Register(SensorValue, 'controller_temp', timestamp)
        self.motor_temp = self.Register(SensorValue, 'motor_temp', timestamp)
        self.rudder_pos = self.Register(SensorValue, 'rudder_pos', timestamp)
        
        self.engauged = self.Register(BooleanValue, 'engauged', False)
        self.max_current = self.Register(RangeProperty, 'max_current', 2, 0, 20, persistent=True)
        self.max_controller_temp = self.Register(RangeProperty, 'max_controller_temp', 60, 45, 100, persistent=True)
        self.max_motor_temp = self.Register(RangeProperty, 'max_motor_temp', 55, 30, 100, persistent=True)
        self.min_rudder_pos = self.Register(RangeProperty, 'min_rudder_pos', -100, -100, 100, persistent=True)
        self.max_rudder_pos = self.Register(RangeProperty, 'max_rudder_pos',  100, -100, 100, persistent=True)
        self.max_slew_speed = self.Register(RangeProperty, 'max_slew_speed', 30, 0, 100, persistent=True)
        self.max_slew_slow = self.Register(RangeProperty, 'max_slew_slow', 50, 0, 100, persistent=True)
        self.period = self.Register(RangeProperty, 'period', .7, .1, 3, persistent=True)
        self.compensate_current = self.Register(BooleanProperty, 'compensate_current', False, persistent=True)
        self.compensate_voltage = self.Register(BooleanProperty, 'compensate_voltage', False, persistent=True)
        self.amphours = self.Register(ResettableValue, 'amp_hours', 0, persistent=True)
        self.watts = self.Register(SensorValue, 'watts', timestamp)

        self.speed = self.Register(SensorValue, 'speed', timestamp)
        self.speed.set(0)

        self.position = self.Register(SensorValue, 'position', timestamp)
        self.position.set(.5)

        self.rawcommand = self.Register(SensorValue, 'raw_command', timestamp)

        self.lastpositiontime = time.time()
        self.lastpositionamphours = 0

        self.windup = 0
        self.windup_change = 0

        self.disengauged = True
        self.disengauge_on_timeout = self.Register(BooleanValue, 'disengauge_on_timeout', True, persistent=True)
        self.force_engauged = False

        self.last_zero_command_time = self.command_timeout = time.time()
        self.driver_timeout_start = 0

        self.mode = self.Register(StringValue, 'mode', 'none')
        self.controller = self.Register(StringValue, 'controller', 'none')
        self.flags = self.Register(ServoFlags, 'flags')

        self.driver = False
        self.raw_command(0)

    def Register(self, _type, name, *args, **kwargs):
        return self.server.Register(_type(*(['servo.' + name] + list(args)), **kwargs))

    def send_command(self):
        t = time.time()
                
        if not self.disengauge_on_timeout.value:
            self.disengauged = False

        if self.servo_calibration.thread.is_alive():
            print 'cal thread'
            return

        timeout = 1 # command will expire after 1 second
        if self.command.value and not self.fault():
            if time.time() - self.command.time > timeout:
                print 'servo command timeout', time.time() - self.command.time
                self.command.set(0)
            self.disengauged = False
            self.velocity_command(self.command.value)
        else:
            #print 'timeout', t - self.command_timeout
            if self.disengauge_on_timeout.value and \
               not self.force_engauged and \
               t - self.command_timeout > self.period.value*3:
                self.disengauged = True
            self.speed.set(0)
            self.raw_command(0)

    def velocity_command(self, speed):
        # complete integration from previous step
        t = time.time()
        dt = t - self.lastpositiontime
        self.lastpositiontime = t

        if speed == 0 and self.speed.value == 0: # optimization
            self.raw_command(0)
            return

        if self.flags.value & ServoFlags.FWD_FAULT and speed > 0 or \
           self.flags.value & ServoFlags.REV_FAULT and speed < 0:
            self.speed.set(0)
            self.raw_command(0)
            return # abort

        position = self.position.value + self.speed.value * dt / 10 # remove when speed is correct
        self.position.set(min(max(position, 0), 1))
        #print 'integrate pos', self.position, self.speed, speed, dt, self.fwd_fault, self.rev_fault
        if self.position.value < .9:
            self.flags.clearbit(ServoFlags.FWD_FAULT)
        if self.position.value > .1:
            self.flags.clearbit(ServoFlags.REV_FAULT)

        if False: # don't keep moving really long in same direction.....
            rng = 5;
            if self.position.value > 1 + rng:
                self.flags.fwd_fault()
            if self.position.value < -rng:
                self.flags.rev_fault()
            
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
        if min_speed > self.max_speed.value:
            self.max_speed.set(min_speed)

        # integrate windup
        self.windup += (speed - self.speed.value) * dt

        # if windup overflows, move at minimum speed
        if abs(self.windup) > self.period.value*min_speed / 1.5:
            if abs(speed) < min_speed:
                speed = min_speed if self.windup > 0 else -min_speed
        else:
            speed = 0

        # don't let windup overflow
        self.windup = min(max(self.windup, -1.5*self.period.value), 1.5*self.period.value)
        #print 'windup', self.windup, dt, self.windup / dt, speed, self.speed
            
        if speed * self.speed.value <= 0: # switched direction or stopped?
            if t - self.windup_change < self.period.value:
                # less than period, keep previous direction, but use minimum speed
                if self.speed.value > 0:
                    speed = min_speed
                elif self.speed.value < 0:
                    speed = -min_speed
                else:
                    speed = 0
            else:
                self.windup_change = t

        # clamp to max speed
        speed = min(max(speed, -self.max_speed.value), self.max_speed.value)
        if True:
            self.speed.set(speed)
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
            raw_speed = -raw_speed
        command = raw_speed

        self.raw_command(command)

    def raw_command(self, command):
        self.rawcommand.set(command)
        if command <= 0:
            if command < 0:
                self.mode.update('reverse')
                self.lastdir = -1
            else:
                self.mode.update('idle')
        else:
            self.mode.update('forward')
            self.lastdir = 1

        t = time.time()
        if command == 0:
            # only send at .5 seconds when command is zero
            if t > self.command_timeout and t - self.last_zero_command_time < .5:
                return
            self.last_zero_command_time = t
        else:
            self.command_timeout = t

        if self.driver:
            if self.disengauged: # keep sending disengauge to keep sync
                self.driver.disengauge()
            else:
                max_current = self.max_current.value
                if self.flags.value & ServoFlags.FWD_FAULT or \
                   self.flags.value & ServoFlags.REV_FAULT: # allow more current to "unstuck" ram
                    max_current *= 2
                self.send_driver_max_values(max_current)
                self.driver.command(command)

                # detect driver timeout if commanded without measuring current
                if self.current.value:
                    self.flags.clearbit(ServoFlags.DRIVER_TIMEOUT)
                    self.driver_timeout_start = 0
                elif command:
                    if self.driver_timeout_start:
                        if time.time() - self.driver_timeout_start > 1:
                            self.flags.setbit(ServoFlags.DRIVER_TIMEOUT)
                    else:
                        self.driver_timeout_start = time.time()

    def reset(self):
        if self.driver:
            self.driver.reset()

    def close_driver(self):
        print 'servo lost connection'
        self.controller.set('none')
        self.device.close()
        self.driver = False

    def send_driver_max_values(self, max_current):
        self.driver.max_values(max_current, self.max_controller_temp.value, self.max_motor_temp.value, self.min_rudder_pos.value, self.max_rudder_pos.value, self.max_slew_speed.value, self.max_slew_slow.value)

    def poll(self):
        if not self.driver:
            device_path = self.serialprobe.probe('servo', [38400], 1)
            if device_path:
                #from arduino_servo.arduino_servo_python import ArduinoServo
                from arduino_servo.arduino_servo import ArduinoServo
                device = serial.Serial(*device_path)
                device.timeout=0 #nonblocking
                fcntl.ioctl(device.fileno(), TIOCEXCL) #exclusive
                self.driver = ArduinoServo(device.fileno())
                self.send_driver_max_values(self.max_current.value)

                t0 = time.time()
                if self.driver.initialize(device_path[1]):
                    self.device = device
                    print 'arduino servo found on', device_path
                    self.serialprobe.probe_success('servo')
                    self.controller.set('arduino')

                    self.driver.command(0)
                    self.lastpolltime = time.time()
                else:
                    print 'failed in ', time.time()-t0
                    device.close()
                    self.driver = False
                    print 'failed to initialize servo on', device

        if not self.driver:
            return
        self.servo_calibration.poll()
        result = self.driver.poll()
        #print 'servo poll', result

        if result == -1:
            print 'servo poll -1'
            self.close_driver()
            return

        if result == 0:
            d = time.time() - self.lastpolltime
            if d > 5: # correct for clock skew
                self.lastpolltime = time.time()
            elif d > 4:
                print 'servo timeout', d
                self.close_driver()
        else:
            self.lastpolltime = time.time()

        if self.fault():
            if not self.flags.value & ServoFlags.FWD_FAULT and \
               not self.flags.value & ServoFlags.REV_FAULT:
                self.faults.set(self.faults.value + 1)
            
            # if overcurrent then fault in the direction traveled
            # this prevents moving further in this direction
            if self.flags.value & ServoFlags.OVERCURRENT:
                if self.lastdir > 0:
                    self.flags.fwd_fault()
                    self.position.set(1)
                elif self.lastdir < 0:
                    self.flags.rev_fault()
                    self.position.set(-1)

            self.reset() # clear fault condition

        t = time.time()
        self.server.TimeStamp('servo', t)
        if result & ServoTelemetry.VOLTAGE:
            self.voltage.set(self.driver.voltage)
        if result & ServoTelemetry.CONTROLLER_TEMP:
            self.controller_temp.set(self.driver.controller_temp)
        if result & ServoTelemetry.MOTOR_TEMP:
            self.motor_temp.set(self.driver.motor_temp)
        if result & ServoTelemetry.RUDDER_POS:
            if math.isnan(self.driver.rudder_pos):
                self.rudder_pos.update(False)
            else:
                self.rudder_pos.set(self.driver.rudder_pos)
        if result & ServoTelemetry.CURRENT:
            self.current.set(self.driver.current)
            # integrate power consumption
            dt = (t - self.current_timestamp)
            #print 'have current', round(1/dt), dt
            self.current_timestamp = t
            if self.current.value:
                amphours = self.current.value*dt/3600
                self.amphours.set(self.amphours.value + amphours)
            lp = .003*dt # 5 minute time constant to average wattage
            self.watts.set((1-lp)*self.watts.value + lp*self.voltage.value*self.current.value)
        if result & ServoTelemetry.FLAGS:
            self.flags.updatedriver(self.driver.flags)
            self.engauged.update(not not self.driver.flags & ServoFlags.ENGAUGED)

        self.send_command()

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

def test(device_path, baud):
    from arduino_servo.arduino_servo import ArduinoServo
    print 'probing arduino servo on', device_path
    device = serial.Serial(device_path, baud)
    device.timeout=0 #nonblocking
    fcntl.ioctl(device.fileno(), TIOCEXCL) #exclusive
    driver = ArduinoServo(device.fileno())
    t0 = time.time()
    if driver.initialize(baud):
        print 'arduino servo found'
        exit(0)
    exit(1)
        
def main():
    for i in range(len(sys.argv)):
        if sys.argv[i] == '-t':
            if len(sys.argv) < i + 3:
                print 'device and baud needed for option -t'
                exit(1)
            test(sys.argv[i+1], int(sys.argv[i+2]))
    
    import serialprobe
    print 'Servo Server'
    server = SignalKServer()
    serial_probe = serialprobe.SerialProbe()
    servo = Servo(server, serial_probe)
    servo.max_current.set(10)

    period = .1
    start = lastt = time.time()
    while True:
        servo.poll()

        if servo.driver:
            print 'voltage:', servo.voltage.value, 'current', servo.current.value, 'ctrl temp', servo.controller_temp.value, 'motor temp', servo.motor_temp.value, 'rudder pos', servo.rudder_pos.value, 'flags', servo.flags.strvalue()
            #print servo.command.value, servo.speed.value, servo.windup
            pass
        server.HandleRequests()

        dt = period - time.time() + lastt
        if dt > 0 and dt < period:
            time.sleep(dt)
            lastt += period
        else:
            lastt = time.time()


if __name__ == '__main__':
    main()
