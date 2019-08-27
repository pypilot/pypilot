#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
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
import serialprobe

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
        self.engaged = False

    def raw_command(self, command):
        if command == 0:
            stop()
            return

        if not self.engaged:
            wiringpi.pinMode(1, wiringpi.GPIO.PWM_OUTPUT)
            wiringpi.pwmSetMode( wiringpi.GPIO.PWM_MODE_MS )

            # fix this to make it higher resolution!!!
            wiringpi.pwmSetRange( 1000 )
            wiringpi.pwmSetClock( 400 )
            self.engaged = True
            
        clockcmd = 60 + 30*command
        clockcmd = int(min(110, max(36, clockcmd)))
        wiringpi.pwmWrite(1, clockcmd)

    def stop():
        wiringpi.pinMode(1, wiringpi.GPIO.PWM_INPUT)
        self.engaged = False
        
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
    ENGAGED = 8

    INVALID=16*1
    FWD_FAULTPIN=16*2
    REV_FAULTPIN=16*4
    BADVOLTAGE=16*8

    MIN_RUDDER=256*1
    MAX_RUDDER=256*2
    CURRENT_RANGE=256*4
    BAD_FUSES=256*8

    DRIVER_MASK = 4095 # bits used for driver flags

    FWD_FAULT=4096*1 # overcurrent faults
    REV_FAULT=4096*2
    DRIVER_TIMEOUT = 4096*4
    SATURATED = 4096*8

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
        if self.value & self.ENGAGED:
            ret += 'ENGAGED '
        if self.value & self.INVALID:
            ret += 'INVALID '
        if self.value & self.FWD_FAULTPIN:
            ret += 'FWD_FAULTPIN '
        if self.value & self.REV_FAULTPIN:
            ret += 'REV_FAULTPIN '
        if self.value & self.BADVOLTAGE:
            ret += 'BADVOLTAGE '
        if self.value & self.MIN_RUDDER:
            ret += 'MIN_RUDDER '
        if self.value & self.MAX_RUDDER:
            ret += 'MAX_RUDDER '
        if self.value & self.BAD_FUSES:
            ret += 'BAD_FUSES '
        if self.value & self.FWD_FAULT:
            ret += 'FWD_FAULT '
        if self.value & self.REV_FAULT:
            ret += 'REV_FAULT '
        if self.value & self.DRIVER_TIMEOUT:
            ret += 'DRIVER_TIMEOUT '
        if self.value & self.SATURATED:
            ret += 'SATURATED '
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
    RUDDER = 128
    EEPROM = 256

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

    def __init__(self, server, sensors):
        self.server = server
        self.sensors = sensors
        self.lastdir = 0 # doesn't matter

        self.servo_calibration = ServoCalibration(self)
        self.calibration = self.Register(JSONValue, 'calibration', {})
        self.load_calibration()

        timestamp = server.TimeStamp('servo')
        self.min_speed = self.Register(RangeProperty, 'min_speed', 1, 0, 1, persistent=True)
        self.max_speed = self.Register(RangeProperty, 'max_speed', 1, 0, 1, persistent=True)
        self.speed_gain = self.Register(RangeProperty, 'speed_gain', 0, 0, 1, persistent=True)
        self.duty = self.Register(SensorValue, 'duty', timestamp)

        self.faults = self.Register(ResettableValue, 'faults', 0)

        # power usage
        self.current_timestamp = time.time()
        self.voltage = self.Register(SensorValue, 'voltage', timestamp)
        self.current = self.Register(SensorValue, 'current', timestamp)
        self.controller_temp = self.Register(SensorValue, 'controller_temp', timestamp)
        self.motor_temp = self.Register(SensorValue, 'motor_temp', timestamp)

        self.engaged = self.Register(BooleanValue, 'engaged', False)
        self.max_current = self.Register(RangeProperty, 'max_current', 2, 0, 60, persistent=True)
        self.current.factor = self.Register(RangeProperty, 'current.factor', 1, 0.8, 1.2, persistent=True)
        self.current.offset = self.Register(RangeProperty, 'current.offset', 0, -1.2, 1.2, persistent=True)
        self.voltage.factor = self.Register(RangeProperty, 'voltage.factor', 1, 0.8, 1.2, persistent=True)
        self.voltage.offset = self.Register(RangeProperty, 'voltage.offset', 0, -1.2, 1.2, persistent=True)
        self.max_controller_temp = self.Register(RangeProperty, 'max_controller_temp', 60, 45, 100, persistent=True)
        self.max_motor_temp = self.Register(RangeProperty, 'max_motor_temp', 60, 30, 100, persistent=True)

        self.max_slew_speed = self.Register(RangeProperty, 'max_slew_speed', 30, 0, 100, persistent=True)
        self.max_slew_slow = self.Register(RangeProperty, 'max_slew_slow', 50, 0, 100, persistent=True)
        self.gain = self.Register(RangeProperty, 'gain', 1, -10, 10, persistent=True)
        self.period = self.Register(RangeProperty, 'period', .7, .1, 3, persistent=True)
        self.compensate_current = self.Register(BooleanProperty, 'compensate_current', False, persistent=True)
        self.compensate_voltage = self.Register(BooleanProperty, 'compensate_voltage', False, persistent=True)
        self.amphours = self.Register(ResettableValue, 'amp_hours', 0, persistent=True)
        self.watts = self.Register(SensorValue, 'watts', timestamp)

        self.command = self.Register(TimedProperty, 'command', 0)
        self.position_command = self.Register(TimedProperty, 'position_command', 0)

        self.speed = self.Register(SensorValue, 'speed', timestamp)

        self.position = self.Register(SensorValue, 'position', timestamp)
        self.position.elp = 0
        self.position.set(0)
        self.position.p = self.Register(RangeProperty, 'position.p', .2, .01, 1, persistent=True)
        self.position.i = self.Register(RangeProperty, 'position.i', .025, 0, 1, persistent=True);
        self.position.d = self.Register(RangeProperty, 'position.d', .1, 0, 1, persistent=True);

        self.rawcommand = self.Register(SensorValue, 'raw_command', timestamp)

        self.use_eeprom = self.Register(BooleanValue, 'use_eeprom', True)

        self.position.inttime = time.time()
        self.position.amphours = 0

        self.windup = 0
        self.last_speed = 0
        self.windup_change = 0

        self.disengaged = True
        self.disengauge_on_timeout = self.Register(BooleanValue, 'disengauge_on_timeout', True, persistent=True)
        self.force_engaged = False

        self.last_zero_command_time = self.command_timeout = time.time()
        self.driver_timeout_start = 0

        self.state = self.Register(StringValue, 'state', 'none')

        self.controller = self.Register(StringValue, 'controller', 'none')
        self.flags = self.Register(ServoFlags, 'flags')

        self.driver = False
        self.raw_command(0)

    def Register(self, _type, name, *args, **kwargs):
        return self.server.Register(_type(*(['servo.' + name] + list(args)), **kwargs))

    def send_command(self):
        t = time.time()

        if not self.disengauge_on_timeout.value:
            self.disengaged = False

        if self.servo_calibration.thread.is_alive():
            print 'cal thread'
            return

        t = time.time()
        dp = t - self.position_command.time
        dc = t - self.command.time

        if dp < dc and not self.sensors.rudder.invalid():
            timeout = 10 # position command will expire after 10 seconds
            self.disengaged = False
            if abs(self.sensors.rudder.angle.value - self.command.value) < 1:
                self.command.set(0)
            else:
                self.do_position_command(self.position_command.value)
                return
        elif self.command.value and not self.fault():
            timeout = 1 # command will expire after 1 second
            if time.time() - self.command.time > timeout:
                #print 'servo command timeout', time.time() - self.command.time
                self.command.set(0)
            self.disengaged = False
        self.do_command(self.command.value)

    def do_position_command(self, position):
        e = self.position.value - position;
        d = self.speed.value
        self.position.elp = .98*self.position.elp + .02*min(max(e, -30), 30)
        #self.position.dlp = .8*self.position.dlp + .2*d;

        p = self.position.p.value*e
        i = self.position.i.value*self.position.elp
        d = self.position.d.value*d
        pid = p + i + d

        self.do_command(pid)
            
    def do_command(self, speed):
        speed *= self.gain.value
        if not speed or self.fault():
            #print 'timeout', t - self.command_timeout
            if self.disengauge_on_timeout.value and \
               not self.force_engaged and \
               time.time() - self.command_timeout > self.period.value*3:
                self.disengaged = True
            self.raw_command(0)
            return
        
        if speed == 0 and self.speed.value == 0: # optimization
            self.raw_command(0)
            return

        if self.flags.value & (ServoFlags.FWD_FAULT | ServoFlags.MAX_RUDDER) and speed > 0 or \
           self.flags.value & (ServoFlags.REV_FAULT | ServoFlags.MIN_RUDDER) and speed < 0:
            #print 'refuse to move', speed
            self.raw_command(0)
            return # abort

        t = time.time()
        dt = t - self.position.inttime
        self.position.inttime = t
        if self.sensors.rudder.invalid():
            # integrate position if no rudder feedback
            # crude integration of position from speed
            position = self.position.value + self.speed.value * dt
            self.position.set(min(max(position, 0), 1))

        #print 'integrate pos', self.position, self.speed, speed, dt, self.fwd_fault, self.rev_fault
        if self.position.value < .9*self.sensors.rudder.range.value:
            self.flags.clearbit(ServoFlags.FWD_FAULT)
        if self.position.value > -.9*self.sensors.rudder.range.value:
            self.flags.clearbit(ServoFlags.REV_FAULT)
            
        if self.compensate_voltage.value and self.voltage.value:
            speed *= 12 / self.voltage.value

        if self.compensate_current.value:
            # get current
            ampseconds = 3600*(self.amphours.value - self.position.amphours)
            current = ampseconds / dt
            self.position.amphours = self.amphours.value
            pass #todo fix this
        # allow higher current with higher voltage???
        #max_current = self.max_current.value
        #if self.compensate_voltage.value:
        #    max_current *= self.voltage.value/voltage
        
        min_speed = self.min_speed.value
        
        # ensure max_speed is at least min_speed
        if min_speed > self.max_speed.value:
            self.max_speed.set(min_speed)

        # compute duty cycle
        lp = .001
        self.duty.set(lp*int(not not speed) + (1-lp)*self.duty.value)

        # adjust speed based on current duty
        min_speed += (self.max_speed.value - min_speed)*self.duty.value*self.speed_gain.value

        # ensure it is in range
        min_speed = max(min(min_speed, self.max_speed.value), self.min_speed.value)
        
        # integrate windup
        self.windup += (speed - self.last_speed) * dt

        # if windup overflows, move at minimum speed
        if abs(self.windup) > self.period.value*min_speed / 1.5:
            if abs(speed) < min_speed:
                speed = min_speed if self.windup > 0 else -min_speed
        else:
            speed = 0

        # don't let windup overflow
        if abs(self.windup) > 1.5*self.period.value:
            self.flags.setbit(ServoFlags.SATURATED)
            self.windup = 1.5*self.period.value*sign(self.windup)
        else:
            self.flags.clearbit(ServoFlags.SATURATED)
        #print 'windup', self.windup, dt, self.windup / dt, speed, self.speed
            
        if speed * self.last_speed <= 0: # switched direction or stopped?
            if t - self.windup_change < self.period.value:
                # less than period, keep previous direction, but use minimum speed
                if self.last_speed > 0:
                    speed = min_speed
                elif self.last_speed < 0:
                    speed = -min_speed
                else:
                    speed = 0
            else:
                self.windup_change = t

        # clamp to max speed
        speed = min(max(speed, -self.max_speed.value), self.max_speed.value)

        if not self.sensors.rudder.invalid():
            self.speed.set(speed)

        #print 'speed', speed
        self.last_speed = speed

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

        self.raw_command(raw_speed)

    def raw_command(self, command):
        self.rawcommand.set(command)
        if command <= 0:
            if command < 0:
                self.state.update('reverse')
                self.lastdir = -1
            else:
                if self.sensors.rudder.invalid():
                    self.speed.set(0)
                self.state.update('idle')
        else:
            self.state.update('forward')
            self.lastdir = 1

        t = time.time()
        if command == 0:
            # only send at .2  seconds when command is zero for more than a second
            if t > self.command_timeout + 1 and t - self.last_zero_command_time < .2:
                return
            self.last_zero_command_time = t
        else:
            self.command_timeout = t

        if self.driver:
            uncorrected_max_current = max(0, self.max_current.value - self.current.offset.value)/ self.current.factor.value 
            if self.disengaged: # keep sending disengauge to keep sync
                self.driver.disengauge()
                self.send_driver_params(uncorrected_max_current)
            else:
                max_current = uncorrected_max_current
                if self.flags.value & ServoFlags.FWD_FAULT or \
                   self.flags.value & ServoFlags.REV_FAULT: # allow more current to "unstuck" ram
                    max_current *= 2
                self.send_driver_params(max_current)

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
        self.sensors.rudder.update(False)

        # for unknown reasons setting timeout to 0 here (already 0)
        # makes device.close() take only .001 seconds instead of .02 seconds
        # but it throws an exception for usb serial ports which we can ignore
        try:
            self.device.timeout=0
        except:
            pass
        self.device.close()
        self.driver = False

    def send_driver_params(self, _max_current):
        self.driver.params(_max_current, self.max_controller_temp.value, self.max_motor_temp.value, self.sensors.rudder.range.value, self.sensors.rudder.offset.value, self.sensors.rudder.scale.value, self.sensors.rudder.nonlinearity.value, self.max_slew_speed.value, self.max_slew_slow.value, self.current.factor.value, self.current.offset.value, self.voltage.factor.value, self.voltage.offset.value, self.min_speed.value, self.max_speed.value, self.gain.value)

    def poll(self):
        if not self.driver:
            device_path = serialprobe.probe('servo', [38400], 1)
            if device_path:
                #from arduino_servo.arduino_servo_python import ArduinoServo
                from arduino_servo.arduino_servo import ArduinoServo
                try:
                    device = serial.Serial(*device_path)
                    device.timeout=0 #nonblocking
                    fcntl.ioctl(device.fileno(), TIOCEXCL) #exclusive
                except Exception as e:
                    print 'failed to open servo on:', device_path, e
                    return
                self.driver = ArduinoServo(device.fileno(), device_path[1])
                uncorrected_max_current = max(0, self.max_current.value - self.current.offset.value)/ self.current.factor.value 
                self.send_driver_params(uncorrected_max_current)
                self.device = device
                self.device.path = device_path[0]
                self.lastpolltime = time.time()

        if not self.driver:
            return

        self.servo_calibration.poll()
                
        result = self.driver.poll()

        if result == -1:
            print 'servo lost'
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

            if self.controller.value == 'none':
                device_path = [self.device.port, self.device.baudrate]
                print 'arduino servo found on', device_path
                serialprobe.success('servo', device_path)
                self.controller.set('arduino')
                self.driver.command(0)


        t = time.time()
        self.server.TimeStamp('servo', t)
        if result & ServoTelemetry.VOLTAGE:
            # apply correction
            corrected_voltage = self.voltage.factor.value*self.driver.voltage;
            corrected_voltage += self.voltage.offset.value
            self.voltage.set(round(corrected_voltage, 3))

        if result & ServoTelemetry.CONTROLLER_TEMP:
            self.controller_temp.set(self.driver.controller_temp)
        if result & ServoTelemetry.MOTOR_TEMP:
            self.motor_temp.set(self.driver.motor_temp)
        if result & ServoTelemetry.RUDDER:
            if self.driver.rudder and not math.isnan(self.driver.rudder):
                data = {'angle': self.driver.rudder, 'timestamp' : t,
                        'device': self.device.path}
                self.sensors.write('rudder', data, 'servo')
        if result & ServoTelemetry.CURRENT:
            # apply correction
            corrected_current = self.current.factor.value*self.driver.current
            if self.driver.current:
                corrected_current = max(0, corrected_current + self.current.offset.value)
            
            self.current.set(round(corrected_current, 3))
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
            self.max_current.set_max(40 if self.driver.flags & ServoFlags.CURRENT_RANGE else 20)
            flags = self.flags.value & ~ServoFlags.DRIVER_MASK | self.driver.flags

            # if rudder angle comes from serial or tcp, may need to set these flags
            angle = self.sensors.rudder.angle.value
            if angle: # note, this is ok here for both False and 0
                if abs(angle) > self.sensors.rudder.range.value:
                    if angle > 0:
                        flags |= ServoFlags.MIN_RUDDER
                    else:
                        flags |= ServoFlags.MAX_RUDDER

            self.flags.update(flags)

            self.engaged.update(not not self.driver.flags & ServoFlags.ENGAGED)

        if result & ServoTelemetry.EEPROM and self.use_eeprom.value: # occurs only once after connecting
            self.max_current.set(self.driver.max_current)
            self.max_controller_temp.set(self.driver.max_controller_temp)
            self.max_motor_temp.set(self.driver.max_motor_temp)
            self.max_slew_speed.set(self.driver.max_slew_speed)
            self.max_slew_slow.set(self.driver.max_slew_slow)
            #print "GOT EEPROM", self.driver.rudder_scale, self.driver.rudder_offset, self.driver.rudder_range
            # if self.sensors.rudder.source == 'servo':
            self.sensors.rudder.scale.set(self.driver.rudder_scale)
            self.sensors.rudder.nonlinearity.set(self.driver.rudder_nonlinearity)
            self.sensors.rudder.offset.set(self.driver.rudder_offset)
            self.sensors.rudder.range.set(self.driver.rudder_range)
            self.current.factor.set(self.driver.current_factor)
            self.current.offset.set(self.driver.current_offset)
            self.voltage.factor.set(self.driver.voltage_factor)
            self.voltage.offset.set(self.driver.voltage_offset)
            self.min_speed.set(self.driver.min_motor_speed);
            self.max_speed.set(self.driver.max_motor_speed);
            self.gain.set(self.driver.gain);

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

        if not self.sensors.rudder.invalid():
            self.position.set(self.sensors.rudder.angle.value)
            
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
            self.calibration.set({'forward': [.2, .8], 'reverse': [.2, .8]})

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
    
    print 'Servo Server'
    server = SignalKServer()

    from sensors import Sensors
    sensors = Sensors(server)
    servo = Servo(server, sensors)
    servo.max_current.set(10)

    period = .1
    start = lastt = time.time()
    while True:
        servo.poll()
        sensors.poll()

        if servo.controller.value != 'none':
            print 'voltage:', servo.voltage.value, 'current', servo.current.value, 'ctrl temp', servo.controller_temp.value, 'motor temp', servo.motor_temp.value, 'rudder pos', sensors.rudder.angle.value, 'flags', servo.flags.strvalue()
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

