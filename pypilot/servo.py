#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from __future__ import print_function
import os, math, sys, time
from pypilot import pyjson

from pypilot.client import pypilotClient
from pypilot.values import *
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
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
    OVERTEMP_FAULT = 2
    OVERCURRENT_FAULT = 4
    ENGAGED = 8

    INVALID=16*1
    PORT_PIN_FAULT=16*2
    STARBOARD_PIN_FAULT=16*4
    BADVOLTAGE_FAULT=16*8

    MIN_RUDDER_FAULT=256*1
    MAX_RUDDER_FAULT=256*2
    CURRENT_RANGE=256*4
    BAD_FUSES=256*8

    DRIVER_MASK = 4095 # bits used for driver flags

    PORT_FAULT=4096*1 # overcurrent faults
    STARBOARD_FAULT=4096*2
    DRIVER_TIMEOUT = 4096*4
    SATURATED = 4096*8

    def __init__(self, name):
        super(ServoFlags, self).__init__(name, 0)
          
    def strvalue(self):
        ret = ''
        if self.value & self.SYNC:
            ret += 'SYNC '
        if self.value & self.OVERTEMP_FAULT:
            ret += 'OVERTEMP_FAULT '
        if self.value & self.OVERCURRENT_FAULT:
            ret += 'OVERCURRENT_FAULT '
        if self.value & self.ENGAGED:
            ret += 'ENGAGED '
        if self.value & self.INVALID:
            ret += 'INVALID '
        if self.value & self.PORT_PIN_FAULT:
            ret += 'PORT_PIN_FAULT '
        if self.value & self.STARBOARD_PIN_FAULT:
            ret += 'STARBOARD_PIN_FAULT '
        if self.value & self.BADVOLTAGE_FAULT:
            ret += 'BADVOLTAGE_FAULT '
        if self.value & self.MIN_RUDDER_FAULT:
            ret += 'MIN_RUDDER_FAULT '
        if self.value & self.MAX_RUDDER_FAULT:
            ret += 'MAX_RUDDER_FAULT '
        if self.value & self.BAD_FUSES:
            ret += 'BAD_FUSES '
        if self.value & self.PORT_FAULT:
            ret += 'PORT_FAULT '
        if self.value & self.STARBOARD_FAULT:
            ret += 'STARBOARD_FAULT '
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
            
    def port_fault(self):
        self.update((self.value | ServoFlags.PORT_FAULT) \
                    & ~ServoFlags.STARBOARD_FAULT)

    def starboard_fault(self):
        self.update((self.value | ServoFlags.STARBOARD_FAULT) \
                    & ~ServoFlags.PORT_FAULT)
        
    def get_pypilot(self):
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
    def __init__(self, name):
        self.time = 0
        super(TimedProperty, self).__init__(name, 0)

    def set(self, value):
        self.time = time.monotonic()
        return super(TimedProperty, self).set(value)

class Servo(object):
    calibration_filename = autopilot.pypilot_dir + 'servocalibration'

    def __init__(self, client, sensors):
        self.client = client
        self.sensors = sensors
        self.lastdir = 0 # doesn't matter

        self.servo_calibration = ServoCalibration(self)
        self.calibration = self.register(JSONValue, 'calibration', {})
        self.load_calibration()

        self.command = self.register(TimedProperty, 'command')
        self.position_command = self.register(TimedProperty, 'position_command')

        self.speed_gain = self.register(RangeProperty, 'speed_gain', 0, 0, 1)
        self.duty = self.register(SensorValue, 'duty')

        self.faults = self.register(ResettableValue, 'faults', 0, persistent=True)

        # power usage
        self.voltage = self.register(SensorValue, 'voltage')
        self.current = self.register(SensorValue, 'current')
        self.current.lasttime = time.monotonic()
        self.controller_temp = self.register(SensorValue, 'controller_temp')
        self.motor_temp = self.register(SensorValue, 'motor_temp')

        self.engaged = self.register(BooleanValue, 'engaged', False)
        self.max_current = self.register(RangeSetting, 'max_current', 7, 0, 60, 'amps')
        self.current.factor = self.register(RangeProperty, 'current.factor', 1, 0.8, 1.2, persistent=True)
        self.current.offset = self.register(RangeProperty, 'current.offset', 0, -1.2, 1.2, persistent=True)
        self.voltage.factor = self.register(RangeProperty, 'voltage.factor', 1, 0.8, 1.2, persistent=True)
        self.voltage.offset = self.register(RangeProperty, 'voltage.offset', 0, -1.2, 1.2, persistent=True)
        self.max_controller_temp = self.register(RangeProperty, 'max_controller_temp', 60, 45, 100, persistent=True)
        self.max_motor_temp = self.register(RangeProperty, 'max_motor_temp', 60, 30, 100, persistent=True)

        self.max_slew_speed = self.register(RangeSetting, 'max_slew_speed', 18, 0, 100, '')
        self.max_slew_slow = self.register(RangeSetting, 'max_slew_slow', 28, 0, 100, '')
        self.gain = self.register(RangeProperty, 'gain', 1, -10, 10, persistent=True)
        self.period = self.register(RangeSetting, 'period', .4, .1, 3, 'sec')
        self.compensate_current = self.register(BooleanProperty, 'compensate_current', False, persistent=True)
        self.compensate_voltage = self.register(BooleanProperty, 'compensate_voltage', False, persistent=True)
        self.amphours = self.register(ResettableValue, 'amp_hours', 0, persistent=True)
        self.watts = self.register(SensorValue, 'watts')

        self.speed = self.register(SensorValue, 'speed')
        self.speed.min = self.register(RangeSetting, 'speed.min', 100, 0, 100, '%')
        self.speed.max = self.register(RangeSetting, 'speed.max', 100, 0, 100, '%')

        self.position = self.register(SensorValue, 'position')
        self.position.elp = 0
        self.position.set(0)
        self.position.p = self.register(RangeProperty, 'position.p', .15, .01, 1, persistent=True)
        self.position.i = self.register(RangeProperty, 'position.i', 0, 0, .1, persistent=True)
        self.position.d = self.register(RangeProperty, 'position.d', .02, 0, .1, persistent=True)

        self.rawcommand = self.register(SensorValue, 'raw_command')

        self.use_eeprom = self.register(BooleanValue, 'use_eeprom', True, persistent=True)

        self.position.inttime = time.monotonic()
        self.position.amphours = 0

        self.windup = 0
        self.windup_change = 0

        self.disengaged = True
        self.disengage_on_timeout = self.register(BooleanValue, 'disengage_on_timeout', True, persistent=True)
        self.force_engaged = False

        self.last_zero_command_time = self.command_timeout = time.monotonic()
        self.driver_timeout_start = 0

        self.state = self.register(StringValue, 'state', 'none')

        self.controller = self.register(StringValue, 'controller', 'none')
        self.flags = self.register(ServoFlags, 'flags')

        self.driver = False
        self.raw_command(0)

    def register(self, _type, name, *args, **kwargs):
        return self.client.register(_type(*(['servo.' + name] + list(args)), **kwargs))

    def send_command(self):
        t = time.monotonic()

        if not self.disengage_on_timeout.value:
            self.disengaged = False

        if self.servo_calibration.thread.is_alive():
            print('cal thread')
            return

        t = time.monotonic()
        dp = t - self.position_command.time
        dc = t - self.command.time

        if dp < dc and not self.sensors.rudder.invalid():
            timeout = 10 # position command will expire after 10 seconds
            self.disengaged = False
            if abs(self.position.value - self.command.value) < 1:
                self.command.set(0)
            else:
                self.do_position_command(self.position_command.value)
                return
        elif self.command.value and not self.fault():
            timeout = 1 # command will expire after 1 second
            if time.monotonic() - self.command.time > timeout:
                #print('servo command timeout', time.monotonic() - self.command.time)
                self.command.set(0)
            self.disengaged = False
        self.do_command(self.command.value)

    def do_position_command(self, position):
        e = position - self.position.value
        d = self.speed.value * self.sensors.rudder.range.value

        self.position.elp = .98*self.position.elp + .02*min(max(e, -30), 30)
        #self.position.dlp = .8*self.position.dlp + .2*d

        p = self.position.p.value*e
        i = self.position.i.value*self.position.elp
        d = self.position.d.value*d
        pid = p + i + d
        #print('pid', pid, p, i, d)
        # map in min_speed to max_speed range
        

        self.do_command(pid)
            
    def do_command(self, speed):
        speed *= self.gain.value # apply gain

        # if not moving or faulted, disengauge
        if not speed or self.fault():
            #print('timeout', t - self.command_timeout)
            if self.disengage_on_timeout.value and \
               not self.force_engaged and \
               time.monotonic() - self.command_timeout > self.period.value*3:
                self.disengaged = True
            self.raw_command(0)
            return

        # save computation if not moving
        if speed == 0 and self.speed.value == 0: # optimization
            self.raw_command(0)
            return

        # prevent moving the wrong direction if flags set
        if self.flags.value & (ServoFlags.PORT_FAULT | ServoFlags.MAX_RUDDER_FAULT) and speed > 0 or \
           self.flags.value & (ServoFlags.STARBOARD_FAULT | ServoFlags.MIN_RUDDER_FAULT) and speed < 0:
            self.raw_command(0)
            return # abort

        # compute time and dt
        t = time.monotonic()
        dt = t - self.position.inttime
        self.position.inttime = t

        # clear faults from overcurrent if moved sufficiently the other direction
        rudder_range = self.sensors.rudder.range.value
        if self.position.value < .9*rudder_range:
            self.flags.clearbit(ServoFlags.PORT_FAULT)
        if self.position.value > -.9*rudder_range:
            self.flags.clearbit(ServoFlags.STARBOARD_FAULT)

        # compensate for fluxuating battery voltage
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
        
        # ensure speed max is at least speed min
        if self.speed.min.value > self.speed.max.value:
            self.speed.max.set(self.speed.min.value)

        min_speed = self.speed.min.value/100.0 # convert percent to 0-1
        max_speed = self.speed.max.value/100.0
        
        # adjust speed based on current duty
        min_speed += (max_speed - min_speed)*self.duty.value*self.speed_gain.value

        # ensure it is in range
        min_speed = min(min_speed, max_speed)
        
        # integrate windup
        self.windup += (speed - self.speed.value) * dt

        # if windup overflows, move at least minimum speed
        if abs(self.windup) > self.period.value*min_speed / 1.5:
            if abs(speed) < min_speed:
                speed = min_speed if self.windup > 0 else -min_speed
        else:
            speed = 0

        # don't let windup overflow
        max_windup = 1.5*self.period.value
        if abs(self.windup) > max_windup:
            self.flags.setbit(ServoFlags.SATURATED)
            self.windup = max_windup*sign(self.windup)
        else:
            self.flags.clearbit(ServoFlags.SATURATED)

        last_speed = self.speed.value
        if speed * last_speed <= 0: # switched direction or stopped?
            if t - self.windup_change < self.period.value:
                # less than period, keep previous direction, but use minimum speed
                if last_speed > 0:
                    speed = min_speed
                elif last_speed < 0:
                    speed = -min_speed
                else:
                    speed = 0
            else:
                self.windup_change = t

        # clamp to max speed
        speed = min(max(speed, -max_speed), max_speed)
        self.speed.set(speed)

        # estimate position
        if self.sensors.rudder.invalid():
            # crude integration of position from speed
            position = self.position.value + speed*rudder_range/10 * dt
            self.position.set(min(max(position, -rudder_range), rudder_range))

        try:
            if speed > 0:
                cal = self.calibration.value['port']
            elif speed < 0:
                cal = self.calibration.value['starboard']
            else:
                self.raw_command(0)
                return

            command = cal[0] + abs(speed)*cal[1]
            if speed < 0:
                command = -command
            self.raw_command(command)
        except:
            print ('servo calibration invalid', self.calibration.value)
            self.calibration.set({'port': [.2, .8], 'starboard': [.2, .8]})            

    def raw_command(self, command):
        # compute duty cycle
        lp = .001
        self.duty.set(lp*int(not not command) + (1-lp)*self.duty.value)

        self.rawcommand.set(command)
        if command <= 0:
            if command < 0:
                self.state.update('reverse')
                self.lastdir = -1
            else:
                self.speed.set(0)
                self.state.update('idle')
        else:
            self.state.update('forward')
            self.lastdir = 1

        t = time.monotonic()
        if command == 0:
            # only send at .2 seconds when command is zero for more than a second
            if t > self.command_timeout + 1 and t - self.last_zero_command_time < .2:
                return
            self.last_zero_command_time = t
        else:
            self.command_timeout = t

        if self.driver:
            if self.disengaged: # keep sending disengage to keep sync
                self.send_driver_params()
                self.driver.disengage()
            else:
                mul = 1
                if self.flags.value & ServoFlags.PORT_FAULT or \
                   self.flags.value & ServoFlags.STARBOARD_FAULT: # allow more current to "unstuck" ram
                    mul = 2
                self.send_driver_params(mul)

                self.driver.command(command)

                # detect driver timeout if commanded without measuring current
                if self.current.value:
                    self.flags.clearbit(ServoFlags.DRIVER_TIMEOUT)
                    self.driver_timeout_start = 0
                elif command:
                    if self.driver_timeout_start:
                        if time.monotonic() - self.driver_timeout_start > 1:
                            self.flags.setbit(ServoFlags.DRIVER_TIMEOUT)
                    else:
                        self.driver_timeout_start = time.monotonic()
                        
    def reset(self):
        if self.driver:
            self.driver.reset()

    def close_driver(self):
        print('servo lost connection')
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

    def send_driver_params(self, mul=1):
        uncorrected_max_current = max(0, self.max_current.value - self.current.offset.value) / self.current.factor.value        
        self.driver.params(mul * uncorrected_max_current,
                           self.sensors.rudder.minmax[0],
                           self.sensors.rudder.minmax[1],
                           self.max_current.value,
                           self.max_controller_temp.value,
                           self.max_motor_temp.value,
                           self.sensors.rudder.range.value,
                           self.sensors.rudder.offset.value,
                           self.sensors.rudder.scale.value,
                           self.sensors.rudder.nonlinearity.value,
                           self.max_slew_speed.value,
                           self.max_slew_slow.value,
                           self.current.factor.value,
                           self.current.offset.value,
                           self.voltage.factor.value,
                           self.voltage.offset.value,
                           self.speed.min.value,
                           self.speed.max.value,
                           self.gain.value)

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
                    print('failed to open servo on:', device_path, e)
                    return
                self.driver = ArduinoServo(device.fileno(), device_path[1])
                self.send_driver_params()
                self.device = device
                self.device.path = device_path[0]
                self.lastpolltime = time.monotonic()

        if not self.driver:
            return

        self.servo_calibration.poll()
                
        result = self.driver.poll()
        if result == -1:
            print('servo lost')
            self.close_driver()
            return
        
        t = time.monotonic()
        if result == 0:
            d = t - self.lastpolltime
            if d > 5: # correct for clock skew
                self.lastpolltime = t
            elif d > 4:
                print('servo timeout', d)
                self.close_driver()
        else:
            self.lastpolltime = t

            if self.controller.value == 'none':
                device_path = [self.device.port, self.device.baudrate]
                print('arduino servo found on', device_path)
                serialprobe.success('servo', device_path)
                self.controller.set('arduino')
                self.driver.command(0)


        if result & ServoTelemetry.VOLTAGE:
            # apply correction
            corrected_voltage = self.voltage.factor.value*self.driver.voltage
            corrected_voltage += self.voltage.offset.value
            self.voltage.set(round(corrected_voltage, 3))

        if result & ServoTelemetry.CONTROLLER_TEMP:
            self.controller_temp.set(self.driver.controller_temp)
        if result & ServoTelemetry.MOTOR_TEMP:
            self.motor_temp.set(self.driver.motor_temp)
        if result & ServoTelemetry.RUDDER:
            if self.driver.rudder:
                if math.isnan(self.driver.rudder): # rudder no longer valid
                    if self.sensors.rudder.source.value == 'servo':
                        self.sensors.lostsensor(self.sensors.rudder)
                else:
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
            dt = (t - self.current.lasttime)
            self.current.lasttime = t
            if self.current.value:
                amphours = self.current.value*dt/3600
                self.amphours.set(self.amphours.value + amphours)
            lp = .003*dt # 5 minute time constant to average wattage
            self.watts.set((1-lp)*self.watts.value + lp*self.voltage.value*self.current.value)

        if result & ServoTelemetry.FLAGS:
            self.max_current.set_max(40 if self.driver.flags & ServoFlags.CURRENT_RANGE else 20)
            flags = self.flags.value & ~ServoFlags.DRIVER_MASK | self.driver.flags

            # if rudder angle comes from serial or tcp, may need to set these flags
            # to prevent rudder movement
            angle = self.sensors.rudder.angle.value
            if angle: # note, this is ok here for both False and 0
                if abs(angle) > self.sensors.rudder.range.value:
                    if angle > 0:
                        flags |= ServoFlags.MAX_RUDDER_FAULT
                    else:
                        flags |= ServoFlags.MIN_RUDDER_FAULT

            self.flags.update(flags)

            self.engaged.update(not not self.driver.flags & ServoFlags.ENGAGED)

        if result & ServoTelemetry.EEPROM and self.use_eeprom.value: # occurs only once after connecting
            self.max_current.set(self.driver.max_current)
            self.max_controller_temp.set(self.driver.max_controller_temp)
            self.max_motor_temp.set(self.driver.max_motor_temp)
            self.max_slew_speed.set(self.driver.max_slew_speed)
            self.max_slew_slow.set(self.driver.max_slew_slow)
            self.sensors.rudder.scale.set(self.driver.rudder_scale)
            self.sensors.rudder.nonlinearity.set(self.driver.rudder_nonlinearity)
            self.sensors.rudder.offset.set(self.driver.rudder_offset)
            self.sensors.rudder.range.set(self.driver.rudder_range)
            self.sensors.rudder.update_minmax()
            self.current.factor.set(self.driver.current_factor)
            self.current.offset.set(self.driver.current_offset)
            self.voltage.factor.set(self.driver.voltage_factor)
            self.voltage.offset.set(self.driver.voltage_offset)
            self.speed.min.set(self.driver.min_speed)
            self.speed.max.set(self.driver.max_speed)
            self.gain.set(self.driver.gain)

        if self.fault():
            if not self.flags.value & ServoFlags.PORT_FAULT and \
               not self.flags.value & ServoFlags.STARBOARD_FAULT:
                self.faults.set(self.faults.value + 1)
            
            # if overcurrent then fault in the direction traveled
            # this prevents moving further in this direction
            if self.flags.value & ServoFlags.OVERCURRENT_FAULT:
                if self.lastdir > 0:
                    self.flags.port_fault()                    
                elif self.lastdir < 0:
                    self.flags.starboard_fault()
                if self.sensors.rudder.invalid() and self.lastdir:
                    rudder_range = self.sensors.rudder.range.value
                    self.position.set(self.lastdir*rudder_range)

            self.reset() # clear fault condition

        # update position from rudder feedback    
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
            print('loading servo calibration', filename)
            file = open(filename)
            self.calibration.set(pyjson.loads(file.readline()))
        except:
            print('WARNING: using default servo calibration!!')
            self.calibration.set(False)

    def save_calibration(self):
        file = open(Servo.calibration_filename, 'w')
        file.write(pyjson.dumps(self.calibration))

def test(device_path, baud):
    from arduino_servo.arduino_servo import ArduinoServo
    print('probing arduino servo on', device_path)
    device = serial.Serial(device_path, baud)
    device.timeout=0 #nonblocking
    fcntl.ioctl(device.fileno(), TIOCEXCL) #exclusive
    driver = ArduinoServo(device.fileno())
    t0 = time.monotonic()
    if driver.initialize(baud):
        print('arduino servo found')
        exit(0)
    exit(1)
        
def main():
    for i in range(len(sys.argv)):
        if sys.argv[i] == '-t':
            if len(sys.argv) < i + 3:
                print('device and baud needed for option -t')
                exit(1)
            test(sys.argv[i+1], int(sys.argv[i+2]))
    
    print('Servo Client')
    server = pypilotServer()
    client = pypilotClient(server)

    from sensors import Sensors
    sensors = Sensors(client)
    servo = Servo(client, sensors)
    servo.max_current.set(10)

    period = .1
    start = lastt = time.monotonic()
    while True:
        servo.poll()
        sensors.poll()

        if servo.controller.value != 'none':
            print('voltage:', servo.voltage.value, 'current', servo.current.value, 'ctrl temp', servo.controller_temp.value, 'motor temp', servo.motor_temp.value, 'rudder pos', sensors.rudder.angle.value, 'flags', servo.flags.strvalue())
            #print(servo.command.value, servo.speed.value, servo.windup)
            pass
        client.poll()
        server.poll()

        dt = period - time.monotonic() + lastt
        if dt > 0 and dt < period:
            time.sleep(dt)
            lastt += period
        else:
            lastt = time.monotonic()

if __name__ == '__main__':
    main()
