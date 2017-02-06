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
import autopilot, serialprobe

from crc import crc8

import serial

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

    def command(self, command):
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
class ArduinoServoFlags(Value):
    SYNC = 1
    FAULTPIN = 2
    OVERCURRENT = 4
    ENGAUGED = 8
    
    def __init__(self, name):
        super(ArduinoServoFlags, self).__init__(name, 0)
            
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
        
    def get_request(self):
        return json.dumps({self.name : {"value" : self.strvalue()}})
        
class ArduinoServo:
    sync_bytes = [0xe7, 0xf9, 0xc7, 0x1e, 0xa7, 0x19, 0x1c, 0xb3]
    def __init__(self, device):
        self.in_sync = self.out_sync = 0
        self.in_sync_count = 0        
        self.in_buf = []
        self.device = serial.Serial(*device)

        #self.device.setTimeout(0)
        self.device.timeout=0
        self.lastcommand = False
        self.lasttime = time.time()
        self.servo = False

        self.flags = ArduinoServoFlags('servo/flags')

        cnt = 0

        while self.flags.value & ArduinoServoFlags.OVERCURRENT or \
          not self.flags.value & ArduinoServoFlags.SYNC:
            self.stop()
            while self.poll():
                pass
            time.sleep(.001)
            cnt+=1
            if cnt == 500:
                raise 'failed to initialize servo', device

    def send_value(self, value):
        value = int(value)
        code = [ArduinoServo.sync_bytes[self.out_sync], value&0xff, (value>>8)&0xff]
        b = '%c%c%c' % (code[1], code[2], crc8(code))

        self.device.write(b)
        #self.device.flush()
        self.out_sync += 1

    def raw_command(self, command):
        if self.out_sync == 0:
            if self.servo:
                max_current = self.servo.max_current.value
            else:
                max_current = 10
            self.send_value(max_current*65536.0/11)
        self.send_value(command)
        if self.out_sync == len(ArduinoServo.sync_bytes):
            self.out_sync = 0;
        
    def command(self, command):
        if command == self.lastcommand:
            dt = time.time() - self.lasttime
            # onlyupdate at .5 seconds when command is zero
            if command == 0 and dt < .5:
                return
        self.lastcommand = command
        self.lasttime = time.time()

        command = min(max(command, -1), 1)

        self.raw_command((command+1)*1000)

    def stop(self):
        self.raw_command(0x5342)

    def poll(self):
        c = self.device.read(1)
        self.device.flush()
        if len(c) == 0:
            return False

        self.in_buf.append(ord(c))
        ret = {'result': True}
        if len(self.in_buf) == 3:
            code = [ArduinoServo.sync_bytes[self.in_sync]] + self.in_buf[:2]
            crc = crc8(code)
	    #print 'got code', code, self.in_buf
            if crc == self.in_buf[2]:
                if self.in_sync_count == 2:
                    value = self.in_buf[0] + (self.in_buf[1]<<8)
                    if self.in_sync > 0:
                        ret['current'] = value * 1.1 / .1 / 65536
                    else:
                        ret['voltage'] = (value >> 4) * 1.1 * 10560 / 560 / 4096
                        self.flags.set(value & 0xf)

                self.in_sync+=1
                if self.in_sync == len(ArduinoServo.sync_bytes):
                    self.in_sync = 0;
                    if self.in_sync_count < 2:
                        self.in_sync_count+=1

                self.in_buf = []
            else:
                self.in_sync = self.in_sync_count = 0
                self.in_buf = self.in_buf[1:]

        return ret

    def fault(self):
        return self.flags.value & (ArduinoServoFlags.FAULTPIN | ArduinoServoFlags.OVERCURRENT) != 0

# special case for servo calibration, and to convert the map key
class CalibrationProperty(Property):
    def __init__(self, name, initial):
        super(CalibrationProperty, self).__init__(name, initial)
        self.set(initial)

    def set(self, value):
        nvalue = {}
        for cal in value: 
            nvalue[float(cal)] = value[cal]
        if not 0 in nvalue: #remove?
            nvalue[0] = 0, 0, 0, 12, 0

        return super(CalibrationProperty, self).set(nvalue)

# a property which records the time when it is updated
class TimestampProperty(Property):
    def __init__(self, name, initial):
        super(TimestampProperty, self).__init__(name, initial)
        self.set(initial)

    def set(self, value):
        self.timestamp = time.time()
        return super(TimestampProperty, self).set(value)

class ResettableValue(Property):
    def __init__(self, name, initial, **kwargs):
        super(ResettableValue, self).__init__(name, initial, **kwargs)
        self.initial = initial

    def processes(self):
        p = super(Property, self).processes()
        p['set'] = self.setdata
        return p

    def setdata(self, socket, data):
        if data['value'] != self.initial:
            print 'resettable value', self.name, 'invalid set'
        else:
            self.set(data['value'])

    def type(self):
        return 'ResettableValue'
    
class Servo:
    calibration_filename = autopilot.pypilot_dir + 'servocalibration'

    def __init__(self, server):
        self.server = server
        self.fwd_fault = self.rev_fault = False
        self.min_speed = self.Register(RangeProperty, 'Min Speed', .3, 0, 1, persistent=True)
        self.max_speed = self.Register(RangeProperty, 'Max Speed', 1, 0, 1, persistent=True)
        self.brake_hack = self.Register(BooleanProperty, 'Brake Hack', True, persistent=True)
        self.brake_hack_state = 0

        # power usage
        self.command = self.Register(TimestampProperty, 'command', 0)
        self.rawcommand = self.Register(TimestampProperty, 'raw_command', False)
        self.voltage = self.Register(SensorValue, 'voltage')
        self.current = self.Register(SensorValue, 'current')
        self.engauged = self.Register(Value, 'engauged', False)
        self.max_current = self.Register(RangeProperty, 'Max Current', 2, 0, 10, persistent=True)
        self.slow_period = self.Register(RangeProperty, 'Slow Period', 1.5, .1, 10, persistent=True)
        self.compensate_current = self.Register(BooleanProperty, 'Compensate Current', False, persistent=True)
        self.compensate_voltage = self.Register(BooleanProperty, 'Compensate Voltage', False, persistent=True)
        self.amphours = self.Register(Value, 'Amp Hours', 0)
        self.powerconsumption = self.Register(ResettableValue, 'Power Consumption', 0, persistent=True)

        self.calibration = self.Register(CalibrationProperty, 'calibration', {})
        self.load_calibration()

        self.position = .5
        self.speed = 0
        self.lastpositiontime = time.time()
        self.lastpositionamphours = 0

        self.mode = self.Register(Value, 'mode', 'none')
        self.controller = self.Register(Value, 'controller', 'none')
        self.drive = self.Register(Value, 'drivetype', 'relative')

        self.driver = False

    def Register(self, _type, name, *args, **kwargs):
        return self.server.Register(_type(*(['servo/' + name] + list(args)), **kwargs))

    def send_command(self):
        timeout = 1 # command will expire after 1 second
        if self.rawcommand.value:
            if time.time() - self.rawcommand.timestamp > timeout:
                self.rawcommand.set(False)
            else:
                self.raw_command(self.rawcommand.value)
        else:
            if time.time() - self.command.timestamp > timeout:
                command = 0
            else:
                command = self.command.value
            if self.drive.value == 'absolute':
                self.position_command(command)
            elif self.drive.value == 'relative':
                self.velocity_command(command)
            else:
                print 'unknown servo drive type'
    
    # estimate position from integrating speed commands
    def position_command(self, position):

        # absolute mode have advantage here
        if self.drive.value == 'absolute':
            if self.driver:
                self.driver.command(2*position - 1)
            self.position = position
            return

        # use PD filter to command speed?
        P, D = 2, 0

        self.velocity_command(P*(position - self.position) - D*self.speed)

    def velocity_command(self, speed):
        if self.drive.value == 'absolute':
            # integrate to determin actual position
            t0 = time.time()
            dt = t0 - self.last_position_time
            self.last_position_time = t0
            self.position += command * dt
            self.position = min(max(self.position, -1), 1)
            self.position_command(self.position)
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
                if (time.time() % self.slow_period.value) / self.slow_period.value > duty:
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
            self.mode.set('brake')
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
                    self.mode.set('reverse')
            else:
#                if self.mode.value == 'idle':
#                    return
                self.mode.set('idle')
        else:
            self.mode.set('forward')

        if not self.driver:
            self.driver = serialprobe.probe('servo', ArduinoServo, [115200])

            if self.driver:
                self.driver.servo = self
                self.controller.set('arduino')
                self.server.Register(self.driver.flags)

                if self.brake_hack.value:
                    self.driver.command(-.2) # flush any brake
                self.driver.command(0)
                self.lastpolltime = time.time()

        if self.driver:
            try:
                self.driver.command(command)
            except:
                self.close_driver()

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
        self.driver = False


    def poll(self):
        while self.driver:
            try:
                result = self.driver.poll()
            except:
                self.close_driver()
                break

            if not result:
                d = time.time() - self.lastpolltime
                if d > 10: # correct for clock skew
                    self.lastpolltime = time.time()
                elif d > 8:
                    print 'd', d
                    self.close_driver()
                    pass
                break
            self.lastpolltime = time.time()

            self.engauged.set(not not self.driver.flags.value & ArduinoServoFlags.ENGAUGED)
            if self.fault():
                if self.speed > 0:
                    self.fwd_fault = True
                elif self.speed < 0:
                    self.rev_fault = True

            if 'voltage' in result:
                self.voltage.set(result['voltage'])
            if 'current' in result:
                lasttimestamp = self.current.timestamp
                self.current.set(result['current'])

                # integrate power consumption
                dt = (self.current.timestamp-lasttimestamp)
                self.amphours.set(self.amphours.value + self.current.value*dt/3600)
                power = self.voltage.value*self.current.value
                self.powerconsumption.set(self.powerconsumption.value + dt*power/3600)

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
