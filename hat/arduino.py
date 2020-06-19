#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# A separate process manages a possible arduino to receive IR/RF
# The arduino also controls the backlight and provides additional
# pins for more functionallity (tack button, 
# spi port.

import os, sys, time, json
import crc
import lircd

END=0xfe
REWIND=0xfb # reset state
READ=0xf9 # read next byte from output buffer, null terminated
WRITE=0xf6 # bytes are any bytes the master sends 
VALIDATE=0xf3 # release output buffer
AGAIN=0xf0
VERIFY=0xac # master test input buffer for OK, INVALID, AGAIN
OK=0xa8
INVALID=0xa4

RF=0x01
IR=0x02
GP=0x03
SERIAL_DATA=0x40
SET_BACKLIGHT=0x6
SET_BUZZER=0x7
SET_BAUD=0x8

try:
    import RPi.GPIO as GPIO
except:
    GPIO = False
    
class arduino(object):
    def __init__(self, config):
        self.spi = False
        self.nmea_socket = False

        self.config = config
        if 'hat' in config:
            hatconfig = config['hat']
        if hatconfig and 'arduino' in hatconfig:
            self.hatconfig = hatconfig['arduino']
        else:
            self.hatconfig = False

        self.lasttime=0

        self.sent_count = 0
        self.sent_start = time.monotonic()

        self.serial_in_count = 0
        self.serial_out_count = 0
        self.serial_time = self.sent_start

        if not self.hatconfig:
            print('No hat config, arduino not found')

    def open(self):
        if not self.hatconfig:
            return

        device = self.hatconfig['device']
        if not device:
            return

        try:
            if device.startswith('/dev/spidev'):
                # update flash if needed
                filename = os.getenv('HOME') + '/.pypilot/hat.hex'
                if not os.path.exists(filename):
                    print('hat firmware not in', filename)
                    print('skipping verification')
                # try to verify twice because sometimes this fails
                elif not self.verify(filename) and not self.verify(filename):
                    if not self.write(filename) or not self.verify(filename):
                        print('failed to verify or upload', filename)
                        #self.hatconfig['device'] = False # prevent retry
                        #return

                port, slave = int(device[11]), int(device[13])
                print('arduino on spidev%d.%d' % (port, slave))
                import spidev
                self.spi = spidev.SpiDev()
                self.spi.open(port, slave)
                self.spi.max_speed_hz=200000
                self.lastdata = []

        except Exception as e:
            print('failed to communicate with arduino', device, e)
            self.hatconfig = False

    def close(self, e):
        print('failed to read spi:', e)
        self.spi.close()
        self.spi = False


    def send(self, data, id):
        if not self.spi:
            return
        data = [id] + data
        while True:
            t0 = time.monotonic()
            x = AGAIN
            while x != WRITE:
                x = self.spi.xfer([WRITE])[0]
                #print('value write %x %x' % (WRITE, x))
                x = self.spi.xfer([END])[0]
                #time.sleep(.01)
            t1 = time.monotonic()
            for c in data:
                x = self.spi.xfer([c])
                #print('value data %x' % c, [c], x)
            t2 = time.monotonic()
            crc7 = 0x7f & crc.crc8(data)
            #print('computecrc', data, len(data))
            x=self.spi.xfer([crc7])
            #print('value crc %x' % crc7, crc7, x)
            t3 = time.monotonic()
            while x != END:
                x=self.spi.xfer([END])[0]

            t4 = time.monotonic()
            x = AGAIN
            while x != OK and x != INVALID:
                x = self.spi.xfer([VERIFY])[0]
                if time.monotonic() - t4 > .5:
                    print('caught invalid on timeout!!!!')
                    x = INVALID
                    break
                #print('verify %x' % x)
            t5 = time.monotonic()
            #print('times', t1-t0, t2-t1, t3-t2, t4-t3, t5-t4)
            if x == OK:
                #print('verified', x)
                return
            if x == INVALID:
                #print('failed to verify write, retry')
                pass
            else:
                print('invalid verification value!!!!', x)

    def recv(self):
        if not self.spi:
            return []
        while True:
            x = 0
            while x != REWIND:
                x = self.spi.xfer([REWIND])[0]
                #'endprint('rewind %x' % x)
                if x == END:
                    return []
            data = []
            while True:
                x = self.spi.xfer([READ])[0]
                #print('read %x' % x)
                if x == REWIND:
                    continue
                if x == END:
                    if not data:
                        return data
                    crc7 = 0x7f & crc.crc8(data[:-1])
                    #print('end', crc7, x)
                    if crc7 != data[-1]:
                        #print('crc match failed recv, try again', crc7, data[-1])
                        break # bad crc, retry read
                    # inform arduino we read it
                    while x != VALIDATE:
                        y = self.spi.xfer([VALIDATE])[0]
                        x = self.spi.xfer([REWIND])[0]
                        #print('validate', x)
                    return data
                if x >= 128:
                    #print('invalid char read %x' % x)
                    # bad state, reset
                    break
                data.append(x) # otherwise record this byte
            
    def set_backlight(self, value, polarity):
        value = min(max(int(value), 0), 100)
        backlight = [value, polarity]
        self.send(backlight, SET_BACKLIGHT)

    def set_baud(self, value):
        self.baud = value
        baud = value['baud']
        baud = [baud%256, baud/256]
        self.send(baud, SET_BAUD)

    def set_buzzer(self, duration, frequency):
        self.send((duration, frequency), SET_BUZZER)

    def get_baud_rate(self):
        t = time.monotonic()
        dt = t - self.serial_time
        self.serial_time = t
        rate_in = self.serial_in_count*10/dt
        rate_out = self.serial_out_count*10/dt
        self.serial_in_count =self.serial_out_count = 0
        return rate_in, rate_out

    def poll(self):
        if not self.spi:
            self.open()
            return []

        while self.nmea_socket:
            try:
                data = self.nmea_socket.recv()
                if not data:
                    break
                count = 0
                self.serial_in_count += len(data)
                while data:
                    count += 1
                    if count > 10:
                        print('too much serial data, dropping', data)
                        break
                    # up to 64 bytes at a time
                    self.send(data[:64], SERIAL_DATA)
                    data = data[64:]
            except Exception as e:
                print('nmea socket exception', e)
                self.nmea_socket.close()
                self.nmea_socket = False
        
        events = []
        while True:
            data = self.recv()
            if not data:
                break
            cmd, data = data[0], data[1:]
            if cmd == SERIAL_DATA:
                if self.lastdata == data:
                    print('duplicate data!!')
                    continue
                self.lastdata = data
                self.serial_out_count += len(data)
                if not self.nmea_socket:
                    if not self.open_nmea():
                        print('failed to send serial data')
                if self.nmea_socket:
                    self.nmea_socket.send(data)
                continue

            if len(data) < 4:
                print('invalid packet!!')
                continue

            key = '%02X%02X%02X%02X' % (data[0], data[1], data[2], data[3])
            count = data[4]
                
            if cmd == RF:
                key = 'rf' + key
            elif cmd == IR:
                key = 'ir' + key
                if lircd.LIRC_version:
                    print('received IR decoded from arduino, disable LIRC')
                    lircd.LIRC_version = 0 # disable lircd if we got ir from arduino
            elif cmd == GP:
                key = 'gpio_ext' + key
            else:
                print('unknown message')
                continue

            events.append((key, count))
        return events

    def open_nmea(self):
        self.nmea_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.nmea_socket.connect((self.config['host'], 20220))

    def flash(self, filename, c):
        global GPIO
        if not GPIO:
            return False

        self.resetpin = self.hatconfig['resetpin']

        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.resetpin, GPIO.OUT)
            GPIO.output(self.resetpin, 0)
        except Exception as e:
            print('failed to setup gpio reset pin for arduino', e)
            GPIO = False # prevent further tries
            return False

        command = 'avrdude -P ' + self.hatconfig['device'] + ' -u -p atmega328p -c linuxspi -U f:' + c + ':' + filename + ' -b 500000'

        ret = os.system(command)
        GPIO.output(self.resetpin, 1)
        GPIO.setup(self.resetpin, GPIO.IN)
        return not ret

    def verify(self, filename):
        return self.flash(filename, 'v')

    def write(self, filename):
        return self.flash(filename, 'w')

def arduino_process(pipe, config)
    a = arduino(config)
    while True:
        t0 = time.monotonic()
        events = a.poll()
        if events:
            pipe.send(events)
        baud_rate = a.getbaudrate()
        pipe.send(('baudrate', baud_rate))
        while True:
            try:
                msg = pipe.recv()
                if not msg:
                    break
                cmd, value = msg
            except Exception as e:
                print('pipe recv failed!!\n')
                return
            if cmd == 'baud':
                a.set_baud(value)
            elif cmd == 'buzzer':
                a.set_buzzer(*value)
            else:
                print('unhandled command', cmd)
        t1 = time.monotonic()
        dt = .03 - (t1-t0)
        if dt > 0:
            time.sleep(dt)      
    
def main():
    print('initializing arduino')
    config = {'arduino':{'device':'/dev/spidev0.1',
                         'resetpin':'gpio16'}}
    a = arduino(config)

    t0 = time.monotonic()

    while True:
        events = a.poll()
        if events:
            print(events)
        time.sleep(.01)
    
if __name__ == '__main__':
    main()
