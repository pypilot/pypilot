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

try:
    import RPi.GPIO as GPIO
except:
    GPIO = False

SET_BACKLIGHT = 0xa1
RF, IR, AR = 0xa1, 0x06, 0x9c;
    
class arduino(object):
    def __init__(self, config):
        self.spi = False
        self.backlight = -1
        self.packet_size = 6
        self.next_packet = [0] * (self.packet_size - 1)

        self.events = []

        if config and 'arduino' in config:
            self.config = config['arduino']
        else:
            self.config = False

            # hack
        if True:
            self.config = {"device":"/dev/spidev0.1", "resetpin":16}
            
        if not self.config:
            print('No hat config, arduino not found')

    def open(self):
        if not self.config:
            return

        device = self.config['device']
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
                        #self.config['device'] = False # prevent retry
                        #return

                port, slave = int(device[11]), int(device[13])
                print('arduino on spidev%d.%d' % (port, slave))
                import spidev
                self.spi = spidev.SpiDev()
                self.spi.open(port, slave)
                self.spi.max_speed_hz=5000
        except Exception as e:
            print('failed to communicate with arduino', device, e)
            exit(1)

    def close(self, e):
        print('failed to read spi:', e)
        self.spi.close()
        self.spi = False

    def packet(self, d):
        if len(d) != self.packet_size - 1:
            raise 'invalid packet'
        return d + [crc.crc8(d)]

    def set_backlight(self, value):
        if (self.backlight != value):
            self.backlight = min(max(int(value), 0), 200)
            self.next_packet = [SET_BACKLIGHT, 0, 0, 0, self.backlight]

    def poll(self):
        if not self.spi:
            self.open()
            return

        while self.read_packet():
            pass

    def read_packet(self):
        s = self.packet_size-1
        try:
            x = self.spi.xfer(self.packet(self.next_packet))
            self.next_packet = [0] * s

        except Exception as e:
            self.close(e)
            return False

        for i in range(s+1):
            if not any(x):
                return False

            ck = crc.crc8(x[:s])
            if ck == x[s]:
                break

            if i == s:
                print('failed to syncronize spi packet', ck, x[s])
                return False
                
            try:
                y = self.spi.xfer([0])
            except Exception as e:
                self.close(e)
                return False
            x = x[1:] + y
        #print('spi packet', x)

        command = x[0]
        if x[0] == RF:
            key = 'rf%02X%02X%02X' % (x[1], x[2], x[3])
            count = x[4]
        elif x[0] == IR:
            key = 'ir%02X%02X%02X' % (x[1], x[2], x[3])
            count = x[4]
        elif x[0] == AR:
            key = 'gpio' % x[1]
            count = x[4]
        else:
            return False
        self.events.append((key, count))

    def flash(self, filename, c):
        global GPIO
        if not GPIO:
            return False

        self.resetpin = self.config['resetpin']

        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.resetpin, GPIO.OUT)
            GPIO.output(self.resetpin, 0)
        except Exception as e:
            print('failed to setup gpio reset pin for arduino', e)
            GPIO = False # prevent further tries
            return False

        command = 'avrdude -P ' + self.config['device'] + ' -u -p atmega328p -c linuxspi -U f:' + c + ':' + filename + ' -b 500000'

        ret = os.system(command)
        GPIO.output(self.resetpin, 1)
        GPIO.setup(self.resetpin, GPIO.IN)
        return not ret

    def verify(self, filename):
        return self.flash(filename, 'v')

    def write(self, filename):
        return self.flash(filename, 'w')

def main():
    print('initializing arduino')
    c = {'arduino':{'device':'/dev/spidev0.1',
                           'resetpin':'gpio16'}}
    ar = arduino(c)

    t0 = time.monotonic()

    while True:
        ar.poll()
        time.sleep(.01)
    
if __name__ == '__main__':
    main()
