#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# A separate process manages a possible arduino to receive IR/RF
# The arduino also controls the backlight and provides additional
# pins for more functionallity (tack button, 
# spi port.

from __future__ import print_function
import os, sys, time, json
import spidev
from signalk.client import SignalKClient

def flash(device, filename, c):
    # detect if this board supports arduino
    # echo 16 > /sys/class/gpio/export || echo ok
    # echo out > /sys/class/gpio/gpio16/direction

    command = 'avrdude -P ' + device + '-u -p atmega328p -c linuxspi -U f:' + c + ':' + filename + ' -b 500000'
    ret = os.system(command)
    # echo in > /sys/class/gpio/gpio16/direction

def verify(device, filename):
    return flash(device, filename, 'v')

def write(device, filename):
    return flash(device, filename, 'w')

class arduino(object):
    def __init__(self, config):
        self.spi = False

        self.config = config
        if not config:
            print('No hat config, arduino not found')

    def open(self):
        if not self.config:
            return

        device = self.config['device']
        try:
            if device.startswith('/dev/spidev'):
                # update flash if needed
                filename = '.pypilot/pypilothat.hex'
                if not verify(device, filename) and not write(device, filename):
                    return
                
                port, slave = int(device[6]), int(device[8])
                self.spi = spidev.SpiDev()
                self.spi.open(port, slave)
                self.spi.max_speed_hz=5000
        except Exception as e:
            print('failed to communicate with arduino', device, e)
            exit(1)

    def close(self, e):
        print('failed to read spi', e)
        self.spi.close()
        self.spi = False
        
    def poll(self):
        if not self.spi:
            self.open()
            return

    def read(self):
        if not self.spi:
            return
    
        try:
            x = self.spi.xfer([0, 0, 0, 0])
        except Exception as e:
            self.close(e)
            return

        for i in range(4):
            crc = crc8(x[:3])
            if crc == x[3]:
                break

            if i == 3:
                print('failed to syncronize to correct crc')
                self.close(e)
                return
                
            try:
                y = self.spi.xfer([0])
            except Exception as e:
                self.close(e)
                return
            x = x[1:] + [y]
                
        command = x[0]
        if command == 0:
            return # no more events

        if command == 1 or command == 2:
            key = 'spikey'+x[1]
            down = command == 1
            count = x[2]
            return key, down, count
