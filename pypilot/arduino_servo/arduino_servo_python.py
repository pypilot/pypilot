#!/usr/bin/env python
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import os, select, time

from servo import *
from crc import crc8

class ArduinoServo:
    sync_bytes = [0xe7, 0xf9, 0xc7, 0x1e, 0xa7, 0x19, 0x1c, 0xb3]

    def __init__(self, fd):
        self.fd = fd
        self.in_sync = self.out_sync = 0
        self.in_sync_count = 0
        self.in_buf = []

        self.poller = select.poll()
        self.poller.register(self.fd, select.POLLIN)

        self.max_current_value = 0

        self.voltage = self.current = False
        self.flags = 0

    def initialize(self):
        cnt = 0
        data = False
        while self.flags & ServoFlags.OVERCURRENT or \
          not self.flags & ServoFlags.SYNC:
            self.stop()
            if self.poll():
                data = True

            time.sleep(.001)
            cnt+=1
            if cnt == 400 and not data:
                return False
            if cnt == 1000:
                return False
        return True

    def command(self, command):
        command = min(max(command, -1), 1)
        self.raw_command((command+1)*1000)

    def stop(self):
        self.raw_command(0x5342)

    def poll(self):
        if len(self.in_buf) < 3:
            if not self.poller.poll(0):
                return False
            try:
                c = os.read(self.fd, 12)
            except:
                return -1
            self.in_buf += map(ord, c)
            if len(self.in_buf) < 3:
                return 0

        ret = 0
        while len(self.in_buf) >= 3:
            code = [ArduinoServo.sync_bytes[self.in_sync]] + self.in_buf[:2]
            crc = crc8(code)
            
	    #print 'got code', code, self.in_buf
            if crc == self.in_buf[2]:
                if self.in_sync_count == 2:
                    value = self.in_buf[0] + (self.in_buf[1]<<8)
                    if self.in_sync > 0:
                        self.current = value * 1.1 / .05 / 65536
                        ret |= ServoTelemetry.CURRENT
                    else:
                        self.voltage = (value >> 4) * 1.1 * 10560 / 560 / 4096
                        self.flags = value & 0xf
                        ret |= ServoTelemetry.VOLTAGE | ServoTelemetry.FLAGS

                self.in_sync+=1
                if self.in_sync == len(ArduinoServo.sync_bytes):
                    self.in_sync = 0
                    if self.in_sync_count < 2:
                        self.in_sync_count+=1

                self.in_buf = self.in_buf[3:]
            else:
                self.in_sync = self.in_sync_count = 0
                self.in_buf = self.in_buf[1:]

        return ret

    def fault(self):
        return self.flags & (ServoFlags.FAULTPIN | ServoFlags.OVERCURRENT) != 0

    def max_current(self, value):
        self.max_current_value = min(10, value)

    def send_value(self, value):
        value = int(value)
        code = [ArduinoServo.sync_bytes[self.out_sync], value&0xff, (value>>8)&0xff]
        b = '%c%c%c' % (code[1], code[2], crc8(code))

        os.write(self.fd, b)
        self.out_sync += 1
        
    def raw_command(self, command):
        if self.out_sync == 0:
            self.send_value(self.max_current_value*65536.0*.05/1.1)
        self.send_value(command)
        if self.out_sync == len(ArduinoServo.sync_bytes):
            self.out_sync = 0
