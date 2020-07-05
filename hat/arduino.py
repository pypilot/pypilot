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

import os, sys, time, socket, errno
import json
import lircd

REWIND=0xab # reset state
READ_SERIAL = 0xb9 # read next serial byte
DISCARD_SERIAL = 0xd1 # once data read is verified
WRITE_DONE = 0xce # backup two bytes since verification failed
WRITE_SERIAL = 0xd6 # set state writing to serial
READ_DATA = 0xea # read next data byte
DISCARD_DATA = 0xa4 # validate read data
WRITE_DATA = 0xb6 # set state for writing data
OK = 0xc8 # returned from sucessful operations
END=0xe3 # returned if reading reaches end
ZERO = 0x85 # encode a 0 byte
INVALID=0x96

RF=0x01
IR=0x02
GP=0x03

SET_BACKLIGHT=0x16
SET_BUZZER=0x17
SET_BAUD=0x18

PACKET_LEN=6

try:
    import RPi.GPIO as GPIO
except:
    GPIO = False
    
class arduino(object):    
    def __init__(self, config):
        self.xfers=0
        self.spi = False
        self.nmea_socket = False
        self.nmea_connect_time = time.monotonic()
        self.pollt0 = [0, time.monotonic()]

        self.config = config
        self.hatconfig = False
        if 'hat' in config:
            hatconfig = config['hat']
            if hatconfig and 'arduino' in hatconfig:
                self.hatconfig = hatconfig['arduino']
        if not self.hatconfig:
            print('No hat config, arduino not found')

        if not 'nmea' in config:
            self.config['nmea'] = {'in': True, 'out': False, 'baud': 38400}

        self.lasttime=0

        self.sent_count = 0
        self.sent_start = time.monotonic()

        self.serial_in_count = 0
        self.serial_out_count = 0
        self.serial_time = self.sent_start + 2

        self.packetout_data = []
        self.packetin_data = []

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
                self.spi.max_speed_hz=1000000

        except Exception as e:
            print('failed to communicate with arduino', device, e)
            self.hatconfig = False

    def close(self, e):
        print('failed to read spi:', e)
        self.spi.close()
        self.spi = False

    def xfer(self, x):
        return self.spi.xfer([x])[0]

    def send(self, id, data):
        self.packetout_data += b'$' + bytes([id]) + bytes(data)

    def set_backlight(self, value, polarity):
        value = min(max(int(value), 0), 100)
        backlight = [value, polarity]
        self.send(SET_BACKLIGHT, backlight)

    def set_baud(self, baud):
        self.config['nmea']['baud'] = baud
        baud = int(baud/100)
        baud = [baud%128, baud//128]
        self.send(SET_BAUD, baud)

    def set_buzzer(self, duration, frequency):
        self.send(SET_BUZZER, (duration, frequency))

    def get_baud_rate(self):
        t = time.monotonic()
        dt = t - self.serial_time
        if dt < 10:
            return False
        self.serial_time = t
        rate_in = self.serial_in_count*10/dt
        rate_out = self.serial_out_count*10/dt
        self.serial_in_count = self.serial_out_count = 0
        return 'TX: %.1f  RX %.1f' % (rate_in, rate_out)

    def poll(self):
        if not self.spi:
            self.open()
            return []

        events = []
        serial_data =  []
        self.open_nmea()

        # don't exceed 90% of baud rate
        baud = self.config['nmea']['baud'] *.9
        while True:
            if self.nmea_socket:
                try:
                    b = self.nmea_socket.recv(40)
                    b = b''
                    self.socketdata += b
                    self.serial_in_count += len(b)
                    if len(self.socketdata) > 160:
                        print('overflow, dropping 64 bytes')
                        self.socketdata = self.socketdata[:64]
                        self.serial_in_count -= 64
                except Exception as e:
                    if e.args[0] is errno.EWOULDBLOCK:
                        pass
                    else:
                        print('nmea socket exception', e)
                        self.nmea_socket.close()
                        self.nmea_socket = False

            i = 0
            if self.socketdata:
                count, t0 = self.pollt0
                dt = time.monotonic() - t0
                rate = 10*(count)/dt
                if rate < baud:
                    if self.socketdata[0] and self.socketdata[0] < 128:
                        i = self.socketdata[0]
                        self.pollt0[0] += 1
                        self.socketdata = self.socketdata[1:]
            else:
                self.pollt0 = [0, time.monotonic()] # reset

            if not i and self.packetout_data:
                i = self.packetout_data[0]
                self.packetout_data = self.packetout_data[1:]

            o = self.xfer(i)
            if not i and not o:
                break

            if o > 127:
                o&=0x7f
                self.packetin_data.append(o)
            elif o:
                serial_data.append(o)

        # now read packet data
        while len(self.packetin_data) >= PACKET_LEN+3:
            if self.packetin_data[0] != ord('$'):
                self.packetin_data = self.packetin_data[1:]
                continue

            cmd = self.packetin_data[1]
            d = self.packetin_data[2:PACKET_LEN+2]
            parity = self.packetin_data[PACKET_LEN+2];
            p = 0
            for x in d:
                p ^= x

            # spi interrupt collides with pin change interrupt, so sometimes
            # bytes are lost when rf is receiving causing parity failure
            # drop invalid packets
            if p != parity:
                self.packetin_data = self.packetin_data[1:]
                continue

            self.packetin_data = self.packetin_data[3+PACKET_LEN:]

            key = '%02X%02X%02X%02X' % (d[0], d[1], d[2], d[3])
            count = d[4]
                
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
                print('unknown message', cmd, d)
                continue

            events.append((key, count))

        if self.nmea_socket and serial_data:# and self.config['nmea']['out']:
            #print('nmea>', bytes(serial_data))
            self.nmea_socket.send(bytes(serial_data))
            self.serial_out_count += len(serial_data)
            #self.open_nmea()
            #print('nmea', self.serial_in_count, self.serial_out_count, self.serial_in_count - self.serial_out_count)
            
        return events

    def open_nmea(self):
        if self.nmea_socket:
            return
        if time.monotonic() < self.nmea_connect_time:
            return

        self.nmea_connect_time += 4
        nmea = self.config['nmea']
        if not nmea['in'] and not nmea['out']:
            return
        try:
            self.socketdata = b''
            self.nmea_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.nmea_socket.setblocking(0)
            print('connect nmea', self.config['host'])
            self.nmea_socket.connect((self.config['host'], 20220))
        except OSError as e:
            if e.args[0] is errno.EINPROGRESS:
                return True
        except Exception as e:
            print('exception', e)
            self.nmea_socket = False

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

def arduino_process(pipe, config):
    a = arduino(config)
    while True:
        t0 = time.monotonic()
        events = a.poll()
        if events:
            pipe.send(events)

        baud_rate = a.get_baud_rate()
        if baud_rate:
            a.set_baud(38400)
            if a.nmea_socket:
                pipe.send([('baudrate', baud_rate)])
            else:
                pipe.send([('baudrate', 'ERROR: no connection to server for nmea')])

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
            elif cmd == 'backlight':
                a.set_backlight(*value)
            elif cmd == 'buzzer':
                a.set_buzzer(*value)
            else:
                print('unhandled command', cmd)
        t1 = time.monotonic()
        # max period to handle 38400 with 192 byte buffers is (192*10) / 38400 = 0.05
        # for now use 0.025, eventually dynamic depending on baud?
        dt = .025 - (t1-t0)
        if dt > 0:
            time.sleep(dt)      
    
def main():
    print('initializing arduino')
    config = {'host':'localhost','hat':{'arduino':{'device':'/dev/spidev0.1',
                                                   'resetpin':'gpio16'}}}
    a = arduino(config)

    dt = 0
    lt = 0
    while True:
        t0 = time.monotonic()
        events = a.poll()
        if events:
            print(events, dt, t0-lt)
            lt = t0
        baud_rate = a.get_baud_rate()
        if baud_rate:
            print('baud rate', baud_rate)
        t1 = time.monotonic()
        dt = t1 - t0
        #print('dt', dt)
        time.sleep(.02)
    
if __name__ == '__main__':
    main()
