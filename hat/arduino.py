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

import os, sys, time, socket, errno, select

RF=0x01
IR=0x02
GP=0x03
VOLTAGE=0x04

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
        self.spi = False
        self.nmea_socket = False
        self.nmea_connect_time = time.monotonic()
        self.pollt0 = [0, time.monotonic()]

        self.config = config

        if 'arduino.debug' in config and config['arduino.debug']:
            self.debug = print
        else:
            self.debug = lambda *args : None
            
        self.hatconfig = False
        if 'hat' in config:
            hatconfig = config['hat']
            if hatconfig and 'arduino' in hatconfig:
                self.hatconfig = hatconfig['arduino']
        if not self.hatconfig:
            print('No hat config, arduino not found')

        self.lasttime=0

        self.sent_count = 0
        self.sent_start = time.monotonic()

        self.socketdata = b''
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
                self.resetpin = self.hatconfig['resetpin']

                try:
                    GPIO.setmode(GPIO.BCM)
                    GPIO.setup(int(self.resetpin), GPIO.IN, pull_up_down=GPIO.PUD_UP)
                    pass
                except Exception as e:
                    print('arduino failed to set reset pin high', e)
                    #return
                    
                port, slave = int(device[11]), int(device[13])
                print('arduino on spidev%d.%d' % (port, slave))
                import spidev
                self.spi = spidev.SpiDev()
                self.spi.open(port, slave)
                self.spi.max_speed_hz=100000

                if 'lcd' in self.config:
                    self.set_backlight(self.config['lcd']['backlight'])
                self.set_baud(self.config['arduino.nmea.baud'])

        except Exception as e:
            print('failed to communicate with arduino', device, e)
            self.hatconfig = False
            self.spi = False

    def close(self, e):
        print('failed to read spi:', e)
        self.spi.close()
        self.spi = False

    def xfer(self, x):
        #time.sleep(.0001)
        return self.spi.xfer([x])[0]

    def send(self, id, data):
        p = id
        self.packetout_data += bytes([ord('$') | 0x80, id | 0x80])
        for i in range(PACKET_LEN-1):
            if i < len(data):
                d = data[i]
            else:
                d = 0
            p ^= d
            self.packetout_data += bytes([d | 0x80])
        self.packetout_data += bytes([p | 0x80])

    def set_backlight(self, value):        
        value = min(max(int(value*3), 0), 120)
        polarity = self.hatconfig['device'] == 'nokia5110'
        backlight = [value, polarity]
        self.send(SET_BACKLIGHT, backlight)

    def set_baud(self, baud):
        try:
            baud = int(baud)
        except:
            baud = 38400

        # 0, 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600
        if baud == 4800:
            d = [5]
        elif baud == 38400:
            d = [8]
        else:
            print('invalid baud', baud)
            d = [8]
        self.debug('nmea set baud', d)
        self.send(SET_BAUD, d)

    def set_buzzer(self, mode, duration):
        duration = int(min(max(duration, 0), 2)*100)
        self.send(SET_BUZZER, (mode, duration))

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
        baud = int(self.config['arduino.nmea.baud']) *.9
        while True:
            if self.nmea_socket and len(self.socketdata) < 100 and self.nmea_socket_poller.poll(0):
                try:
                    b = self.nmea_socket.recv(40)
                    if self.config['arduino.nmea.out']:
                        self.socketdata += b
                        self.serial_in_count += len(b)
                except Exception as e:
                    if e.args[0] is errno.EWOULDBLOCK:
                        pass
                    else:
                        print('nmea socket exception reading', e)
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
                #print('send %c %x' %(i,i))
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
            # (maybe even impossible at 100khz and below)
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
                if not self.config['arduino.ir']:
                    continue
                key = 'ir' + key
            elif cmd == GP:
                key = 'gpio_ext' + key
            elif cmd == VOLTAGE:
                vcc = (d[0] + (d[1]<<7))/1000.0
                vin = (d[2] + (d[3]<<7))/1000.0
                events.append(['voltage', {'vcc': vcc, 'vin': vin}])
                continue
            else:
                print('unknown message', cmd, d)
                continue

            events.append([key, count])

        if serial_data:
            self.debug('nmea>', bytes(serial_data))
            self.debug('nmea data', self.serial_in_count, 'bytes in',
                       self.serial_out_count, 'bytes_out')
            if self.nmea_socket and self.config['arduino.nmea.in']:
                try:
                    self.nmea_socket.send(bytes(serial_data))
                except Exception as e:
                    print('nmea socket exception sending', e)
                    self.nmea_socket.close()
                    self.nmea_socket = False

            self.serial_out_count += len(serial_data)
            
        return events

    def open_nmea(self):
        c = self.config
        if not c['arduino.nmea.in'] and not c['arduino.nmea.out']:
            if self.nmea_socket:
                self.nmea_socket.close()
                self.nmea_socket = False
            return
        
        if self.nmea_socket:
            return
        if time.monotonic() < self.nmea_connect_time:
            return

        self.nmea_connect_time += 8
        self.socketdata = b''
        try:
            self.nmea_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.nmea_socket.setblocking(0)
            self.nmea_socket.connect((self.config['host'], 20220))
        except OSError as e:
            if e.args[0] is errno.EINPROGRESS:
                self.nmea_socket_poller = select.poll()
                self.nmea_socket_poller.register(self.nmea_socket, select.POLLIN)
                try:
                    self.nmea_socket.send(bytes('$PYPBS*48\r\n', 'utf-8'))
                except Exception as e:
                    print('nmea socket exception sending', e)
                    self.nmea_socket.close()
                    self.nmea_socket = False
            else:
                print('os error', e)
        except Exception as e:
            print('exception', e)
            self.nmea_socket = False
        except:
            print('MOOOOOOOOOOOOOOORE')

    def flash(self, filename, c):
        global GPIO
        if not GPIO:
            return False

        self.resetpin = self.hatconfig['resetpin']

        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(int(self.resetpin), GPIO.OUT)
            GPIO.output(int(self.resetpin), 0)
        except Exception as e:
            print('failed to setup gpio reset pin for arduino', e)
            GPIO = False # prevent further tries
            return False

        command = 'avrdude -P ' + self.hatconfig['device'] + ' -u -p atmega328p -c linuxspi -U f:' + c + ':' + filename + ' -b 500000'
        print('cmd', command)
        ret = os.system(command)
        GPIO.output(int(self.resetpin), 1)
        GPIO.setup(int(self.resetpin), GPIO.IN)
        return not ret

    def verify(self, filename):
        return self.flash(filename, 'v')

    def write(self, filename):
        return self.flash(filename, 'w')

def arduino_process(pipe, config):
    a = arduino(config)
    period = .05
    periodtime = 0
    while True:
        t0 = time.monotonic()
        events = a.poll()
        t1 = time.monotonic()
        baud_rate = a.get_baud_rate()
        if baud_rate:
            #print('baud', baud_rate)
            if a.nmea_socket:
                events.append(['baudrate', baud_rate])
            else:
                events.append(['baudrate', 'ERROR: no connection to server for nmea'])

        if events:
            pipe.send(events)
            #print('events', events, time.monotonic())
            period = .05
            periodtime = t0
        elif periodtime - t0 > 5:
            period = .2

        while True:
            try:
                msg = pipe.recv()
                if not msg:
                    break
                name, value = msg
            except Exception as e:
                print('pipe recv failed!!\n')
                return

            config[name] = value
            if name == 'backlight':
                a.set_backlight(value)
            elif name == 'buzzer':
                a.set_buzzer(*value)
            elif name == 'arduino.nmea.baud':
                a.set_baud(value)
        t2 = time.monotonic()
        # max period to handle 38400 with 192 byte buffers is (192*10) / 38400 = 0.05
        # for now use 0.025, eventually dynamic depending on baud?
        dt = period - (t2-t0)
        #print('arduino times', period, dt, t1-t0, t2-t1)
        if dt > 0:
            time.sleep(dt)      
    
def main():
    print('initializing arduino')
    config = {'host':'localhost','hat':{'arduino':{'device':'/dev/spidev0.1',
                                                   'resetpin':'16'}},
              'arduino.nmea.baud': 4800,
              'arduino.nmea.in': False,
              'arduino.nmea.out': False,
              'arduino.ir': True,
              'arduino.debug': True}

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
        time.sleep(.04)
    
if __name__ == '__main__':
    main()
