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
from pypilot.client import udp_control_port

RF=0x01
IR=0x02
GP=0x03
VOLTAGE=0x04
ANALOG=0x05
VERSION=0x0a

SET_BACKLIGHT=0x16
SET_BUZZER=0x17
SET_BAUD=0x18
SET_ADC_CHANNELS=0x19
GET_VERSION=0x1b

PACKET_LEN=6

try:
    import RPi.GPIO as GPIO
except:
    GPIO = False

def update_firmware(config):
    if not 'hat' in config:
        return

    hatconfig = config['hat']
    if not 'arduino' in hatconfig:
        return

    arduinoconfig = hatconfig['arduino']

    device = arduinoconfig.get('device')
    if not device:
        return

    if not device.startswith('/dev/spidev'):
        print('only support spi devices to update firmware')
        return

    if not arduinoconfig.get('resetpin'):
        print('the reset pin is unknown, cannot update firmware')
        return

    # determine firmware version we have on disk
    firmware = False
    try:
        path = os.getenv('HOME') + '/.pypilot/firmware'
        for filename in os.listdir(path):
            print("FILE", filename)
            if filename.startswith('hat_') and filename.endswith('.hex'):
                try:
                    config['arduino_firmware_version_available'] = float(filename[4:-4])
                    firmware = path+os.path.sep + filename
                    break
                except Exception as e:
                    print("invalid firmware filename found", filename, e)
        else:
            print('no firmware file found in', path)
            return
                
    except Exception as e:
        print('failed to find firmware', e)
        return

    if not 'arduino_firmware_version' in config:
        print('cannot update firmware until version is known')
        return

    if config['arduino_firmware_version_available'] <= config['arduino_firmware_version']:
        print('firmware up to date')
        return

    if os.system('which avrdude') != 0:
        print('cannot update firmware without avrdude')
        return

    print('found firmware update', firmware)
    time.sleep(1) # needed?

    resetpin = str(arduinoconfig['resetpin'])
    import tempfile, subprocess
    temp = tempfile.mkstemp()
    p=subprocess.Popen(['avrdude', '-?'], stderr=temp[0], close_fds=True)
    p.wait()

    f = os.fdopen(temp[0], 'r')
    f.seek(0)
    release = ''
    while not 'version' in release:
        release = f.readline().rstrip()
    f.close()
    avrdude_version = release.split('version')[1]
    print('avrdude version', avrdude_version)
    old_avrdude = avrdude_version[1].split()[0].startswith('6')
    if old_avrdude:
        print('detected old avrdude')
        os.system('echo ' + resetpin + ' > /sys/class/gpio/export')
        os.system('echo out > /sys/class/gpio/gpio' + resetpin + '/direction')
    
    def flash(filename, c):
        if old_avrdude:
            command = 'sudo avrdude -P ' + device + ' -u -p atmega328p -c linuxspi -U f:' + c + ':' + filename + ' -b 1000000'
        else:
            command = 'sudo avrdude -P ' + device + ':' + '/dev/gpiochip0:' + resetpin + ' -u -p atmega328p -c linuxspi -U f:' + c + ':' + filename + ' -b 1000000'
        print('flash cmd', command)
        ret = os.system(command)             
        return not ret
    
    def verify(filename):
        print('verifying', filename)
        return flash(filename, 'v')
    
    def write(filename):
        print('writing', filename)
        return flash(filename, 'w')

    # try to verify twice because sometimes this fails
    if verify(firmware) or verify(firmware):
        print('firmware already up to date!', firmware)
        config['arduino_firmware_version'] = config['arduino_firmware_version_available']
    elif not write(firmware):
        print('failed to write firmware', firmware)
    elif not verify(firmware):
        print('failed to verify or upload', firmware)
    else:
        print('success updating hat firmware!', firmware)

    if old_avrdude:
        os.system('echo in > /sys/class/gpio/gpio' + resetpin + '/direction')
    
    
class arduino(object):    
    def __init__(self, config):
        self.spi = False
        self.nmea_socket = False
        self.nmea_connect_time = time.monotonic()
        self.pollt0 = [0, time.monotonic()]
        self.config = config
        self.enabled = False

        if 'arduino.debug' in config and config['arduino.debug']:
            self.debug = print
        else:
            self.debug = lambda *args : None
            
        self.hatconfig = False
        self.backlight_polarity = False
        if 'hat' in config:
            hatconfig = config['hat']
            if hatconfig and 'arduino' in hatconfig:
                self.hatconfig = hatconfig['arduino']
            if hatconfig and 'lcd' in hatconfig and 'driver' in hatconfig['lcd']:
                self.backlight_polarity = hatconfig['lcd']['driver'] == 'nokia5110'
        if not self.hatconfig:
            print('No hat config, arduino not found')

        self.lasttime=0

        self.sent_count = 0
        self.sent_start = time.monotonic()

        self.socketdata = b''
        self.serial_in_count = 0
        self.serial_out_count = 0
        self.serial_time = self.sent_start + 2
        self.udp_socket = None

        self.packetout_data = []
        self.packetin_data = []
        self.version = False
        self.get_version_time = time.monotonic()

        self.control_socket = False
        self.set_actions(config['actions'])

    def open(self):
        if not self.hatconfig:
            return

        device = self.hatconfig['device']
        if not device:
            return

        try:
            port, slave = int(device[11]), int(device[13])
            print('arduino on spidev%d.%d' % (port, slave))

            if False:
                import spidev
                self.spi = spidev.SpiDev()
                self.spi.open(port, slave)
                self.spi.max_speed_hz=100000
            else:
                from pypilot.hat.spireader import spireader
                self.spi = spireader.spireader(10, 5)
                if self.spi.open(port, slave, 500000) == -1:
                    self.close()

        except Exception as e:
            print('failed to communicate with arduino', device, e)
            self.hatconfig = False
            self.spi = False
            return

        if 'lcd' in self.config and 'backlight' in self.config['lcd']:
            self.set_backlight(self.config['lcd']['backlight'])
        self.set_baud(self.config['arduino.nmea.baud'])
        try:
            self.set_adc_channels(len(self.config['arduino.adc_channels']))
        except:
            pass

    def close(self, e):
        print('failed to read spi:', e)
        self.spi.close()
        self.spi = False

    def send(self, id, data=[]):
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
        backlight = [value, self.backlight_polarity]
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

    def set_adc_channels(self, value):
        value = min(len(value), 3)
        self.send(SET_ADC_CHANNELS, [value])

    def set_buzzer(self, pitch, pulse, duration):
        duration = int(round(min(max(duration, 0), 2)*100))
        mode = pitch | (pulse << 4)
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
        t0 = time.monotonic()
        if not self.spi:
            self.open()
            return []

        if not self.version and t0-self.get_version_time > 10:
            self.send(GET_VERSION)
            self.get_version_time = t0+170
        
        events = []
        serial_data =  []
        self.open_nmea()
        # don't exceed 90% of baud rate
        baud = int(self.config['arduino.nmea.baud']) *.9
        t1 = time.monotonic()
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

            ta = time.monotonic()
            o = self.spi.xfer(i, not i and len(self.packetin_data) < PACKET_LEN+3)
            tb = time.monotonic()
            
            if not i and not o:
                break

            if o > 127:
                o&=0x7f
                self.packetin_data.append(o)
            elif o:
                serial_data.append(o)

        t2 = time.monotonic()
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
                #print("key", key, count, time.time()-1681305360)
            elif cmd == IR:
                if not self.config['arduino.ir']:
                    continue
                key = 'ir' + key
            elif cmd == GP:
                key = 'gpio_ext%02X' % d[0]
            elif cmd == VOLTAGE:
                vcc = (d[0] + (d[1]<<7))/1000.0
                vin = (d[2] + (d[3]<<7))/1000.0
                events.append(['voltage', {'vcc': vcc, 'vin': vin}])
                continue
            elif cmd == ANALOG:
                adc = []
                for i in range(3):
                    adc.append(d[2*i] + (d[2*i+1]<<7))
                events.append(['analog', adc])
                continue
            elif cmd == VERSION:
                self.version = float('%d.%d' % (d[0], d[1]))
                print('hat firmware version', self.version)
                events.append(['version', self.version])
                continue
            else:
                print('unknown message', cmd, d)
                continue

            if not self.enabled and key in self.manual_control_keys:
                self.send_manual(self.manual_control_keys[key], count)
                if count != 1:
                    continue
                count = -1 # report press for programming
            events.append([key, count])

        t3 = time.monotonic()
        if serial_data:
            self.debug('nmea>', bytes(serial_data))
            if self.nmea_socket and self.config['arduino.nmea.in']:
                try:
                    self.nmea_socket.send(bytes(serial_data))
                except Exception as e:
                    print('nmea socket exception sending', e)
                    self.nmea_socket.close()
                    self.nmea_socket = False

            self.serial_out_count += len(serial_data)
        t4 = time.monotonic()
        #print('times', t1-t0, t2-t1, t3-t2, t4-t3)
        return events

    def set_actions(self, actions):
        self.manual_control_keys = {}

        manual_keys = {'+1': -.8,
                       '-1':  .8}
        for action, keys in actions.items():
            if action in manual_keys:
                for key in keys:
                    self.manual_control_keys[key] = manual_keys[action]

    def send_manual(self, command, count):
        command = str(command)+'\n' if count else "0\n"
        try:
            if not self.udp_socket:
                self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            count = self.udp_socket.sendto(command.encode(), ('127.0.0.1', udp_control_port))
        except Exception as e:
            print('udp socket failed to send', e)
            if self.udp_socket:
                self.udp_socket.close()
                self.udp_socket = None
            count = 0
        if count != len(command):
            print('udp socket failed count', count, len(command))

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
                    # send a "special" pypilot nmea message to
                    # enable rebroadcasting from this socket connection
                    self.nmea_socket.send(bytes('$PYPBS*48\r\n', 'utf-8'))
                except Exception as e:
                    print('nmea socket exception sending', e)
                    self.nmea_socket.close()
                    self.nmea_socket = False
            else:
                print('os error', e)
                self.nmea_socket = False
        except Exception as e:
            print('exception', e)
            self.nmea_socket = False
        except:
            print('exception? 975m33')

def arduino_process(pipe, config):
    start = time.monotonic()
    a = arduino(config)
    period = .05
    periodtime = 0
    while True:
        t0 = time.monotonic()
        events = a.poll()
        #print('events', events)
        t1 = time.monotonic()
        baud_rate = a.get_baud_rate()
        if baud_rate:
            a.debug('nmea baud rate', baud_rate)
            if a.nmea_socket:
                events.append(['baudrate', baud_rate])
            else:
                events.append(['baudrate', 'ERROR: no connection to server for nmea'])

        if events and t0 - start > 2:
            pipe.send(events)
            if events[0][0] != 'voltage':
                period = 0
                periodtime = t0
        elif t0 - periodtime > 5:
            period = .1

        while True:
            try:
                msg = pipe.recv()
                if not msg:
                    break
                name, value = msg
            except Exception as e:
                print('pipe recv failed!!', e, msg)
                return

            config[name] = value
            if name == 'ap.enabled':
                a.enabled = value
            elif name == 'backlight':
                a.set_backlight(value)
            elif name == 'buzzer':
                a.set_buzzer(*value)
            elif name == 'arduino.nmea.baud':
                a.set_baud(value)
            elif name == 'arduino.adc_channels':
                a.set_adc_channels(value)
            elif name == 'actions':
                a.set_actions(value)

        t2 = time.monotonic()
        # max period to handle 38400 with 192 byte buffers is (192*10) / 38400 = 0.05
        # for now use 0.025, eventually dynamic depending on baud?
        dt = period - (t2-t0)
        #print('arduino times', period, dt, t1-t0, t2-t1, events)
        if dt > 0:
            time.sleep(dt)
    
def main():
    print('initializing arduino')
    config = {'host':'localhost','hat':{'arduino':{'device':'/dev/spidev0.1',
                                                   'resetpin':'26'}},
              'actions': {},
              'arduino.nmea.baud': 4800,
              'arduino.nmea.in': False,
              'arduino.nmea.out': False,
              'arduino.ir': True,
              'arduino.debug': True,
              'arduino.adc_channels': []}

    a = arduino(config)
    lt = 0
    while True:
        t0 = time.monotonic()
        events = a.poll()
        if events:
            print(events, t0, t0-lt)
            lt = t0
        baud_rate = a.get_baud_rate()
        if baud_rate:
            print('baud rate', baud_rate)
    
if __name__ == '__main__':
    main()
