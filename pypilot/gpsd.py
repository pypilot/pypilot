#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from datetime import datetime
import multiprocessing, time, socket, select
from nonblockingpipe import NonBlockingPipe
from bufferedsocket import LineBufferedNonBlockingSocket
from values import *
import serialprobe, pyjson

def gps_json_loads(line):
    try:
        return pyjson.loads(line)
    except:
        pass
    act = '"activated"'
    i = line.index(act)
    j = line.index('Z', i)
    line = line[:i]+act+':"'+line[i+12:j+1]+'"'+line[j+1:]
    return pyjson.loads(line)

class gpsProcess(multiprocessing.Process):
    def __init__(self):
        # split pipe ends
        self.gpsd_failed_connect = False
        self.pipe, pipe = NonBlockingPipe('gps pipe', True)
        super(gpsProcess, self).__init__(target=self.gps_process, args=(pipe,), daemon=True)
        
    def connect(self):
        time.sleep(2)
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect(('127.0.0.1', 2947))
            self.poller.register(sock, select.POLLIN)
            sock.settimeout(0)
            sock.send('?WATCH={"enable":true,"json":true};'.encode())
            self.gpsd_socket = LineBufferedNonBlockingSocket(sock, 'gpsd')
            self.gpsconnecttime = time.monotonic()
            self.devices = []
            print(_('gpsd connected'))
        #except socket.error:
        except ConnectionRefusedError:
            if not self.gpsd_failed_connect:
                print(_('gpsd failed to connect'))
                self.gpsd_failed_connect = True
            self.gpsd_socket = False
            time.sleep(30)
        except Exception as e:
            self.gpsd_socket = False
            print(_('exception connecting to gps'), e)
            time.sleep(60)

    def disconnect(self):
        print(_('gpsd disconnected'))
        self.poller.unregister(self.gpsd_socket.socket)
        self.gpsd_socket.close()
        self.gpsd_socket = False
        self.devices = []

    def read_pipe(self, pipe):
        while True:
            device = pipe.recv()
            if not device:
                break
            if self.gpsd_socket and not self.devices: # only probe if there are no gpsd devices
                print('gpsd PROBING...', device)                    
                if not os.system('timeout -s KILL -t 30 gpsctl -f ' + device + ' 2> /dev/null'):
                    print(_('gpsd PROBE success'), device)
                    # probe was success
                    os.environ['GPSD_SOCKET'] = '/tmp/gpsd.sock'
                    os.environ['GPSD_OPTIONS'] = '-N -G -F /tmp/gpsd.sock' # should not run gpsd..
                    realpath = os.path.realpath(device)
                    os.system('gpsdctl add ' + realpath)
                    self.devices = [device]
                else:
                    print(_('gpsd probe failed'))
            # always reply with devices when asked to probe
            print('GPSD send devices', self.devices)
            pipe.send({'devices': self.devices})

    def parse_gpsd(self, msg, pipe):
        if not 'class' in msg:
            return False# unrecognized

        ret = False
        cls = msg['class']
        if cls  == 'DEVICES':
            self.devices = []
            for dev in msg['devices']:
                self.devices.append(dev['path'])
            ret = True
        elif cls == 'DEVICE':
            device = msg['path']
            if msg['activated']:
                if not device in self.devices:
                    self.devices.append(device)
                    ret = True
            else:
                print(_('gpsd deactivated'), device, self.devices)
                if device in self.devices:
                    self.devices.remove(device)
                    ret = True
        elif cls == 'TPV':
            if msg['mode'] == 3:
                fix = {'speed': 0}
                for key in ['track', 'speed', 'lat', 'lon', 'device', 'climb']:
                    if key in msg:
                        fix[key] = msg[key]
                if 'altHAE' in msg:
                    fix['alt'] = msg['altHAE']
                if 'time' in msg:
                    try:
                        ts = time.strptime(msg['time'], '%Y-%m-%dT%H:%M:%S.%f%z')
                        fix['timestamp'] = time.mktime(ts)
                    except:
                        pass

                fix['speed'] *= 1.944 # knots
                device = msg['device']
                if self.baud_boot_device_hint != device:
                    self.write_baud_boot_hint(device)
                if not device in self.devices:
                    self.devices.append(device)
                    ret = True
                pipe.send(fix, False)
        return ret

    def write_baud_boot_hint(self, device):
        self.baud_boot_device_hint = device
        try:
            stty=os.popen('sudo stty -F ' + device)
            line = stty.readline()
            stty.close()
            speed = line.index('speed')
            baud = line.index('baud')
            bps = int(line[speed+6:baud-1])
            f = open(os.getenv('HOME') + '/.pypilot/gpsd_baud_hint', 'w')
            f.write(str(bps))
            f.close()
        except Exception as e:
            print(_('gpsd failed to determine serial baud rate of device'))
            
    def gps_process(self, pipe):
        print('gps process', os.getpid())
        self.gpsd_socket = False
        self.poller = select.poll()
        self.baud_boot_device_hint = ''
        while True:
            self.read_pipe(pipe)
            if not self.gpsd_socket:
                self.connect()
                continue

            events = self.poller.poll(1000)
            #print('gpsd poll', events)
            if not events:
                if self.gpsconnecttime and time.monotonic() - self.gpsconnecttime > 10:
                    print(_('gpsd timeout from lack of data'))
                    self.disconnect();
                continue

            self.gpsconnecttime = False
            fd, flag = events.pop()
            if flag & select.POLLIN and self.gpsd_socket.recvdata():
                while True:
                    line = self.gpsd_socket.readline()
                    if not line:
                        break
                    try:
                        if self.parse_gpsd(gps_json_loads(line), pipe):
                            pipe.send({'devices': self.devices})                    
                    except Exception as e:
                        print(_('gpsd received invalid message'), line, e)
            else: # gpsd connection lost
                self.disconnect()
                pipe.send({'devices': self.devices})                    

class gpsd(object):
    def __init__(self, sensors):
        self.sensors = sensors
        self.devices = False # list of devices used by gpsd, or False if not connected

        self.process = gpsProcess()
        self.process.start()

        READ_ONLY = select.POLLIN | select.POLLHUP | select.POLLERR
        self.poller = select.poll()
        self.poller.register(self.process.pipe.fileno(), READ_ONLY)
        self.last_read_time = time.monotonic()

    def read(self):
        data = self.process.pipe.recv()
        while data:
            if 'devices' in data:
                print('GPSD devices', data['devices'])
                if self.devices and not data['devices']:
                    # shortcut normal timeout
                    self.sensors.lostgpsd() # gpsd is highest priority
                self.devices = data['devices']
                serialprobe.gpsddevices(self.devices)
            else:
                self.sensors.write('gps', data, 'gpsd')
            data = self.process.pipe.recv()

    def poll(self):
        # no gpsd devices: probe
        t0 = time.monotonic()

        while True:
            events = self.poller.poll(0)
            if not events:
                break
            while events:
                event = events.pop()
                fd, flag = event
                if flag != select.POLLIN:
                    # hope this is never hit
                    print(_('gpsd got flag for pipe:'), flag)
                    continue
                self.last_read_time = t0
                self.read()

        return # don't probe gpsd anymore
        if (not self.devices is False) and (t0 - self.last_read_time > 20 or not self.devices):
            device_path = serialprobe.probe('gpsd', [4800], 4)
            if device_path:
                print(_('gpsd serial probe'), device_path)
                self.probe_device, baud = device_path
                self.process.pipe.send(self.probe_device)
                
