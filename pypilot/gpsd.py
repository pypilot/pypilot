#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import multiprocessing, time, socket, select
from nonblockingpipe import NonBlockingPipe
from bufferedsocket import LineBufferedNonBlockingSocket
from values import *
import serialprobe, pyjson

class gpsProcess(multiprocessing.Process):
    def __init__(self):
        # split pipe ends
        self.pipe, pipe = NonBlockingPipe('gps_pipe', True)
        super(gpsProcess, self).__init__(target=self.gps_process, args=(pipe,), daemon=True)
        
    def connect(self):        
        try:
            socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            socket.settimeout(0)
            socket.connect(('127.0.0.1', 2947))
            socket.write('?WATCH={"enable":true,"json":true};'.encode())
            self.poller.register(socket, select.POLLIN)
            self.gpsd_socket = LineBufferedNonBlockingSocket(socket, 'gpsd')

            #self.gpsd = gps.gps(mode=gps.WATCH_ENABLE) #starting the stream of info
            self.devices = []
            print('gpsd connected')
        #except socket.error:
        except ConnectionRefusedError:
            print('gpsd failed to connect')
            self.gpsd_socket = False
            time.sleep(10)
        except Exception as e:
            self.gpsd_socket = False
            print('exception connecting to gps', e)
            time.sleep(10)

    def disconnect(self):
        print('gpsd disconnected')
        self.poller.unregister(self.gpsd_socket.socket)
        self.gpsd_socket.close()
        self.gpsd_socket = False
        self.devices = []

    def read_pipe(self, pipe):
        while True:
            device = pipe.recv()
            if not device:
                break
            if not self.devices: # only probe if there are no gpsd devices
                print('gpsd PROBING...', device)                    
                if not os.system('timeout -s KILL -t 8 gpsctl -f ' + device + ' 2> /dev/null'):
                    print('gpsd PROBE success', device)
                    # probe was success
                    os.environ['GPSD_SOCKET'] = '/tmp/gpsd.sock'
                    os.system('gpsdctl add ' + device)
                    print('gpsd probe success: ' + device)
                    self.devices = [device]
                else:
                    print('gpsd probe failed')
            # always reply with devices when asked to probe
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
                print('gpsd activated device', device)
                if not device in self.devices:
                    self.devices.append(device)
                    ret = True
            else:
                print('gpsd deactivated', device)
                if device in self.devices:
                    self.devices.remove(device)
                    ret = True
        elif cls == 'TPV':
            if msg['mode'] == 3:
                fix = {}
                for key in ['track', 'speed', 'latitude', 'longitude', 'device']:
                    fix[key] = msg[key]
                fix['speed'] *= 1.944 # knots
                if not msg['device'] in self.devices:
                    self.devices.append(device)
                    ret = True
                pipe.send(fix, False)
        return ret

    def gps_process(self, pipe):
        print('gps process', os.getpid())
        self.gpsd_socket = False
        self.poller = select.poll()
        self.devices = []
        while True:
            if not self.gpsd_socket:
                self.connect()
                self.read_pipe(pipe)
            events = self.poller.poll(3):
            if events:
                fd, flag = events.pop()
                if flag & select.POLLIN and self.gpsd_socket.recvdata():
                    while True:
                        line = connection.readline()
                        if not line:
                            break
                        try:
                            if self.parse_gpsd(pyjson.loads(line), pipe):
                                pipe.send({'devices': self.devices})                    
                        except Exception as e:
                            print('gpsd received invalid message', line, e)
                else: # gpsd connection lost
                    self.disconnect()
                    pipe.send({'devices': self.devices})                    

class gpsd(object):
    def __init__(self, sensors):
        self.sensors = sensors
        self.reserved_devices = [] # list of devices used by gps we didn't probe
        self.probe_device = False  # device process is currently probing
        self.probed_device = False # device process probed
        self.devices = [] # list of devices used by gpsd

        self.process = gpsProcess()
        self.process.start()

        READ_ONLY = select.POLLIN | select.POLLHUP | select.POLLERR
        self.poller = select.poll()
        self.poller.register(self.process.pipe.fileno(), READ_ONLY)
            
    def read(self):
        data = self.process.pipe.recv()
        if 'devices' in data:
            self.devices = data['devices']
            if self.probe_device:
                if self.probe_device in self.devices:
                    serialprobe.success('gpsd', (self.probe_device, 4800))
                    self.probed_device = self.probe_device
                else: # probe failed
                    serialprobe.relinquish('gpsd')
                self.probe_device = False

            for device in list(self.reserved_devices):
                if not device in self.devices:
                    serialprobe.unreserve(device)
                    self.reserved_devices.remove(device)
                    
            for device in self.devices:
                if device != self.probed_device and \
                   not device in self.reserved_devices:
                    serialprobe.reserve(device)
                    self.reserved_devices.append(device)
        else:
            self.sensors.write('gps', data, 'gpsd')

    def poll(self):
        if not self.devices and not self.probe_device: # no gpsd devices: probe
            device_path = serialprobe.probe('gpsd', [4800], 4)
            if device_path:
                print('gpsd serial probe', device_path)
                self.probe_device, baud = device_path
                self.process.pipe.send(self.probe_device)

        while True:
            events = self.poller.poll(0)
            if not events:
                break
            while events:
                event = events.pop()
                fd, flag = event
                if flag != select.POLLIN:
                    # hope this is never hit
                    print('gpsd got flag for pipe:', flag)
                    continue
                self.read()
