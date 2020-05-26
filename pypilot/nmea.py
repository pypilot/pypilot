#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

#
# A separate process listens on port 20220 for tcp connections
# any nmea data received is relayed to all clients
#
# serial ports are probed for incoming nmea data, if found
# this port will be managed and sentences translated used
# for sensor inputs such as:
#  wind, gps, rudder
#
# inputs nmea: wind, rudder, autopilot commands (serial and tcp)
# outputs nmea: pitch, roll, and heading messages, wind, rudder (tcp)


DEFAULT_PORT = 20220

import sys, select, time, socket
import multiprocessing
import serial
from client import pypilotClient
from values import *
from nonblockingpipe import LineBufferedNonBlockingPipe
from sensors import source_priority
import serialprobe

import fcntl
# these are not defined in python module
TIOCEXCL = 0x540C

# nmea uses a simple xor checksum
def nmea_cksum(msg):
    value = 0
    for c in msg: # skip over the $ at the begining of the sentence
        value ^= ord(c)
    return value & 255

def check_nmea_cksum(line):
    cksplit = line.split('*')
    try:
        return nmea_cksum(cksplit[0][1:]) == int(cksplit[1], 16)
    except:
        return False

def parse_nmea_gps(line):
    if line[3:6] != 'RMC':
        return False

    try:
        data = line[7:len(line)-3].split(',')
        timestamp = float(data[0])
        speed = float(data[6])
        heading = float(data[7])
    except:
        return False

    if data[1] == 'V':
        return False

    return 'gps', {'timestamp': timestamp, 'track': heading, 'speed': speed}


'''
   ** MWV - Wind Speed and Angle
   **
   **
   **
   ** $--MWV,x.x,a,x.x,a*hh<CR><LF>**
   ** Field Number:
   **  1) Wind Angle, 0 to 360 degrees
   **  2) Reference, R = Relative, T = True
   **  3) Wind Speed
   **  4) Wind Speed Units, K/M/N
   **  5) Status, A = Data Valid
   **  6) Checksum
'''
def parse_nmea_wind(line):
    if line[3:6] != 'MWV':
        return False

    data = line.split(',')
    msg = {}
    try:
        msg['direction'] = float(data[1])
    except:
        return False  # require direction

    try:
        speed = float(data[3])
        speedunit = data[4]
        if speedunit == 'K': # km/h
            speed *= .53995
        elif speedunit == 'M': # m/s
            speed *= 1.94384
        msg['speed'] = speed
    except:
        pass
        
    return 'wind', msg

def parse_nmea_rudder(line):
    if line[3:6] != 'RSA':
        return False

    data = line.split(',')
    try:
        angle = float(data[1])
    except:
        angle = False

    return 'rudder', {'angle': angle}


def parse_nmea_apb(line):
    # also allow ap commands (should we allow via serial too??)
    '''
   ** APB - Autopilot Sentence "B"
   **                                         13    15
   **        1 2 3   4 5 6 7 8   9 10   11  12|   14|
   **        | | |   | | | | |   | |    |   | |   | |
   ** $--APB,A,A,x.x,a,N,A,A,x.x,a,c--c,x.x,a,x.x,a*hh<CR><LF>
   **
   **  1) Status
   **     V = LORAN-C Blink or SNR warning
   **     V = general warning flag or other navigation systems when a reliable
   **         fix is not available
   **  2) Status
   **     V = Loran-C Cycle Lock warning flag
   **     A = OK or not used
   **  3) Cross Track Error Magnitude
   **  4) Direction to steer, L or R
   **  5) Cross Track Units, N = Nautical Miles
   **  6) Status
   **     A = Arrival Circle Entered
   **  7) Status
   **     A = Perpendicular passed at waypoint
   **  8) Bearing origin to destination
   **  9) M = Magnetic, T = True
   ** 10) Destination Waypoint ID
   ** 11) Bearing, present position to Destination
   ** 12) M = Magnetic, T = True
   ** 13) Heading to steer to destination waypoint
   ** 14) M = Magnetic, T = True
   ** 15) Checksum
        '''
    if line[3:6] != 'APB':
        return False
    try:
        data = line[7:len(line)-3].split(',')
        mode = 'compass' if data[13] == 'M' else 'gps'
        track = float(data[12])
        xte = float(data[2])
        xte = min(xte, 0.15) # maximum 0.15 miles
        if data[3] == 'L':
            xte = -xte
        return 'apb', {'mode': mode, 'track':  track, 'xte': xte, '**': line[1:3] == 'GP'}
    except Exception as e:
        print('ex', e)
        return False

nmea_parsers = {'gps': parse_nmea_gps, 'wind': parse_nmea_wind, 'rudder': parse_nmea_rudder, 'apb': parse_nmea_apb}

from pypilot.linebuffer import linebuffer
class NMEASerialDevice(object):
    def __init__(self, path):
        self.device = serial.Serial(*path)
        self.path = path
        self.device.timeout=0 #nonblocking
        fcntl.ioctl(self.device.fileno(), TIOCEXCL)
        self.b = linebuffer.LineBuffer(self.device.fileno())

    def readline(self):
        return self.b.readline_nmea()

    def close(self):
        self.device.close()

nmeasocketuid = 0
class NMEASocket(object):
    def __init__(self, connection):
        super(NMEASocket, self).__init__(connection)

        global nmeasocketuid
        self.uid = nmeasocketuid
        nmeasocketuid += 1
        
    def readline(self):
        if self.b: # optimized version in c
            return self.b.readline_nmea()
        line = self.readline()
        if not line:
            return False
        if len(line) > 4 and line[0] == '$':
            cksum = int(buf[-2:])
            if cksum == nmea_cksum(line[1:-2]):
                return line
        return self.readline()

class Nmea(object):
    def __init__(self, sensors):
        self.client = sensors.client

        self.sensors = sensors
        self.nmea_bridge = nmeaBridge(self.client.server)
        self.pipe = self.nmea_bridge.nmea_pipe
        self.sockets = False

        self.poller = select.poll()
        self.process_fd = self.pipe.fileno()
        self.poller.register(self.process_fd, select.POLLIN)
        self.device_fd = {}

        self.nmea_times = {}
        self.last_imu_time = time.monotonic()
        self.last_rudder_time = time.monotonic()

        self.devices = []
        self.devices_lastmsg = {}
        self.probedevice = None

        self.start_time = time.monotonic()

    def __del__(self):
        #print('terminate nmea process')
        #self.process.terminate()
        pass

    def read_process_pipe(self):
        msgs = self.pipe.recv()
        if type(msgs) == type('string'):
            if msgs == 'sockets':
                self.sockets = True
            elif msgs == 'nosockets':
                self.sockets = False
            elif msgs[:10] == 'lostsocket':
                self.sensors.lostdevice(msgs[4:])
            else:
                print('handled nmea pipe string')
        else:
            for name in msgs:
                self.sensors.write(name, msgs[name], 'tcp')
                
    def read_serial_device(self, device, serial_msgs):
        t = time.monotonic()
        line = device.readline()
        if not line:
            return
        if self.sockets:
            nmea_name = line[:6]
            # we output mwv and rsa messages after calibration
            # do not relay apb messages
            if not nmea_name[3:] in ['MWV', 'RSA', 'APB']:
                # do not output nmea data over tcp faster than 5hz
                # for each message time
                # forward nmea lines from serial to tcp

                dt = t-self.nmea_times[nmea_name] if nmea_name in self.nmea_times else -1
                if dt > .2:
                    self.pipe.send(line, False)
                    self.nmea_times[nmea_name] = t

        self.devices_lastmsg[device] = t
        parsers = []

        # only process if
        # 1) current source is lower priority
        # 2) we do not have a source yet
        # 3) this the correct device for this data
        for name in nmea_parsers:
            name_device = self.sensors.sensors[name].device
            current_source = self.sensors.sensors[name].source.value
            if source_priority[current_source] > source_priority['serial'] or \
               not name_device or name_device[2:] == device.path[0]:
                parsers.append(nmea_parsers[name])

        # parse the nmea line, and update serial messages
        for parser in parsers:
            result = parser(line)
            if result:
                name, msg = result
                if name:
                    msg['device'] = line[1:3]+device.path[0]
                    serial_msgs[name] = msg
                break

    def remove_serial_device(self, device):
        index = self.devices.index(device)
        print('lost serial nmea%d' % index)
        self.sensors.lostdevice(self.devices[index].path[0])
        self.devices[index] = False
        self.poller.unregister(device.device.fileno())
        del self.devices_lastmsg[device]
        device.close()
            
    def poll(self):
        t0 = time.monotonic()
        self.probe_serial()

        t1 = time.monotonic()
        # handle tcp nmea messages
        serial_msgs = {}
        while True:
            events = self.poller.poll(0)
            if not events:
                break
            while events:
                event = events.pop()
                fd, flag = event
                if fd == self.process_fd:
                    if flag != select.POLLIN:
                        print('nmea got flag for process pipe:', flag)
                    else:
                        self.read_process_pipe()
                elif flag == select.POLLIN:
                    #if flag & (select.POLLHUP | select.POLLERR | select.POLLNVAL):
                    self.read_serial_device(self.device_fd[fd], serial_msgs)
                else:
                    self.remove_serial_device(self.device_fd[fd])

        t2 = time.monotonic()
        for name in serial_msgs:
            self.sensors.write(name, serial_msgs[name], 'serial')
        t3 = time.monotonic()
                
        for device in self.devices:
            # timeout serial devices
            if not device:
                continue
            dt = time.monotonic() - self.devices_lastmsg[device]
            if dt > 2:
                if dt < 3:
                    print('serial device dt', dt, device.path, 'is another process accessing it?')
            if dt > 15: # no data for 15 seconds
                print('serial device timed out', dt, device)
                self.remove_serial_device(device)
        t4 = time.monotonic()

        # send nmea messages to sockets at 2hz
        dt = time.monotonic() - self.last_imu_time
        if self.sockets and dt > .5 and \
           'imu.pitch' in self.client.values:
            self.send_nmea('APXDR,A,%.3f,D,PTCH' % self.client.values['imu.pitch'].value)
            self.send_nmea('APXDR,A,%.3f,D,ROLL' % self.client.values['imu.roll'].value)
            self.send_nmea('APHDM,%.3f,M' % self.client.values['imu.heading_lowpass'].value)
            self.last_imu_time = time.monotonic()

        # should we output gps?  for now no
            
        # limit to 5hz output of wind and rudder
        t = time.monotonic()
        for name in ['wind', 'rudder'] if self.sockets else []:
            dt = t - self.nmea_times[name] if name in self.nmea_times else -1
            source = self.sensors.sensors[name].source.value
            # only output to tcp if we have a better source
            if dt > .2 and source_priority[source] < source_priority['tcp']:
                if name == 'wind':
                    wind = self.sensors.wind
                    self.send_nmea('APMWV,%.3f,R,%.3f,N,A' % (wind.direction.value, wind.speed.value))
                else:
                    self.send_nmea('APRSA,%.3f,A,,' % self.sensors.rudder.angle.value)
                self.nmea_times[name] = t
            
        t5 = time.monotonic()
        if not self.nmea_bridge.process:
            self.nmea_bridge.poll()
        
        t6 = time.monotonic()
        if t6 - t0 > .1 and self.start_time - t0 > 1:
            print('nmea poll times', self.start_time-t0, t1-t0, t2-t1, t3-t2, t4-t3, t5-t4, t6-t5)
            
    def probe_serial(self):
        # probe new nmea data devices
        if not self.probedevice:
            try:
                self.probeindex = self.devices.index(False)
            except:
                self.probeindex = len(self.devices)
            self.probedevicepath = serialprobe.probe('nmea%d' % self.probeindex, [38400, 4800])
            if self.probedevicepath:
                try:
                    self.probedevice = NMEASerialDevice(self.probedevicepath)
                    self.probetime = time.monotonic()
                except serial.serialutil.SerialException:
                    print('failed to open', self.probedevicepath, 'for nmea data')
                    pass
        elif time.monotonic() - self.probetime > 5:
            print('nmea serial probe timeout', self.probedevicepath)
            self.probedevice = None # timeout
        else:
            # see if the probe device gets a valid nmea message
            if self.probedevice:
                if self.probedevice.readline():
                    print('new nmea device', self.probedevicepath)
                    serialprobe.success('nmea%d' % self.probeindex, self.probedevicepath)
                    if self.probeindex < len(self.devices):
                        self.devices[self.probeindex] = self.probedevice
                    else:
                        self.devices.append(self.probedevice)
                    fd = self.probedevice.device.fileno()
                    self.device_fd[fd] = self.probedevice
                    self.poller.register(fd, select.POLLIN)
                    self.devices_lastmsg[self.probedevice] = time.monotonic()
                    self.probedevice = None

    def send_nmea(self, msg):
        line = '$' + msg + ('*%02X' % nmea_cksum(msg))
        self.pipe.send(line, False)
        
class nmeaBridge(object):
    def __init__(self, server):
        self.client = pypilotClient(server)
        self.multiprocessing = server.multiprocessing
        self.nmea_pipe, self.pipe = LineBufferedNonBlockingPipe('nmea pipe', self.multiprocessing)
        if self.multiprocessing:
            self.process = multiprocessing.Process(target=self.nmea_process, daemon=True)
            self.process.start()
        else:
            self.process = False
            self.setup()

    def setup_watches(self, watch=True):
        watchlist = ['gps.source', 'wind.source', 'rudder.source', 'apb.source']
        for name in watchlist:
            self.client.watch(name, watch)

    def receive_nmea(self, line, device):
        parsers = []

        # optimization to only to parse sentences here that would be discarded
        # in the main process anyway because they are already handled by a source
        # with a higher priority than tcp
        tcp_priority = source_priority['tcp']
        for name in nmea_parsers:
            if source_priority[self.last_values[name + '.source']] >= tcp_priority:
                parsers.append(nmea_parsers[name])

        for parser in  parsers:
            result = parser(line)
            if result:
                name, msg = result
                msg['device'] = device + line[1:3]
                self.msgs[name] = msg
                return

    def new_socket_connection(self, connection, address):
        max_connections = 10
        if len(self.sockets) == max_connections:
            connection.close()
            print('nmea server has too many connections')
            return
    
        if not self.sockets:
            self.setup_watches()
            self.pipe.send('sockets')

        sock = NMEASocket(connection)
        self.sockets.append(sock)

        self.addresses[sock] = address
        fd = sock.socket.fileno()
        self.fd_to_socket[fd] = sock

        self.poller.register(sock.socket, select.POLLIN)

        return sock

    def socket_lost(self, sock, fd):
        if sock == self.client_socket:
            self.client_socket = False
        try:
            self.sockets.remove(sock)
        except:
            print('nmea sock not in sockets!')
            return
        
        self.pipe.send('lostsocket' + str(sock.uid))
        if not self.sockets:
            self.setup_watches(False)
            self.pipe.send('nosockets')

        try:
            self.poller.unregister(fd)
        except Exception as e:
            print('nmea failed to unregister socket', e)

        try:
            del self.fd_to_socket[fd]
        except Exception as e:
            print('nmea failed to remove fd', e)

        try:
            del self.addresses[sock]
        except Exception as e:
            print('nmea failed to remove address', e)

        sock.close()

    def connect_client(self):
        host, port = self.nmea_client.split(':')
        port = int(port)
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tc0 = time.monotonic()
            s.connect((host, port))
            print('connected to', host, port, 'in', time.monotonic() - tc0, 'seconds')
            self.client_socket = self.new_socket_connection(s, self.nmea_client)
            self.client_socket.nmea_client = self.nmea_client
        except Exception as e:
            print('failed to connect to', self.nmea_client, ':', e)
            self.client_socket = False

    def nmea_process(self):
        self.process = True
        self.setup()
        while True:
            t0 = time.monotonic()
            timeout = 100 if self.sockets else 10000
            self.process_poll(timeout)

    def poll(self):
        if not self.process:
            self.process_poll(0)

    def setup(self):
        self.sockets = []

        self.nmea_client = self.client.register(Property('nmea.client', '', persistent=True))

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setblocking(0)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_socket = False

        port = DEFAULT_PORT
        while True:
            try:
                self.server.bind(('0.0.0.0', port))
                break
            except:
                print('nmea server on port %d: bind failed.', port)
            time.sleep(1)
        print('listening on port', port, 'for nmea connections')

        self.server.listen(5)

        self.last_values = {'gps.source' : 'none', 'wind.source' : 'none', 'rudder.source': 'none', 'apb.source': 'none'}
        self.addresses = {}
        cnt = 0

        self.poller = select.poll()
        self.poller.register(self.server, select.POLLIN)
        self.fd_to_socket = {self.server.fileno() : self.server}
        if self.multiprocessing:
            self.poller.register(self.pipe, select.POLLIN)
            self.fd_to_socket[self.pipe.fileno()] = self.pipe

        self.msgs = {}

    def receive_pipe(self):
        while True: # receive all messages in pipe
            msg = self.pipe.recv()
            if not msg:
                return
            # relay nmea message from server to all tcp sockets
            for sock in self.sockets:
                sock.send(msg)

    def process_poll(self, timeout=0):
        t0 = time.monotonic()
        events = self.poller.poll(timeout)
        t1 = time.monotonic()
        if t1-t0 > timeout:
            print('poll took too long in nmea process!')

        while events:
            fd, flag = events.pop()
            sock = self.fd_to_socket[fd]
            if flag & (select.POLLHUP | select.POLLERR | select.POLLNVAL):
                if sock == self.server:
                    print('nmea bridge lost server connection')
                    exit(2)
                if sock == self.pipe:
                    print('nmea bridge pipe to autopilot')
                    exit(2)
                self.socket_lost(sock, fd)
            elif sock == self.server:
                self.new_socket_connection(*self.server.accept())
            elif sock == self.pipe:
                self.receive_pipe()
            elif flag & select.POLLIN:
                if not sock.recv():
                    self.socket_lost(sock, fd)
                else:
                    while True:
                        line = sock.readline()
                        if not line:
                            break
                        self.receive_nmea(line, 'socket' + str(sock.uid))
            else:
                print('nmea bridge unhandled poll flag', flag)

        # if we are not multiprocessing, the pipe won't be pollable, so receive any data now
        if not self.process:
            self.receive_pipe()
                
        t2 = time.monotonic()

        # send any parsed nmea messages the server might care about
        if self.msgs:
            if self.pipe.send(msgs):
                self.msgs = {}
        t3 = time.monotonic()

        # receive pypilot messages
        try:
            pypilot_msgs = self.client.receive()
            for name in pypilot_msgs:
                value = pypilot_msgs[name]['value']
                self.last_values[name] = value
        except Exception as e:
            print('nmea exception receiving:', e)
        t4 = time.monotonic()

        # flush sockets
        for sock in self.sockets:
            sock.flush()
        t5 = time.monotonic()

        # reconnect client tcp socket
        if self.client_socket:
            if self.client_socket.nmea_client != self.nmea_client.value:
                self.client_socket.socket.close() # address has changed, close connection
        elif self.nmea_client.value:
            try:
                self.connect_client(self.nmea_client.value)
            except Exception as e:
                print('failed to create nmea socket as host:port', self.nmea_client.value, e)
                self.last_values['nmea.client'] = '' # don't try until changed
                                
        t6 = time.monotonic()

        if t6-t1 > .1:
            print('nmea process loop too slow:', dt, t1-t0, t2-t1, t3-t2, t4-t3, t5-t4, t6-t5)
