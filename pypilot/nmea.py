#!/usr/bin/env python
#
#   Copyright (C) 2021 Sean D'Epagnier
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
from nonblockingpipe import NonBlockingPipe
from bufferedsocket import LineBufferedNonBlockingSocket
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

gps_timeoffset = 0
def parse_nmea_gps(line):
    def degrees_minutes_to_decimal(n):
        n/=100
        degrees = int(n)
        minutes = n - degrees
        return degrees + minutes*10/6

    if line[3:6] != 'RMC':
        return False

    try:
        data = line[7:len(line)-3].split(',')
        if data[1] == 'V':
            return False
        gps = {}

        try: # since we are given only time and not date, use current day
            ts = time.mktime(time.strptime(data[0], '%H%M%S.%f'))
            t0 = time.time()
            global gps_timeoffset
            ts += gps_timeoffset # seconds since 1970
            sec_in_day = 86400
            if ts > t0 or ts < t0 - sec_in_day: # wrong day
                ts -= gps_timeoffset # undo offset
                day = int(t0/sec_in_day)
                gps_timeoffset = sec_in_day*sec_in_day
                if ts + gps_timeoffset > t0:
                    gps_timeoffset -= sec_in_day
                ts += gps_timeoffset
                print('reset gps timeoffset', day)
        except:
            ts = time.time()

        lat = degrees_minutes_to_decimal(float(data[2]))
        if data[3] == 'S':
            lat = -lat

        lon = degrees_minutes_to_decimal(float(data[4]))
        if data[5] == 'W':
            lon = -lon

        speed = float(data[6]) if data[6] else 0
        gps = {'timestamp': ts, 'speed': speed, 'lat': lat, 'lon': lon}
        if data[7]:
            gps['track'] = float(data[7])

    except Exception as e:
        print(_('nmea failed to parse gps'), line, e)
        return False

    return 'gps', gps


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
    except Exception as e:
        print(_('nmea failed to parse wind'), line, e)
        return False
        
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
        return 'apb', {'mode': mode, 'track':  track, 'xte': xte, 'senderid': line[1:3]}
    except Exception as e:
        print(_('exception parsing apb'), e, line)
        return False


# waterspeed
def parse_nmea_water(line):
    '''
   ** VHW - Water speed and heading
   **
   **        1   2 3   4 5   6 7   8 9
   **        |   | |   | |   | |   | |
   ** $--VHW,x.x,T,x.x,M,x.x,N,x.x,K*hh<CR><LF>
   **
   ** Field Number:
   **  1) Degress True
   **  2) T = True
   **  3) Degrees Magnetic
   **  4) M = Magnetic
   **  5) Knots (speed of vessel relative to the water)
   **  6) N = Knots
   **  7) Kilometers (speed of vessel relative to the water)
   **  8) K = Kilometers
   **  9) Checksum

   ** LWY - Nautical Leeway Angle Measurement
   **
   **        1 2   3
   **        | |   |
   ** $--LWY,A,x.x*hh<CR><LF>
   **
   ** Field Number:
   **  1) A=Valid V=not valid
   **  2) Nautical Leeway Angle in degrees (positive indicates slippage to starboard)
   **  3) Checksum
    '''
    if line[3:6] == 'VHW':
        try:
            # for now only use the knots field
            data = line.split(',')
            speed = float(data[5])
            return 'water', {'speed': speed}
        except Exception as e:
            print(_('exception parsing vhw'), e, line)

    elif line[3:6] == 'LWY':
        try:
            # for now only use the knots field
            data = line.split(',')
            if data[1] == 'A':
                leeway = float(data[2])
                return 'water', {'leeway': leeway}
        except Exception as e:
            print(_('exception parsing vhw'), e, line)
        
    return False

nmea_parsers = {'gps': parse_nmea_gps, 'wind': parse_nmea_wind, 'rudder': parse_nmea_rudder, 'apb': parse_nmea_apb, 'water': parse_nmea_water}

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
class NMEASocket(LineBufferedNonBlockingSocket):
    def __init__(self, connection, address):
        super(NMEASocket, self).__init__(connection, address)

        global nmeasocketuid
        self.uid = nmeasocketuid
        nmeasocketuid += 1
        
    def readline(self):
        if self.b: # optimized version in c
            return self.b.readline_nmea()
        while True: # python version (not used normally)
            line = super(NMEASocket, self).readline()
            if not line:
                return False
            line = line.rstrip()
            if len(line) > 4 and (line[0] == '$' or line[0] == '!'):
                cksum = int(line[-2:], 16)
                #print("cksum", '%02X'%cksum, '%02X'%nmea_cksum(line[1:-3]))
                if cksum == nmea_cksum(line[1:-3]):
                    return line

class Nmea(object):
    def __init__(self, sensors):
        self.client = sensors.client

        self.sensors = sensors
        self.nmea_bridge = nmeaBridge(self.client.server)
        self.process = self.nmea_bridge.process
        self.pipe = self.nmea_bridge.pipe_out
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
        self.probeindex = 0

        self.start_time = time.monotonic()

    def __del__(self):
        #print('terminate nmea process')
        #self.process.terminate()
        pass

    def read_process_pipe(self):
      while True:
        msgs = self.pipe.recv()
        if not msgs:
            return
        if type(msgs) == type('string'):
            if msgs == 'sockets':
                self.sockets = True
            elif msgs == 'nosockets':
                self.sockets = False
            elif msgs[:10] == 'lostsocket':
                self.sensors.lostdevice(msgs[4:])
            else:
                print(_('unhandled nmea pipe string'), msgs)
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
            blacklist = ['MWV', 'RSA', 'APB']
            if self.sensors.gps.filtered.output.value:  # pass gps through, or use filtered gps depends on setting
                blacklist.append('RMC')
            
            if not nmea_name[3:] in blacklist:
                # do not output nmea data over tcp faster than 4hz
                # for each message time
                # forward nmea lines from serial to tcp
                dt = t-self.nmea_times[nmea_name] if nmea_name in self.nmea_times else 1
                if dt > .25:
                    self.pipe.send(line)
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
                    msg['device'] = line[1:3] + device.path[0]
                    serial_msgs[name] = msg
                break

    def remove_serial_device(self, device):
        index = self.devices.index(device)
        print(_('lost serial') + ' nmea%d' % index)
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
                        print(_('nmea got flag for process pipe:'), flag)
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
                if dt < 2.3:
                    print('serial device dt', dt, device.path, _('is another process accessing it?'))
            if dt > 15: # no data for 15 seconds
                print(_('serial device timed out'), dt, device)
                self.remove_serial_device(device)
        t4 = time.monotonic()

        # send imu nmea messages to sockets at 2hz
        dt = time.monotonic() - self.last_imu_time
        values = self.client.values.values
        if self.sockets:
            if dt > .5 and 'imu.pitch' in values:
                self.send_nmea('APXDR,A,%.3f,D,PTCH' % values['imu.pitch'].value)
                self.send_nmea('APXDR,A,%.3f,D,ROLL' % values['imu.roll'].value)
                self.send_nmea('APHDM,%.3f,M' % values['imu.heading_lowpass'].value)
                self.send_nmea('APROT,%.3f,A' % values['imu.headingrate_lowpass'].value)
                self.last_imu_time = time.monotonic()

            # limit output rate of wind and rudder
            t = time.monotonic()
            for name in ['wind', 'rudder']:
                source = self.sensors.sensors[name].source.value
                # only output to tcp if we have a better source
                if source_priority[source] >= source_priority['tcp']:
                    continue
                
                if not name in self.nmea_times:
                    self.nmea_times[name] = 0
                    
                dt = t - self.nmea_times[name]
                freq = 1/dt
                rate = self.sensors.sensors[name].rate.value
                if freq < rate:
                    if name == 'wind':
                        wind = self.sensors.wind
                        self.send_nmea('APMWV,%.3f,R,%.3f,N,A' % (wind.direction.value, wind.speed.value))
                    elif name == 'rudder':
                        self.send_nmea('APRSA,%.3f,A,,' % self.sensors.rudder.angle.value)
                    period = 1/rate
                    self.nmea_times[name] = max(min(self.nmea_times[name] + period, t+period), t)

        t5 = time.monotonic()
        if not self.nmea_bridge.process:
            self.nmea_bridge.poll()
        
        t6 = time.monotonic()
        if t6 - t0 > .05 and t0-self.start_time > 1: # report times if processing takes more than 0.05 seconds
            print('nmea poll times', t6-self.start_time, t1-t0, t2-t1, t3-t2, t4-t3, t5-t4, t6-t5, t6-t0)
            
    def probe_serial(self):
        # probe new nmea data devices
        if not self.probedevice:
            try:
                index = self.devices.index(False)
            except:
                index = len(self.devices)
            if self.probeindex != index and \
               (self.probeindex >= len(self.devices) or not self.devices[self.probeindex]):
                serialprobe.relinquish('nmea%d' % self.probeindex)
            self.probeindex = index

            self.probedevicepath = serialprobe.probe('nmea%d' % self.probeindex, [38400, 4800], 8)
            if self.probedevicepath:
                print('nmea probe', self.probedevicepath)
                try:
                    self.probedevice = NMEASerialDevice(self.probedevicepath)
                    self.probetime = time.monotonic()
                except Exception as e: # serial.serialutil.SerialException:
                    print(_('failed to open'), self.probedevicepath, 'for nmea data', e)

            return

        # see if the probe device gets a valid nmea message
        if self.probedevice.readline():
            #print('nmea new device', self.probedevicepath)
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
        elif time.monotonic() - self.probetime > 5:
            #print('nmea serial probe timeout', self.probeindex, index, self.probedevicepath)
            self.probedevice = None # timeout


    def send_nmea(self, msg):
        self.pipe.send(msg)


def getddmmyy(self):
    today = datetime.date.today()
    return '%02d%02d%02d' % (today.day, today.month, today.year % 100)

def gethhmmss(self):
    t = datetime.datetime.now()
    return t.strftime("%H%M%S.%f")[:-4]

def getddmmmmmm(self, degrees, n, s):
    minutes = (abs(degrees) - abs(int(degrees))) * 60
    return '%02d%07.4f,%c' % (abs(degrees), minutes, n if degrees >= 0 else s)
        
class nmeaBridge(object):
    def __init__(self, server):
        self.client = pypilotClient(server)
        self.client.connection.name += 'nmeabridge'
        self.multiprocessing = server.multiprocessing
        self.pipe, self.pipe_out = NonBlockingPipe('nmea pipe', self.multiprocessing)
        if self.multiprocessing:
            self.process = multiprocessing.Process(target=self.nmea_process, daemon=True)
            self.process.start()
        else:
            self.process = False
            self.setup()

    def setup(self):
        self.sockets = []

        self.nmea_client = self.client.register(Property('nmea.client', '', persistent=True))

        self.gps_id = self.client.register(EnumProperty('nmea.gps_id', 'APRMC', ['APRMC', 'GPRMC'], persistent=True))

        
        self.client_socket_warning_address = False

        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setblocking(0)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.connecting_client_socket = False
        self.client_socket = False

        port = DEFAULT_PORT
        while True:
            try:
                self.server.bind(('0.0.0.0', port))
                break
            except:
                print(_('nmea server on port') + (' %d: '  % port) + _('bind failed.'))
            time.sleep(1)
        print(_('listening on port'), port, _('for nmea connections'))

        self.server.listen(5)

        self.client_socket_nmea_address = False
        self.nmea_client_connect_time = 0
        self.last_values = {}
        keys = ['gps.source', 'wind.source', 'rudder.source', 'apb.source', 'water.source', 'gps.filtered.output']
        for k in keys:
            self.last_values[k] = 'none'                    
        for name in self.last_values:
            self.client.watch(name)
        self.addresses = {}
        cnt = 0

        self.poller = select.poll()
        self.poller.register(self.server, select.POLLIN)
        self.fd_to_socket = {self.server.fileno() : self.server}

        self.poller.register(self.client.connection, select.POLLIN)
        self.fd_to_socket[self.client.connection.fileno()] = self.client
        
        if self.multiprocessing:
            self.poller.register(self.pipe, select.POLLIN)
            self.fd_to_socket[self.pipe.fileno()] = self.pipe

        self.msgs = {}

    def setup_watches(self, watch=True):
        watchlist = ['gps.source', 'wind.source', 'rudder.source', 'apb.source']
        for name in watchlist:
            self.client.watch(name, watch)

    def receive_nmea(self, line, sock):
        device = 'socket' + str(sock.uid)
        parsers = []

        # if we receive a "special" pypilot nmea message from this
        # socket, then mark it to rebroadcast to other nmea sockets
        # normally only nmea data received from serial ports is broadcast
        if not sock.broadcast:
            if line == '$PYPBS*48':
                sock.broadcast = True
                return
        else:
            for s in self.sockets:
                if s != sock:
                    s.write(line+'\r\n')
        
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
                msg['device'] = line[1:3] + device
                self.msgs[name] = msg
                return

    def new_socket_connection(self, connection, address):
        #print('nmea new socket connection', connection, address)
        max_connections = 10
        if len(self.sockets) == max_connections:
            connection.close()
            print(_('nmea server has too many connections'))
            return
    
        if not self.sockets:
            self.setup_watches()
            self.pipe.send('sockets')

        sock = NMEASocket(connection, address)
        # normally don't re-transmit nmea data received from sockets
        # if it is marked to broadcast, then data received will re-transmit
        sock.broadcast = False
        self.sockets.append(sock)

        self.addresses[sock] = address
        fd = sock.socket.fileno()
        self.fd_to_socket[fd] = sock

        self.poller.register(sock.socket, select.POLLIN)
        return sock

    def socket_lost(self, sock, fd):
        #print('nmea socket lost', fd, sock, self.connecting_client_socket)
        if sock == self.connecting_client_socket:
            self.close_connecting_client()
            return
        if sock == self.client_socket:
            print(_('nmea client lost connection'))
            self.client_socket = False
        try:
            self.sockets.remove(sock)
        except:
            print(_('nmea sock not in sockets!'))
            return
        
        self.pipe.send('lostsocket' + str(sock.uid))
        if not self.sockets:
            self.setup_watches(False)
            self.pipe.send('nosockets')

        try:
            self.poller.unregister(fd)
        except Exception as e:
            print(_('nmea failed to unregister socket'), e)

        try:
            del self.fd_to_socket[fd]
        except Exception as e:
            print(_('nmea failed to remove fd'), e)

        try:
            del self.addresses[sock]
        except Exception as e:
            print(_('nmea failed to remove address'), e)

        sock.close()

    def connect_client(self):
        if self.client_socket: # already connected
            if self.client_socket_nmea_address != self.nmea_client.value:
                self.client_socket.socket.close() # address has changed, close connection
            return

        timeout = 30
        t = time.monotonic()
        if self.client_socket_nmea_address != self.nmea_client.value:
            self.nmea_client_connect_time = t - timeout + 5 # timeout sooner if it changed

        self.client_socket_nmea_address = self.nmea_client.value
        if t - self.nmea_client_connect_time < timeout:
            return
        
        self.nmea_client_connect_time = t

        if not self.nmea_client.value:
            return
        
        if not ':' in self.nmea_client.value:
            self.warn_connecting_client(_('invalid value'))
            return
        
        hostport = self.nmea_client.value.split(':')

        host = hostport[0]
        port = hostport[1]

        self.client_socket = False
        def warning(e, s):
            self.warn_connecting_client(_('connect error') + ' : ' + str(e))
            s.close()            
            
        try:
            port = int(port)
            if self.connecting_client_socket:
                self.close_connecting_client()
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setblocking(0)
            s.connect((host, port))
            self.warn_connecting_client('connected without blocking')
            self.client_connected(s)
        except OSError as e:
            import errno
            if e.args[0] is errno.EINPROGRESS:
                self.poller.register(s, select.POLLOUT)
                self.fd_to_socket[s.fileno()] = s
                self.connecting_client_socket = s
                return
            warning(e, s)
        except Exception as e:
            warning(e, s)

    def warn_connecting_client(self, msg):
        if self.client_socket_warning_address != self.client_socket_nmea_address:
            print('nmea client ' + msg, self.client_socket_nmea_address)
            self.client_socket_warning_address = self.client_socket_nmea_address
            
    def close_connecting_client(self):
        self.warn_connecting_client(_('failed to connect'))
        fd = self.connecting_client_socket.fileno()
        self.poller.unregister(fd)
        del self.fd_to_socket[fd]
        self.connecting_client_socket.close()
        self.connecting_client_socket = False

    def client_connected(self, connection):
        print(_('nmea client connected'), self.client_socket_nmea_address)
        self.client_socket_warning_address = False
        self.client_socket = self.new_socket_connection(connection, self.client_socket_nmea_address)
        self.connecting_client_socket = False
        
    def nmea_process(self):
        print('nmea process', os.getpid())
        self.setup()
        while True:
            t0 = time.monotonic()
            timeout = 100 if self.sockets else 10000
            self.poll(timeout)

    def receive_pipe(self):
        while True: # receive all messages in pipe
            msg = self.pipe.recv()
            if not msg:
                return
            if msg[0] != '$': # perform checksum in this subprocess
                msg = '$' + msg + ('*%02X' % nmea_cksum(msg))
            # relay nmea message from server to all tcp sockets
            for sock in self.sockets:
                sock.write(msg + '\r\n')

    def poll(self, timeout=0):
        t0 = time.monotonic()
        events = self.poller.poll(timeout)
        t1 = time.monotonic()
        if t1-t0 > timeout:
            print(_('poll took too long in nmea process!'))
            
        while events:
            fd, flag = events.pop()
            sock = self.fd_to_socket[fd]
            if flag & (select.POLLHUP | select.POLLERR | select.POLLNVAL):
                if sock == self.server:
                    print(_('nmea bridge lost server connection'))
                    exit(2)
                if sock == self.pipe:
                    print(_('nmea bridge lost pipe to autopilot'))
                    exit(2)
                self.socket_lost(sock, fd)
            elif sock == self.server:
                self.new_socket_connection(*self.server.accept())
            elif sock == self.pipe:
                self.receive_pipe()
            elif sock == self.client:
                pass # wake from poll
            elif sock == self.connecting_client_socket and flag & select.POLLOUT:
                self.poller.unregister(fd)
                del self.fd_to_socket[fd]
                self.client_connected(self.connecting_client_socket)
            elif flag & select.POLLIN:
                if not sock.recvdata():
                    self.socket_lost(sock, fd)
                else:
                    while True:
                        line = sock.readline()
                        if not line:
                            break
                
                        self.receive_nmea(line, sock)
            else:
                print(_('nmea bridge unhandled poll flag'), flag)

        # if we are not multiprocessing, the pipe won't be pollable, so receive any data now
        if not self.process:
            self.receive_pipe()
                
        t2 = time.monotonic()

        # send any parsed nmea messages the server might care about
        if self.msgs:
            #print('nmea msgs', self.msgs)
            if self.pipe.send(self.msgs):
                self.msgs = {}
        t3 = time.monotonic()

        # receive pypilot messages
        pypilot_msgs = self.client.receive()
        for name in pypilot_msgs:
            value = pypilot_msgs[name]
            self.last_values[name] = value
            if name == 'gps.filtered.output': # watch filtered fix only if we are outputting it
                self.client.watch('gps.filtered.fix', value)
        #except Exception as e:
        #    print('nmea exception receiving:', e)
        t4 = time.monotonic()

        # generate filtered gps output if enabled
        if self.last_values['gps.filtered.output'] is True and self.last_values['gps.filtered.fix']:
            fix = self.last_values['gps.filtered.fix']
            self.last_values['gps.filtered.fix'] = False
            try:
                lat, lon, speed, track = map(lambda k: fix[k], ['lat', 'lon', 'speed', 'track'])
                # todo: incorporate timestamp accurate?
                msg = self.gps_id.value + ',' + gethhmmss() + ',A,' \
                      + getddmmmmmm(lat, 'N', 'S') + ',' + getddmmmmmm(lon, 'E', 'W') \
                      + ',%.2f,' % speed + '%.2f,' % (track if track > 0 else 360 + track) \
                      + getddmmyy() + ',,,A'
                msg = '$' + msg + ('*%02X' % nmea_cksum(msg))
                # relay nmea message from server to all tcp sockets
                for sock in self.sockets:
                    sock.write(msg + '\r\n')

            except:
                pass
        
        # flush sockets
        for sock in self.sockets:
            sock.flush()
        t5 = time.monotonic()

        # reconnect client tcp socket
        self.connect_client()
                                
        t6 = time.monotonic()

        if t6-t1 > .1:
            print(_('nmea process loop too slow:'), t1-t0, t2-t1, t3-t2, t4-t3, t5-t4, t6-t5)
