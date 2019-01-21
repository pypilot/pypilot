#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  


# this daemon translates nmea tcp messages to signalk
# serving as a tcp nmea server and signalk client
# allowing the autopilot to work with an unmodified opencpn
#
# signalk->nmea: pitch, roll, and heading messages
# nmea->signalk: autopilot commands


import sys, select, time, socket
import multiprocessing
from signalk.client import SignalKClient
from signalk.bufferedsocket import LineBufferedNonBlockingSocket
_

# nmea uses a simple xor checksum
def cksum(msg):
    value = 0
    for c in msg: # skip over the $ at the begining of the sentence
        value ^= ord(c)
    return '%02x' % (value & 255)

class NmeaBridge(object):
    def __init__(self, server, gps, wind):
        self.process = False
        self.server = server
        self.gps = gps
        self.wind = wind
        self.process = NmeaBridgeProcess()
        self.process.start()

    def poll(self):
        data = False
        try:
            while True:
                data = self.pipe.recv()
        except IOError:
            return False

        if 'gps' in data:
            data = data['gps'] 
            # if internal gps track is more than 2 seconds old, use externally supplied gps
            if self.gps.source.value == 'external' or \
               time.time() - self.gps.last_update > 2:
                #print 'gps', name, 'val', value
                self.server.TimeStamp('gps', data['timestamp'])
                self.gps.track.set(data['track'])
                self.gps.speed.set(data['speed'])
                self.gps.source.update('external')
        elif 'wind' in data:
            data = data['wind']
            if self.wind.source.value != 'internal':
                self.server.TimeStamp('wind', time.time());
                self.wind.direction.set(data['direction'])
                self.wind.speed.set(data['speed'])
                self.wind.last_update = time.time()
                self.wind.source.update('external')
    
class NmeaBridgeProcess(multiprocessing.Process):
    def __init__(self):
        self.pipe, pipe = nonblockingpipe()
        super(NmeaBridgeProcess, self).__init__(target=nmea_bridge_process, args=(pipe,))


def nmea_bridge_process(pipe=False):
    import os
    sockets = []
    watchlist = ['ap.enabled', 'ap.mode', 'ap.heading_command', 'imu/pitch', 'imu/roll', 'imu/heading_lowpass', 'gps.source', 'wind.speed', 'wind.direction', 'wind.source']

    def setup_watches(client, watch=True):
        for name in watchlist:
            client.watch(name, watch)

    def on_con(client):
        print 'nmea client connected'
        if sockets:
            setup_watches(client)

    # we actually use a local connection to the server to simplify logic
    print 'nmea try connections'
    while True:
        try:
            client = SignalKClient(on_con, 'localhost', autoreconnect=True)
            break
        except:
            time.sleep(2)
    print 'nmea connected'

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setblocking(0)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    port = 10110
    try:
        server.bind(('0.0.0.0', port))
    except:
        print 'nmea_bridge: bind failed.'
        exit(1)
    print 'listening on port', port, 'for nmea connections'

    server.listen(5)
    max_connections = 10
    READ_ONLY = select.POLLIN | select.POLLHUP | select.POLLERR

    ap_enabled = 'N/A'
    ap_mode = 'N/A'
    ap_heading_command = 180
    addresses = {}
    cnt = 0

    poller = select.poll()
    poller.register(server, READ_ONLY)
    fd_to_socket = {server.fileno() : server}
    windspeed = 0

    gps_source = wind_source = False
    while True:
        if sockets:
            timeout = 100
        else:
            timeout = 10000
        events = poller.poll(timeout)
        while events:
            event = events.pop()
            fd, flag = event
            sock = fd_to_socket[fd]
            if sock == server:
                connection, address = sock.accept()
                if len(sockets) == max_connections:
                    connection.close()
                else:
                    if not sockets:
                        setup_watches(client)
                    sock = LineBufferedNonBlockingSocket(connection)
                    sockets.append(sock)
                    print 'new connection: ', address
                    addresses[sock] = address
                    fd = sock.socket.fileno()
                    fd_to_socket[fd] = sock

                    poller.register(sock.socket, READ_ONLY)

            elif (flag & (select.POLLHUP | select.POLLERR)) or \
                 (flag & select.POLLIN and not sock.recv()):
                print 'lost connection: ', addresses[sock]
                sockets.remove(sock)
#                addresses.remove(sock)
                if not sockets:
                    setup_watches(client, False)
                poller.unregister(sock.socket)
                fd = sock.socket.fileno()
                del fd_to_socket[fd]
                sock.socket.close()
#            elif flag & select.POLLOUT:
#                sock.flush()
#                if not sock.out_buffer:
#                    poller.register(sock.socket, READ_ONLY)

        for sock in sockets:
            line = sock.readline()
            if not line:
                continue

            if line[:6] == '$GPRMC':
                if pipe and gps_source != 'internal':
                    data = line[7:len(line)-3].split(',')
                    timestamp = float(data[0])
                    speed = float(data[6])
                    heading = float(data[7])
                
                    pipe.send({'gps' : {'timestamp': timestamp, 'track': heading, 'speed': speed}}, False)

            elif line[0] == '$' and line[3:6] == 'MVW':
                if pipe and wind_source != 'internal':
                    winddata = wind.parse_nmea(line)
                    if winddata:
                        pipe.send({'wind' : winddata}, False)

            elif line[0] == '$' and line[3:6] == 'APB':
                data = line[7:len(line)-3].split(',')
                if not ap_enabled:
                    client.set('ap.enabled', True)

                if ap_mode != 'gps':
                    client.set('ap.mode', 'gps')

                if abs(ap_heading_command - float(data[7])) > .1:
                    client.set('ap.heading_command', float(data[7]))

        msgs = client.receive()
        for name in msgs:
            data = msgs[name]
            value = data['value']

            msg = False
            if name == 'ap.enabled':
                ap_enabled = value
            elif name == 'ap.mode':
                ap_mode = value
            elif name == 'ap.heading_command':
                ap_heading_command = value
            elif name == 'imu/pitch':
                msg = 'APXDR,A,%.3f,D,PTCH' % value
            elif name == 'imu/roll':
                msg = 'APXDR,A,%.3f,D,ROLL' % value
            elif name == 'imu/heading_lowpass':
                msg = 'APHDM,%.3f,M' % value
            elif name == 'gps.source':
                gps_source = value
            elif name == 'wind.speed':
                windspeed = value
            elif name == 'wind.direction':
                msg = 'APMWV,%.1f,R,%.1f,K,A' % (value, windspeed)
            elif name == 'wind.source':
                wind_source = value

            if msg:
                msg = '$' + msg + '*' + cksum(msg) + '\r\n'
                for sock in sockets:
                    sock.send(msg)
                    sock.flush()

if __name__ == '__main__':
    nmea_bridge_process()
