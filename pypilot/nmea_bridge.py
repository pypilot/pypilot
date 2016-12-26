#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
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


import sys, select, socket
from signalk.client import SignalKClient
from signalk.server import LineBufferedNonBlockingSocket

# nmea uses a simple xor checksum
def cksum(msg):
    value = 0
    for c in msg: # skip over the $ at the begining of the sentence
        value ^= ord(c)
    return '%02x' % (value & 255)

if __name__ == '__main__':
    sockets = []
    watchlist = ['ap/mode', 'ap/heading_command', 'imu/pitch', 'imu/roll', 'imu/heading_lowpass']

    def setup_watches(client, watch=True):
        for name in watchlist:
            client.watch(name, watch)

    def on_con(client):
        if sockets:
            watch(client)

    host = False
    if len(sys.argv) > 1:
        host = sys.argv[1]

    client = SignalKClient(on_con, host)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setblocking(0)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server.bind(('0.0.0.0', 12345))
    except:
        print 'nmea_bridge: bind failed.'
        exit(1)

    server.listen(5)
    max_connections = 10
    READ_ONLY = select.POLLIN | select.POLLPRI | select.POLLHUP | select.POLLERR

    ap_mode = 'disabled'
    ap_heading_command = 180
    addresses = {}

    while True:
        poller = select.poll()
        poller.register(server, READ_ONLY)
        fd_to_socket = {server.fileno() : server}
        for socket in sockets:
            flags = READ_ONLY
            if socket.out_buffer != '':
                flags |= select.POLLOUT
            fd = socket.socket.fileno()
            poller.register(fd, flags)
            fd_to_socket[fd] = socket

        events = poller.poll(50) # 50 milliseconds
        while events:
            event = events.pop()
            fd, flag = event
            socket = fd_to_socket[fd]
            if socket == server:
                connection, address = socket.accept()
                if len(sockets) == max_connections:
                    connection.close()
                else:
                    if not sockets:
                        setup_watches(client)
                    socket = LineBufferedNonBlockingSocket(connection)
                    sockets.append(socket)
                    print 'new connection: ', address
                    addresses[socket] = address
            elif (flag & (select.POLLHUP | select.POLLERR)) or \
                 (flag & select.POLLIN and not socket.recv()):
                print 'lost connection: ', addresses[socket]
                sockets.remove(socket)
#                addresses.remove(socket)
                if not sockets:
                    setup_watches(client, False)
                socket.socket.close()
            elif flag & select.POLLOUT:
                socket.flush()

        for socket in sockets:
            line = socket.readline()
            if not line:
                continue

            if line[:6] == '$GPRMC':
                data = line[7:len(line)-3].split(',')
                speed = float(data[6])
                heading = float(data[7])
                #client.set('gps/heading', heading)
            elif line[3:6] == 'APB':
                data = line[7:len(line)-3].split(',')

                if ap_mode != 'gps':
                    client.set('ap/mode', 'gps')

                if abs(ap_heading_command - float(data[7])) > .1:
                    client.set('ap/heading_command', float(data[7]))

        msgs = client.receive()
        for name in msgs:
            data = msgs[name]
            value = data['value']

            msg = False
            if name == 'ap/mode':
                ap_mode = value
            elif name == 'ap/heading_command':
                ap_heading_command = value
            elif name == 'imu/pitch':
                msg = 'APXDR,A,%.3f,D,PTCH' % value
            elif name == 'imu/roll':
                msg = 'APXDR,A,%.3f,D,ROLL' % value
            elif name == 'imu/heading_lowpass':
                msg = 'APHDM,%.3f,M' % value

            if msg:
                msg = '$' + msg + '*' + cksum(msg) + '\r\n'
                for socket in sockets:
                    socket.send(msg)
