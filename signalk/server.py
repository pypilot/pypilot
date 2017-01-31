#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import select, socket, time, json

from values import *

DEFAULT_PORT = 21311
max_connections = 20

class LineBufferedNonBlockingSocket():
    def __init__(self, connection):
        if type(connection) == type(socket.socket()):
            connection.setblocking(0)
            self.nonblocking = True
        else:           
            self.nonblocking = False

        self.socket = connection
        self.in_buffer = ''
        self.out_buffer = ''
        self.no_newline = True

    def send(self, data):
        self.out_buffer += data

    def flush(self):
        if not len(self.out_buffer):
            return
        try:
            if self.nonblocking:
                count = self.socket.send(self.out_buffer)
            else:
                count = self.socket.write(self.out_buffer[0])

            self.out_buffer = self.out_buffer[count:]
        except:
            self.socket.close()

    def recv(self):
        if self.nonblocking:
            size = 4096
            data = self.socket.recv(size)
        else:
            size = 1
            data = self.socket.read()

        self.no_newline = False
        self.in_buffer += data
        l = len(data)
        if l == 0:
            return False
        if self.nonblocking and l == size:
            return l+self.recv()
        return l

    def readline(self):
        if self.no_newline:
            return False
        pos = 0
        for c in self.in_buffer:
            if c=='\n':
                ret = self.in_buffer[:pos]
                self.in_buffer = self.in_buffer[pos+1:]
                return ret
            pos += 1
        self.no_newline = True
        return False

class SignalKServer:
    def __init__(self, port=DEFAULT_PORT):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setblocking(0)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.port = port
        self.init = False
        self.sockets = []
        self.values = {}

    def __del__(self):
        self.server_socket.close()
        for socket in self.sockets:
            socket.socket.close()

    def Register(self, value):
        if value.name in self.values:
            print 'warning, registering existing value:', value.name
        self.values[value.name] = value
        return value

    def SendAllClients(self, msg):
        for socket in self.sockets:
            socket.send(json.dumps(msg) + '\n')

    def RemoveSocket(self, socket):
        self.sockets.remove(socket)
        socket.socket.close()

        for value in self.values:
            if socket in self.values[value].watchers:
                self.values[value].watchers.remove(socket)
            
    def ListValues(self, socket):
        msg = {}
        for value in self.values:
            t = self.values[value].type()
            if type(t) == type(""):
                t = {'type' : t}
            msg[value] = t

        socket.send(json.dumps(msg) + '\n')
    
    def HandleRequest(self, socket, request):
        data = json.loads(request)
        method = data['method']
        if method == 'list':
            self.ListValues(socket)
        else:
            name = data['name']
            if not name in self.values:
                socket.send('invalid request name: ' + name + '\n')
            else:
                value = self.values[name]

                processes = value.processes()
                if method in processes:
                    processes[method](socket, data)
                else:
                    socket.send('invalid method: ' + method + ' for ' + name + '\n')
    
    def HandleRequests(self, totaltime):
        if not self.init:
            try:
                self.server_socket.bind(('0.0.0.0', self.port))
            except:
                print 'signalk_server: bind failed, try again.'
                time.sleep(1)
                return

            self.server_socket.listen(5)
            self.init = True
        
        t1 = t2 = time.time()
        READ_ONLY = select.POLLIN | select.POLLPRI | select.POLLHUP | select.POLLERR
        while t2 - t1 < totaltime:
            poller = select.poll()
            poller.register(self.server_socket, READ_ONLY)
            fd_to_socket = {self.server_socket.fileno() : self.server_socket}
            for socket in list(self.sockets):
                flags = READ_ONLY
                if socket.out_buffer != '':
                    flags |= select.POLLOUT
                try:
                    fd = socket.socket.fileno()
                    poller.register(fd, flags)
                    fd_to_socket[fd] = socket
                except:
                    self.RemoveSocket(socket)
                    
            events = poller.poll(1000.0 * (totaltime - (t2 - t1)))
            while t2 - t1 < totaltime and events != []:
                event = events.pop()
                fd, flag = event
                socket = fd_to_socket[fd]
                if socket == self.server_socket:
                    connection, address = socket.accept()
                    if len(self.sockets) == max_connections:
                        print 'max connections reached!!!', len(self.sockets)
                        if True:
                            self.RemoveSocket(self.sockets[0])
                        else:
                            connection.close()
                    else:
                        self.sockets.append(LineBufferedNonBlockingSocket(connection))
                        #print 'connections: ', len(self.sockets)
                elif flag & (select.POLLHUP | select.POLLERR):
                    self.RemoveSocket(socket)
                    continue
                elif flag & select.POLLIN:
                    if not socket.recv():
                        self.RemoveSocket(socket)
                elif flag & select.POLLOUT:
                    socket.flush()

                t2 = time.time()

            for socket in self.sockets:
                line = socket.readline()
                if not line:
                    continue

                if False: # true to debug
                    self.HandleRequest(socket, line)
                else:
                    try:
                        self.HandleRequest(socket, line)
                    except:
                        socket.send('invalid request: ' + line + '\n')

                if t2 - t1 >= totaltime:
                    return
                t2 = time.time()
            t2 = time.time()

if __name__ == '__main__':
    server = SignalKServer()
    timeout = 1000
    print 'signalk demo server, try running signalk_client.py'
    server.Register(Property('test', 0))
    clock = Property('clock', 0)
    server.Register(clock)
#    try:
    if True:
        while True:
            # handle requests for 1 second, then increment clock 1 second
            server.HandleRequests(1)
            clock.set(clock.value + 1)
#    except:
#        pass
