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

def nonblockingpipe():
  import _multiprocessing, socket
  s = socket.socketpair()
  map(lambda t : t.setblocking(False), s)
  p = map(lambda t : _multiprocessing.Connection(os.dup(t.fileno())), s)
  s[0].close(), s[1].close()
  return p

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
        self.no_newline_pos = 0

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

        self.in_buffer += data

        l = len(data)

        if l == 0:
            return False
        if self.nonblocking and l == size:
            return l+self.recv()
        return l

    def readline(self):
        if self.no_newline_pos == len(self.in_buffer):
            return False
        pos = self.no_newline_pos
        for c in self.in_buffer:
            if c=='\n':
                ret = self.in_buffer[:pos]
                self.in_buffer = self.in_buffer[pos+1:]
                self.no_newline_pos = 0
                return ret
            pos += 1
        self.no_newline_pos = pos
        return False
    
class SignalKServer(object):
    def __init__(self, port=DEFAULT_PORT):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setblocking(0)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.port = port
        self.init = False
        self.sockets = []
        self.values = {}
        self.timestamps = {}

    def __del__(self):
        self.server_socket.close()
        for socket in self.sockets:
            socket.socket.close()

    def Register(self, value):
        if value.name in self.values:
            print 'warning, registering existing value:', value.name
        self.values[value.name] = value

        def make_send():
            def send():
                if value.need_persistent_store():
                    value.store_persistent()
                if value.watchers:
                    request = value.get_signalk() + '\n'
                    for socket in value.watchers:
                        socket.send(request)
            return send
        value.send = make_send()
        return value

    def TimeStamp(self, name, t=False):
        if not name in self.timestamps:
            self.timestamps[name] = [t, name]
        else:
            self.timestamps[name][0] = t
        return self.timestamps[name]

    def ListValues(self, socket):
        msg = {}
        for value in self.values:
            t = self.values[value].type()
            if type(t) == type(""):
                t = {'type' : t}
            msg[value] = t

        socket.send(json.dumps(msg) + '\n')

    def HandleNamedRequest(self, socket, data):
        method = data['method']
        name = data['name']
        value = self.values[name]

        if method == 'get':
            socket.send(value.get_signalk() + '\n')
        elif method == 'set':
            if value.client_can_set:
                value.set(data['value'])
            else:
                socket.send('value: ' + name + ' is readonly\n')
        elif method == 'watch':
            watch = data['value'] if 'value' in data else True
            if socket in value.watchers:
                if not watch:
                    value.watchers.remove(socket)
            elif watch:
                value.watchers.append(socket)
        else:
            socket.send('invalid method: ' + method + ' for ' + name + '\n')
        
    def HandleRequest(self, socket, request):
        try:
            data = json.loads(request)
        except:
            print 'invalid request', request
            socket.send('invalid request: ' + request + '\n')
            return
        if data['method'] == 'list':
            self.ListValues(socket)
        else:
            name = data['name']
            if not name in self.values:
                socket.send('invalid request: unknown value: ' + name + '\n')
            else:
                self.HandleNamedRequest(socket, data)

    def RemoveSocket(self, socket):
        self.sockets.remove(socket)
        fd = socket.socket.fileno()
        self.poller.unregister(socket.socket)

        socket.socket.close()

        del self.fd_to_socket[fd]

        for name in self.values:
            if socket in self.values[name].watchers:
                self.values[name].watchers.remove(socket)
    
    def HandleRequests(self, totaltime):
      READ_ONLY = select.POLLIN | select.POLLHUP | select.POLLERR
      if not self.init:
          try:
              self.server_socket.bind(('0.0.0.0', self.port))
          except:
              print 'signalk_server: bind failed, try again.'
              time.sleep(1)
              return

          self.server_socket.listen(5)
          self.init = True
          self.fd_to_socket = {self.server_socket.fileno() : self.server_socket}
          self.poller = select.poll()
          self.poller.register(self.server_socket, READ_ONLY)
        
      t1 = t2 = time.time()

      while t2 - t1 < totaltime:
        dt = t2 - t1
        if dt > totaltime:
          print 'time overflow, clock adjusted?'
          dt = totaltime / 2

        events = self.poller.poll(1000.0 * (totaltime - dt))
#        events = self.poller.poll(0)
        while events:
            event = events.pop()
            fd, flag = event
            socket = self.fd_to_socket[fd]
            if socket == self.server_socket:
                connection, address = socket.accept()
                print 'new client', address
                if len(self.sockets) == max_connections:
                    print 'max connections reached!!!', len(self.sockets)
                    self.RemoveSocket(self.sockets[0]) # dump first socket

                socket = LineBufferedNonBlockingSocket(connection)
                self.sockets.append(socket)
                fd = socket.socket.fileno()
                self.fd_to_socket[fd] = socket
                self.poller.register(socket.socket, READ_ONLY)
            elif flag & (select.POLLHUP | select.POLLERR):
                self.RemoveSocket(socket)
                continue
            elif flag & select.POLLIN:
                if not socket.recv():
                    self.RemoveSocket(socket)
            elif flag & select.POLLOUT:
                socket.flush()

        for socket in self.sockets:
            while True:
                line = socket.readline()
                if not line:
                    break
                self.HandleRequest(socket, line)

            socket.flush()            

        t2 = time.time()

if __name__ == '__main__':
    server = SignalKServer()
    print 'signalk demo server, try running signalk_client'
    clock = server.Register(Value('clock', 0))
    while True:
        clock.set(clock.value + 1)
        server.HandleRequests(.02)
