#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import select, socket, time, kjson
import fcntl, os
from values import *

DEFAULT_PORT = 21311
max_connections = 20
default_persistent_path = os.getenv('HOME') + '/.pypilot/pypilot.conf'

from signalk.linebuffer import linebuffer
#class LineBufferedNonBlockingSocket(linebuffer.LineBuffer):
class LineBufferedNonBlockingSocket(object):
    def __init__(self, connection):
        connection.setblocking(0)
        #//fcntl.fcntl(connection.fileno(), fcntl.F_SETFD, os.O_NONBLOCK)
        # somehow it's much slower to baseclass ?!?
        #super(LineBufferedNonBlockingSocket, self).__init__(connection.fileno())
        self.b = linebuffer.LineBuffer(connection.fileno())

        self.socket = connection
        self.out_buffer = ''
        self.pollout = select.poll()
        self.pollout.register(connection, select.POLLOUT)
        self.sendfail_msg = self.sendfail_cnt = 0

    def recv(self):
        return self.b.recv()
        
    def readline(self):
        return self.b.line()

    def send(self, data):
        self.out_buffer += data
        if len(self.out_buffer) > 65536:
            self.out_buffer = data
            print 'overflow in signalk socket', len(data)
    
    def flush(self):
        if not self.out_buffer:
            return
        try:
            if not self.pollout.poll(0):
                if sendfail_cnt >= sendfail_msg:
                    print 'signalk socket failed to send', sendfail_cnt
                    self.sendfail_msg *= 10
                self.sendfail_cnt += 1
                return
            t0 = time.time()
            count = self.socket.send(self.out_buffer)
            t1 = time.time()
            if t1-t0 > .1:
                print 'socket send took too long!?!?', t1-t0
            if count < 0:
                print 'socket send error', count
                self.socket.close()
            self.out_buffer = self.out_buffer[count:]
        except:
            self.socket.close()

class LineBufferedNonBlockingSocketPython(object):
    def __init__(self, connection):
        connection.setblocking(0)
        self.socket = connection
        self.in_buffer = ''
        self.no_newline_pos = 0
        self.out_buffer = ''

    def send(self, data):
        self.out_buffer += data

    def flush(self):
        if not len(self.out_buffer):
            return
        try:
            count = self.socket.send(self.out_buffer)
            if count < 0:
                print 'socket send error in server flush'
                self.out_buffer = ''
                self.socket.close()
                return

            self.out_buffer = self.out_buffer[count:]
        except:
            self.out_buffer = ''
            self.socket.close()

    def recv(self):
        size = 4096
        data = self.socket.recv(size)

        l = len(data)
        if l == 0:
            return False

        self.in_buffer += data
        if l == size:
            return l+self.recv()
        return l

    def readline(self):
        while self.no_newline_pos < len(self.in_buffer):
            c = self.in_buffer[self.no_newline_pos]
            if c=='\n':
                ret = self.in_buffer[:self.no_newline_pos]
                self.in_buffer = self.in_buffer[self.no_newline_pos+1:]
                self.no_newline_pos = 0
                return ret
            self.no_newline_pos += 1
        return ''

def LoadPersistentData(persistent_path, server=True):
    try:
        file = open(persistent_path)
        persistent_data = kjson.loads(file.readline())
        file.close()
    except Exception as e:
        print 'failed to load', persistent_path, e
        print 'WARNING Alignment and other data lost!!!!!!!!!'

        if server:
            # log failing to load persistent data
            persist_fail = os.getenv('HOME') + '/.pypilot/persist_fail'
            file = open(persist_fail, 'a')
            file.write(str(time.time()) + ' ' + str(e) + '\n')
            file.close()

        try:
            file = open(persistent_path + '.bak')
            persistent_data = kjson.loads(file.readline())
            file.close()
            return persistent_data
        except Exception as e:
            print 'backup data failed as well', e
        return {}

    if server:
        # backup persistent_data if it loaded with success
        file = open(persistent_path + '.bak', 'w')
        file.write(kjson.dumps(persistent_data)+'\n')
        file.close()
    return persistent_data
    
class SignalKServer(object):
    def __init__(self, port=DEFAULT_PORT, persistent_path=default_persistent_path):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setblocking(0)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.port = port
        self.init = False
        self.sockets = []
        self.values = {}
        self.timestamps = {}

        self.persistent_path = persistent_path
        self.persistent_timeout = time.time() + 300
        self.persistent_data = LoadPersistentData(persistent_path)

    def __del__(self):
        self.StorePersistentValues()
        self.server_socket.close()
        for socket in self.sockets:
            socket.socket.close()
            
    def StorePersistentValues(self):
        self.persistent_timeout = time.time() + 600 # 10 minutes
        need_store = False
        for name in self.values:
            value = self.values[name]
            if not value.persistent:
                continue
            if not name in self.persistent_data or value.value != self.persistent_data[name]:
                self.persistent_data[name] = value.value
                need_store = True

        if not need_store:
            return
                
        try:
            file = open(self.persistent_path, 'w')
            file.write(kjson.dumps(self.persistent_data)+'\n')
            file.close()
        except Exception as e:
            print 'failed to write', self.persistent_path, e

    def Register(self, value):
        if value.persistent and value.name in self.persistent_data:
            value.value = self.persistent_data[value.name]
            #print 'persist', value.name, ' = ', value.value

        if value.name in self.values:
            print 'warning, registering existing value:', value.name
            
        self.values[value.name] = value
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
            if type(t) == type(''):
                t = {'type' : t}
            msg[value] = t

        socket.send(kjson.dumps(msg) + '\n')

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
        data = kjson.loads(request)
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

        found = False
        for fd in self.fd_to_socket:
            if socket == self.fd_to_socket[fd]:
                del self.fd_to_socket[fd]
                self.poller.unregister(fd)
                found = True
                break

        socket.socket.close()

        if not found:
            print 'socket not found in fd_to_socket'

        for name in self.values:
            if socket in self.values[name].watchers:
                self.values[name].watchers.remove(socket)

    def PollSockets(self):
        events = self.poller.poll(0)
        while events:
            event = events.pop()
            fd, flag = event
            socket = self.fd_to_socket[fd]
            if socket == self.server_socket:
                connection, address = socket.accept()
                if len(self.sockets) == max_connections:
                    print 'max connections reached!!!', len(self.sockets)
                    self.RemoveSocket(self.sockets[0]) # dump first socket??

                socket = LineBufferedNonBlockingSocket(connection)
                self.sockets.append(socket)
                fd = socket.socket.fileno()
                # print 'new client', address, fd
                self.fd_to_socket[fd] = socket
                self.poller.register(fd, select.POLLIN)
            elif flag & (select.POLLHUP | select.POLLERR | select.POLLNVAL):
                self.RemoveSocket(socket)
            elif flag & select.POLLIN:
                if not socket.recv():
                    self.RemoveSocket(socket)
                while True:
                    line = socket.readline()
                    if not line:
                        break
                    try:
                        self.HandleRequest(socket, line)
                    except Exception as e:
                        print 'invalid request from socket', line, e
                        socket.send('invalid request: ' + line + '\n')

        # flush all sockets
        for socket in self.sockets:
            socket.flush()
                
    def HandleRequests(self):
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
          self.poller.register(self.server_socket, select.POLLIN)
        
      t1 = time.time()
      #print 'store', t1 - self.persistent_timeout
      if t1 >= self.persistent_timeout:
          self.StorePersistentValues()
          if time.time() - t1 > .1:
              print 'persistent store took too long!', time.time() - t1
              return

      self.PollSockets()

if __name__ == '__main__':
    server = SignalKServer()
    print 'signalk demo server, try running signalk_client'
    clock = server.Register(Value('clock', 0))
    test = server.Register(Property('test', 1234))
    while True:
        clock.set(clock.value + 1)
        server.HandleRequests()
        time.sleep(.02)
