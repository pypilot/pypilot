#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from __future__ import print_function
import select, socket, time, numbers
from pypilot import pyjson
import fcntl, os
from pypilot.values import *
from pypilot.bufferedsocket import LineBufferedNonBlockingSocket

DEFAULT_PORT = 21311
max_connections = 30
default_persistent_path = os.getenv('HOME') + '/.pypilot/pypilot.conf'

def LoadPersistentData(persistent_path, server=True):
    try:
        file = open(persistent_path)
        persistent_data = pyjson.loads(file.read())
        file.close()
    except Exception as e:
        print('failed to load', persistent_path, e)

        if server:
            # log failing to load persistent data
            persist_fail = os.getenv('HOME') + '/.pypilot/persist_fail'
            file = open(persist_fail, 'a')
            file.write(str(time.time()) + ' ' + str(e) + '\n')
            file.close()

        try:
            file = open(persistent_path + '.bak')
            persistent_data = pyjson.loads(file.read())
            file.close()
            return persistent_data
        except Exception as e:
            print('backup data failed as well', e)
        return {}

    if server:
        # backup persistent_data if it loaded with success
        file = open(persistent_path + '.bak', 'w')
        file.write(pyjson.dumps(persistent_data)+'\n')
        file.close()
    return persistent_data
    
class pypilotServer(object):
    def __init__(self, port=DEFAULT_PORT, persistent_path=default_persistent_path):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setblocking(0)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.port = port
        self.init = False
        self.sockets = []
        self.values = {}

        self.persistent_path = persistent_path
        self.persistent_timeout = time.time() + 300
        self.persistent_data = LoadPersistentData(persistent_path)

    def __del__(self):
        self.StorePersistentValues()
        self.server_socket.close()
        for socket in self.sockets:
            socket.socket.close()
            
    def StorePersistentValues(self):
        self.persistent_timeout = time.time() + 30 # 30 seconds
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
            file.write(pyjson.dumps(self.persistent_data).replace(',', ',\n')+'\n')
            file.close()
        except Exception as e:
            print('failed to write', self.persistent_path, e)

    def Register(self, value):
        if value.persistent and value.name in self.persistent_data:
            v = self.persistent_data[value.name]
            if isinstance(v, numbers.Number):
                v = float(v) # convert any numeric to floating point
            value.set(v) # set persistent value

        if value.name in self.values:
            print('warning, registering existing value:', value.name)
            
        self.values[value.name] = value
        return value

    def ListValues(self, socket):
        msg = {}
        for value in self.values:
            t = self.values[value].type()
            if type(t) == type(''):
                t = {'type' : t}
            msg[value] = t
        socket.send(pyjson.dumps(msg) + '\n')

    def HandleNamedRequest(self, socket, data):
        method = data['method']
        name = data['name']
        value = self.values[name]

        if method == 'get':
            socket.send(value.get_pypilot() + '\n')
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
        data = pyjson.loads(request)
        if data['method'] == 'list':
            self.ListValues(socket)
        else:
            name = data['name']
            if not name in self.values:
                socket.send('invalid request: ' + data['method'] + ' unknown value: ' + name + '\n')
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
            print('socket not found in fd_to_socket')

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
                    print('pypilot server: max connections reached!!!', len(self.sockets))
                    self.RemoveSocket(self.sockets[0]) # dump first socket??

                socket = LineBufferedNonBlockingSocket(connection)
                self.sockets.append(socket)
                fd = socket.socket.fileno()
                # print('new client', address, fd)
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
                        print('invalid request from socket', line, e)
                        socket.send('invalid request: ' + line + '\n')

        # flush all sockets
        for socket in self.sockets:
            socket.flush()
                
    def HandleRequests(self):
      if not self.init:
          try:
              self.server_socket.bind(('0.0.0.0', self.port))
          except:
              print('pypilot_server: bind failed; already running a server?')
              time.sleep(1)
              return

          self.server_socket.listen(5)
          self.init = True
          self.fd_to_socket = {self.server_socket.fileno() : self.server_socket}
          self.poller = select.poll()
          self.poller.register(self.server_socket, select.POLLIN)
        
      t1 = time.time()
      #print('store', t1 - self.persistent_timeout)
      if t1 >= self.persistent_timeout:
          self.StorePersistentValues()
          if time.time() - t1 > .1:
              print('persistent store took too long!', time.time() - t1)
              return

      self.PollSockets()

if __name__ == '__main__':
    server = pypilotServer()
    print('pypilot demo server, try running pypilot_client')
    clock = server.Register(Value('clock', 0))
    test = server.Register(Property('test', 1234))
    while True:
        clock.set(clock.value + 1)
        server.HandleRequests()
        time.sleep(.02)
