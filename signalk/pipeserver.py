#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# The pipe server communicates traffic via a pipe to
# offload socket and parsing work to background process

import time
from server import SignalKServer, DEFAULT_PORT
from values import *
import multiprocessing
import select


def nonblockingpipe():
  import _multiprocessing, socket
  s = socket.socketpair()
  map(lambda t : t.setblocking(False), s)
  p = map(lambda t : _multiprocessing.Connection(os.dup(t.fileno())), s)
  s[0].close(), s[1].close()
  return p

class SignalKPipeServerClient(SignalKServer):
    def __init__(self, pipe, port):
      super(SignalKPipeServerClient, self).__init__(port)
      self.watches = {}
      self.gets = {}
      self.pipe = pipe

      self.pipe_poller = select.poll()
      self.pipe_poller.register(pipe, select.POLLIN)

    def Register(self, value):
      super(SignalKPipeServerClient, self).Register(value)
      self.gets[value.name] = []
      try:
        value.timestamp = self.TimeStamp(value.timestamp)
      except:
        pass
      return value
    
    def RemoveSocket(self, socket):
      super(SignalKPipeServerClient, self).RemoveSocket(socket)
      for name in self.values:
          if not self.values[name].watchers and name in self.watches:
              self.pipe.send({'method': 'watch', 'name': name, 'value': False})
              del self.watches[name]

    def HandleNamedRequest(self, socket, data):
        method = data['method']
        name = data['name']
        value = self.values[name]

        if method == 'get':
          if name in self.watches: # already have recent value in this process
            socket.send(value.get_signalk() + '\n')
          else:
            self.gets[name].append(socket)
            self.pipe.send(data)
        elif method == 'set':
            self.pipe.send(data)
        elif method == 'watch':
          super(SignalKPipeServerClient, self).HandleNamedRequest(socket, data)
          watch = data['value'] if 'value' in data else True
          if watch:
            if not name in self.watches:
              self.watches[name] = True
              self.pipe.send({'method': 'watch', 'name': name, 'value': True})
          elif not value.watchers and name in self.watches:
            del self.watches[name]
            self.pipe.send({'method': 'watch', 'name': name, 'value': False})
        else:
          print 'unimplemented pipe method', method

    def HandlePipeMessage(self):
        if not self.pipe_poller.poll(0):
          return False

        msgs = self.pipe.recv()

        values = {}
        for name in msgs:
            value = msgs[name]
            if name == '_register':
                self.Register(value)
            elif name in self.timestamps:
                self.TimeStamp(name, value)
            else:
                values[name] = value

        # send values once all potential timestamps are received
        # TODO: benchmark without timestamp
        for name in values:
          if False:
            self.values[name].set(values[name])
            
          else:
            value = self.values[name]
            value.value = values[name]
            self.values[name].send()

          if self.gets[name]:
              response = self.values[name].get_signalk() + '\n'
              for socket in self.gets[name]:
                  socket.send(response)
              self.gets[name] = []
        return True

def pipe_server_process(pipe, port):
    print 'pipe server on', os.getpid()
    server = SignalKPipeServerClient(pipe, port)
    # handle only pipe messages (to get all registrations) for first second
    t0 = time.time()
    while time.time() - t0 < 2:
      while server.HandlePipeMessage():
        pass
      time.sleep(.1)

    while True:
      while server.HandlePipeMessage():
        pass
      
      server.HandleRequests(.1)


class SignalKPipeServer:
    def __init__(self, port=DEFAULT_PORT):
        self.pipe, process_pipe = nonblockingpipe()
        self.process = multiprocessing.Process(target=pipe_server_process, args=(process_pipe, port))
        self.process.start()
        self.values = {}
        self.sets = {}
        self.timestamps = {}
        self.last_recv = time.time()

        self.poller = select.poll()
        READ_ONLY = select.POLLIN | select.POLLHUP | select.POLLERR
        self.poller.register(self.pipe, READ_ONLY)
          
    def __del__(self):
        self.process.terminate()

    def Register(self, value):
        self.pipe.send({'_register': value})
        self.values[value.name] = value

        def make_send():
            def send():
                if value.watchers or value.need_persistent_store():
                  if value.timestamp in self.timestamps:
                    self.sets[value.timestamp] = self.timestamps[value.timestamp]
                  self.sets[value.name] = value.value
            return send
        value.send = make_send()
        return value

    def TimeStamp(self, name, t=False):
        self.timestamps[name] = t
        return name

    def HandleRequest(self, request):
      method = request['method']
      name = request['name']

      if method == 'get':
        value = self.values[name]
        if value.timestamp:
          self.sets[value.timestamp] = self.timestamps[value.timestamp]
        self.sets[name] = value.value
      elif method == 'set':
        self.values[name].set(request['value'])
        self.sets[name] = self.values[name].value
      elif method == 'watch':
          self.values[name].watchers = request['value']
        
    def HandleRequests(self, totaltime):            
        t0 = time.time()
        dt = totaltime
        while dt >= 0:
          if self.sets:
            self.pipe.send(self.sets)
            self.sets = {}
            dt = totaltime - (time.time()-t0)
  
          if self.poller.poll(1000.0 * dt):
              self.HandleRequest(self.pipe.recv())
          dt = totaltime - (time.time()-t0)
    
if __name__ == '__main__':
    print 'pipe server demo'
    server = SignalKPipeServer()
#    server = SignalKServer()
    test_sensor = server.Register(SensorValue('sensor', server.TimeStamp('testtime')))
    clock = server.Register(Value('clock', 0))
    test_property = server.Register(Property('test_property', 100))
    test_range = server.Register(RangeProperty('test_range', 1, 0, 10))
    test_enum = server.Register(EnumProperty('test_enum', 'a', ['a', 'b', 'c']))
    test_boolean = server.Register(BooleanProperty('test_boolean', False))
    while True:
        clock.set(clock.value + 1)
        server.TimeStamp('testtime', time.time())
        test_sensor.set(test_sensor.value+1)
        server.HandleRequests(.1)
