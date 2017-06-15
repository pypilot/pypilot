#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# the pipe server communicates traffic via a pipe to
# offload socket and parsing work to a separate process

import time
from server import SignalKServer, DEFAULT_PORT, default_persistent_path
from values import *
import multiprocessing
import select

class NonBlockingPipeEnd(object):
    def __init__(self, pipe, name, recvfailok):
        self.pipe = pipe
        self.pollin = select.poll()
        self.pollin.register(self.pipe, select.POLLIN)
        self.pollout = select.poll()
        self.pollout.register(self.pipe, select.POLLOUT)
        self.name = name
        self.sendfailcount = 0
        self.failcountmsg = 1
        self.recvfailok = recvfailok

    def fileno(self):
      return self.pipe.fileno()
        
    def recv(self, timeout=0):
        if self.pollin.poll(1000.0*timeout):
            return self.pipe.recv()
        if not self.recvfailok:
            print 'error pipe block on recv!', self.name
        return False

    def send(self, value, block=True):
        if block or self.pollout.poll(0):
            self.pipe.send(value)
            return True
        
        self.sendfailcount += 1
        if self.sendfailcount == self.failcountmsg:
            print 'pipe full (%d)' % self.sendfailcount, self.name, 'cannot send'
            self.failcountmsg *= 10
        return False


def NonBlockingPipe(name, recvfailok=False):
  pipe = multiprocessing.Pipe()
  return NonBlockingPipeEnd(pipe[0], name+'[0]', recvfailok), NonBlockingPipeEnd(pipe[1], name+'[1]', recvfailok)

class SignalKPipeServerClient(SignalKServer):
    def __init__(self, pipe, port, persistent_path):
      super(SignalKPipeServerClient, self).__init__(port, persistent_path)
      self.watches = {}
      self.gets = {}
      self.pipe = pipe

    def __del__(self):
      while self.HandlePipeMessage():
        pass
      super(SignalKPipeServerClient, self).__del__()

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
        msgs = self.pipe.recv()
        if not msgs:
            return False

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

def pipe_server_process(pipe, port, persistent_path):
    #print 'pipe server on', os.getpid()
    server = SignalKPipeServerClient(pipe, port, persistent_path)
    # handle only pipe messages (to get all registrations) for first second
    t0 = time.time()
    while time.time() - t0 < 2:
      while server.HandlePipeMessage():
        pass
      time.sleep(.1)

    while True:
        while server.HandlePipeMessage():
            pass
        server.HandleRequests()
        time.sleep(.1)


class SignalKPipeServer(object):
    def __init__(self, port=DEFAULT_PORT, persistent_path=default_persistent_path):
        #self.pipe, process_pipe = multiprocessing.Pipe()
        self.pipe, process_pipe = NonBlockingPipe('signalkpipeserver', True)
    
        self.process = multiprocessing.Process(target=pipe_server_process, args=(process_pipe, port, persistent_path))
        self.process.start()
        self.values = {}
        self.sets = {}
        self.timestamps = {}
        self.last_recv = time.time()
        
        self.persistent_path = persistent_path
        self.ResetPersistentState()
        self.LoadPersistentValues()

    def LoadPersistentValues(self): # unfortunately duplicated from server
        try:
            file = open(self.persistent_path)
            self.persistent_data = json.loads(file.readline())
            file.close()
        except:
            print 'failed to load', self.persistent_path
            self.persistent_data = {}
          
    def __del__(self):
      # ensure persistent values get sent to server process
      self.SetPersistentValues()
      self.pipe.send(self.sets, False)
      self.process.terminate()
        
    def SetPersistentValues(self):
      for name in self.persistent_sets:
        if self.persistent_sets[name]:
          self.sets[name] = self.values[name].value
      self.ResetPersistentState()

    def ResetPersistentState(self):
      self.persistent_timeout = time.time()+600
      self.persistent_sets = {}

    def queue_send(self, value):
      if value.timestamp:
          self.sets[value.timestamp] = self.timestamps[value.timestamp]
      self.sets[value.name] = value.value
      if value.persistent:
        self.persistent_sets[value.name] = False
        
    def Register(self, value):
        if value.persistent and value.name in self.persistent_data:
            value.value = self.persistent_data[value.name]
      
        self.pipe.send({'_register': value})
        self.values[value.name] = value

        def make_send():
            def send():
              if value.watchers:
                self.queue_send(value)
              elif value.persistent:
                self.persistent_sets[value.name] = True
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
        self.queue_send(self.values[name])
      elif method == 'set':
        self.values[name].set(request['value'])
        self.queue_send(self.values[name])
      elif method == 'watch':
          self.values[name].watchers = request['value']
        
    def HandleRequests(self):
        t0 = time.time()
        if t0 >= self.persistent_timeout:
            self.SetPersistentValues()

        if self.sets:
            ta = time.time()
            # should we break up sets if there are many!?!
            l = len(self.sets)
            if l > 20:
                setnames = list(self.sets)
                while setnames:
                    sets = {}
                    for i in range(20): # send 20 values at a time
                        if not setnames:
                            break
                        name = setnames.pop()
                        sets[name] = self.sets[name]
                    if not self.pipe.send(sets, False):
                        break
                    for name in sets:
                        del self.sets[name]
            else:
                if self.pipe.send(self.sets, False):
                    self.sets = {}

            dta = time.time() - ta
            if dta > .02:
              print 'too long to send sets down pipe', dta, l

        while True:
            request = self.pipe.recv()
            if request:
                self.HandleRequest(request)
            else:
                break
    
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
