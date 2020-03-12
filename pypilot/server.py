#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from __future__ import print_function
import select, socket, time, numbers
import sys, os, heapq
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import pyjson
from bufferedsocket import LineBufferedNonBlockingSocket
from nonblockingpipe import LineBufferedNonBlockingPipe

DEFAULT_PORT = 23322
max_connections = 30
default_persistent_path = os.getenv('HOME') + '/.pypilot/pypilot.conf'
server_persistent_period = 60 # store data every 60 seconds
use_multiprocessing = True # run server in a separate process

class Watch(object):
    def __init__(self, value, connection, period):
        self.value = value
        self.connections = [connection]
        self.period = period
        self.time = 0 # send first update immediate

class pypilotValue(object):
    def __init__(self, values, name, info={}, connection=False, msg=False):
        self.values = values
        self.name = name
        self.info = info
        self.lasttime = time.monotonic()
        self.connection = connection
        self.watching = False # is False, or the period, so 0 rather than True

        self.awatches = [] # all watches
        self.pwatches = [] # periodic watches limited in period
        self.msg = msg

    def get_msg(self):
        return self.msg
        
    def set(self, msg, connection):
        t0 = time.monotonic()
        if self.connection == connection:
            # received new value from owner, inform watchers
            self.msg = msg

            if self.awatches:
                watch = self.awatches[0]
                if watch.period == 0:
                    for connection in watch.connections:
                        connection.send(msg)

                for watch in self.pwatches:
                    if t0 >= watch.time:
                        watch.time = t0
                    self.values.insert_watch(watch)
                self.pwatches = []

        elif self.connection: # inform owner of change if we are not owner
            if 'writable' in self.info and self.info['writable']:
                self.connection.send(msg)
            else: # inform key can not be set arbitrarily
                connection.send('error='+self.name+' is not writable\n')

    def remove_watches(self, connection):
        for watch in self.awatches:
            if connection in watch.connections:
                watch.connections.remove(connection)
                if not watch.connections:
                    self.awatches.remove(watch)
                    self.calculate_watch_period()
                break
            
    def calculate_watch_period(self):
        # find minimum watch period from all watches
        watching = False
        if 'persistent' in self.info and self.info['persistent']:
            watching = server_persistent_period
        for watch in self.awatches:
            if len(watch.connections) == 0:
                print("ERRORRRR  no connections in watch")
            if watching is False or watch.period < watching:
                watching = watch.period
                
        if watching is not self.watching:
            self.watching = watching
            if watching == 0:
                watching = True
            if self.connection:
                self.connection.cwatches[self.name] = watching

    def unwatch(self, connection, recalc):
        for watch in self.awatches:
            if connection in watch.connections:
                watch.connections.remove(connection)
                if not watch.connections:
                    self.awatches.remove(watch)
                    if recalc and watch.period == period:
                        self.calculate_watch_period()
                return True
        return False
            
    def watch(self, connection, period):
        if connection == self.connection:
            connection.send('error=can not add watch for own value: ' + self.name + '\n')
            return
        
        if period is False: # period is False: remove watch
            if not self.unwatch(connection, True):
                # inform client there was no watch
                connection.send('error=cannot remove unknown watch for ' + self.name + '\n')
        else:
            if period is True:
                period = 0 # True is same as a period of 0, for fastest watch

            # unwatch by removing
            self.unwatch(connection, False)
            for watch in self.awatches:
                if watch.period == period: # already watching at this rate, add connection
                    watch.connections.append(connection)
                    if period > self.watching: # only need to update if period is relaxed
                        self.calculate_watch_period()
                    return

            # need a new watch for this unique period
            watch = Watch(self, connection, period)
            if period == 0: # make sure period 0 is always at start of list
                self.awatches.insert(0, watch)
            else:
                self.awatches.append(watch)
            self.calculate_watch_period()
            if period:
                self.pwatches.append(watch)

            if self.msg: # for initial retrieval
                connection.send(self.get_msg())

class ServerWatch(pypilotValue):
    def __init__(self, values):
        super(ServerWatch, self).__init__(values, 'watch', {}, None)
        self.values = values

    def set(self, msg, connection):
        name, data = msg.rstrip().split('=', 1)        
        watches = pyjson.loads(data)
        values = self.values.values
        for name in watches:
            if not name in values:
                # watching value not yet registered, add it so we can watch it
                values[name] = pypilotValue(self.values, name)
            values[name].watch(connection, watches[name])

class ServerValues(pypilotValue):
    def __init__(self):
        super(ServerValues, self).__init__(self, 'values', {}, None)
        self.values = {'values': self, 'watch': ServerWatch(self)}
        self.pipevalues = {}
        self.msg = 'new'
        self.load()
        self.pqwatches = [] # priority queue of watches
        self.last_send_watches = 0
        self.persistent_timeout = time.monotonic() + server_persistent_period

    def get_msg(self):
        if not self.msg or self.msg == 'new':
            msg = 'values={'
            notsingle = False
            for name in self.values:
                if name == 'values' or name == 'watch':
                    continue
                if notsingle:
                    msg += ','
                msg += '"' + name + '":' + pyjson.dumps(self.values[name].info)
                notsingle = True
            self.msg = msg + '}\n'
        return self.msg

    def sleep_time(self):
        # sleep until the first value in heap is ready
        if not self.pqwatches:
            return None
        return self.pqwatches[0][0] - time.monotonic()
            
    def send_watches(self):
        t0 = time.monotonic()
        while self.pqwatches:
            if t0 < self.pqwatches[0][0]:
                break # no more are ready
            t, i, watch = heapq.heappop(self.pqwatches) # pop first element
            if not watch.connections:
                continue # forget this watch
            for connection in watch.connections:
                connection.send(watch.value.get_msg())
            watch.time += watch.period
            watch.value.pwatches.append(watch) # put back on value periodic watch list
            
    def insert_watch(self, watch):
        heapq.heappush(self.pqwatches, (watch.time, time.monotonic(), watch))

    def remove(self, connection):
        for name in self.values:
            value = self.values[name]
            if value.connection == connection:
                value.connection = False
                continue
            value.remove_watches(connection)
            
    def set(self, msg, connection):
        name, data = msg.rstrip().split('=', 1)        
        values = pyjson.loads(data)
        for name in values:
            info = values[name]
            if name in self.values:
                value = self.values[name]
                if value.connection:
                    connection.send('error=value already held: ' + name + '\n')
                value.connection = connection
                value.info = info # update info
                value.watching = False
                if value.msg:
                    connection.send(value.get_msg()) # send value
                value.calculate_watch_period()
                continue
            value = pypilotValue(self, name, info, connection)
            if 'persistent' in info and info['persistent']:
                value.calculate_watch_period()
                if name in self.persistent_data:
                    v = self.persistent_data[name]
                    if isinstance(v, numbers.Number):
                        v = float(v) # convert any numeric to floating point
                    value.set(v, connection) # set persistent value

            self.values[name] = value
            self.msg = 'new'

            msg = False # inform watching clients of updated values
            for watch in self.awatches:
                for c in watch.connections:
                    if c != connection:
                        if not msg:
                            msg = 'values=' + pyjson.dumps(values) + '\n'
                        c.send(msg)

    def HandleRequest(self, msg, connection):
        name, data = msg.split('=', 1)        
        if not name in self.values:
            connection.send('invalid unknown value' + name + '\n')
        self.values[name].set(msg, connection)

    def load_file(self, f):
        line = f.readline()
        while line:
            name, data = line.split('=', 1)
            self.persistent_data[name] = line
            if name in self.values:
                value = self.values[name]
                if value.connection:
                    connection.send(line)
                else:
                    value.msg = line
                    
            self.values[name] = pypilotValue(self.values, name, msg=line)
            
            line = f.readline()
        f.close()
        
    def load(self):
        self.persistent_data = {}
        try:
            self.load_file(open(default_persistent_path))
        except Exception as e:
            print('failed to load', default_persistent_path, e)
            # log failing to load persistent data
            persist_fail = os.getenv('HOME') + '/.pypilot/persist_fail'
            file = open(persist_fail, 'a')
            file.write(str(time.time()) + ' ' + str(e) + '\n')
            file.close()

            try:
                self.load_file(open(default_persistent_path + '.bak'))
                return
            except Exception as e:
                print('backup data failed as well', e)
            return

        # backup persistent_data if it loaded with success
        file = open(default_persistent_path + '.bak', 'w')
        file.write(pyjson.dumps(self.persistent_data)+'\n')
        file.close()

    def store(self):
        self.persistent_timeout = time.monotonic() + server_persistent_period
        need_store = False
        for name in self.values:
            value = self.values[name]
            if not 'persistent' in value.info or not value.info['persistent']:
                continue
            if not name in self.persistent_data or value.msg != self.persistent_data[name]:
                self.persistent_data[name] = value.msg
                need_store = True

        if not need_store:
            return                
        try:
            file = open(default_persistent_path, 'w')
            for name in self.persistent_data:
                file.write(self.persistent_data[name])
            file.close()
        except Exception as e:
            print('failed to write', self.persistent_path, e)

class pypilotServer(object):
    def __init__(self):
        self.pipes = []
        self.multiprocessing = use_multiprocessing
        self.initialized = False
        self.process=False

    def pipe(self):
        if self.initialized:
            print('direct pipe clients must be created before the server is run')
            exit(0)

        pipe0, pipe1 = LineBufferedNonBlockingPipe('pypilotServer pipe' + str(len(self.pipes)), self.multiprocessing)
        self.pipes.append(pipe1)
        return pipe0
        
    def run(self):
        print('pypilotServer on', os.getpid())
        # if server is in a separate process
        self.init()
        while True:
            dt = self.values.sleep_time()
            #t0 = time.monotonic()
            self.poll(dt)
            #print('times', time.monotonic() - t0, dt)

    def init_process(self):
        if self.multiprocessing:
            import multiprocessing
            self.process = multiprocessing.Process(target=self.run, daemon=True)
            self.process.start()
        else:
            self.init()

    def init(self):
        self.process = 'main process'
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setblocking(0)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.port = DEFAULT_PORT
        self.sockets = []
        self.fd_to_pipe = {}

        self.values = ServerValues()

        while True:
            try:
                self.server_socket.bind(('0.0.0.0', self.port))
                break
            except:
                print('pypilot_server: bind failed; already running a server?')
                time.sleep(3)                

        # listen for tcp sockets
        self.server_socket.listen(5)
        fd = self.server_socket.fileno()
        self.fd_to_connection = {fd: self.server_socket}
        self.poller = select.poll()
        self.poller.register(fd, select.POLLIN)

        # setup direct pipe clients
        print('server setup has', len(self.pipes), 'pipes')
        if self.multiprocessing:
            for pipe in self.pipes:
                fd = pipe.fileno()
                self.poller.register(fd, select.POLLIN)
                self.fd_to_connection[fd] = pipe
                self.fd_to_pipe[fd] = pipe
                pipe.cwatches = {'values': True} # server always watches client values

        self.initialized = True

    def __del__(self):
        if not self.initialized:
            return
        self.values.store()
        self.server_socket.close()
        for socket in self.sockets:
            socket.close()
        for pipe in self.pipes:
            pipe.close()

    def HandleRequest(self, connection, request):
        self.values.HandleRequest(request, connection)

    def RemoveSocket(self, socket):
        self.sockets.remove(socket)

        found = False
        for fd in self.fd_to_connection:
            if socket == self.fd_to_connection[fd]:
                del self.fd_to_connection[fd]
                self.poller.unregister(fd)
                found = True
                break

        if not found:
            print('socket not found in fd_to_connection')

        socket.close()
        self.values.remove(socket)

    def poll(self, timeout=0):
        # server is in subprocess
        if self.process != 'main process':
            if not self.process:
                self.init_process()
            return

        t0 = time.monotonic()
        if t0 >= self.values.persistent_timeout:
            self.values.store()
            if time.monotonic() - t0 > .1:
                print('persistent store took too long!', time.monotonic() - t0)
                return

        if timeout:
            timeout *= 1000 # milliseconds

        events = self.poller.poll(timeout)
        while events:
            event = events.pop()
            fd, flag = event
                                    
            connection = self.fd_to_connection[fd]
            if connection == self.server_socket:
                connection, address = connection.accept()
                if len(self.sockets) == max_connections:
                    print('pypilot server: max connections reached!!!', len(self.sockets))
                    self.RemoveSocket(self.sockets[0]) # dump first socket??
                socket = LineBufferedNonBlockingSocket(connection)
                
                self.sockets.append(socket)
                fd = socket.fileno()
                socket.cwatches = {'values': True} # server always watches client values 

                self.fd_to_connection[fd] = socket
                self.poller.register(fd, select.POLLIN)
            elif flag & (select.POLLHUP | select.POLLERR | select.POLLNVAL):
                if not connection in self.sockets:
                    print('internal pipe closed, server exiting')
                    exit(0)
                self.RemoveSocket(connection)
            elif flag & select.POLLIN:
                if fd in self.fd_to_pipe:
                    while True:
                        line = connection.readline()
                        if not line:
                            break
                        self.HandleRequest(connection, line)
                    continue
                if not connection.recv():
                    self.RemoveSocket(connection)
                    continue
                while True:
                    line = connection.readline()
                    if not line:
                        break
                    try:
                        self.HandleRequest(connection, line)
                    except Exception as e:
                        print('invalid request from connection', line)
                        print(e)
                        connection.send('invalid request: ' + line + '\n')

        if not self.multiprocessing:
            # these pipes are not pollable
            for pipe in self.pipes:
                while True:
                    line = pipe.readline()
                    if not line:
                        break
                    self.HandleRequest(pipe, line)
                        
        # send periodic watches
        self.values.send_watches()

        # flush all sockets
        for socket in self.sockets:
            socket.flush()

        # send watches
        for connection in self.sockets + self.pipes:
            if connection.cwatches:
                connection.send('watch=' + pyjson.dumps(connection.cwatches) + '\n')
                connection.cwatches = {}

if __name__ == '__main__':
    server = pypilotServer()
    from client import pypilotClient
    from values import *
    client1 = pypilotClient(server) # direct pipe to server
    clock = client1.register(Value('clock', 0))
    test1 = client1.register(Property('test', 1234))
    client1.watch('test2', 10)

    client2 = pypilotClient('localhost') # tcp socket connection
    test2 = client2.register(Property('test2', [1,2,3,4]))
    client2.watch('clock', 1)

    client3 = pypilotClient('localhost')
    client3.watch('clock', 3)    

    def print_msgs(name, msgs):
        for msg in msgs:
            print(name, msg, msgs[msg])
    
    print('pypilot demo server')
    t00 = t0 = time.monotonic()
    while True:
        server.poll()
        print_msgs('client1', client1.receive())
        print_msgs('client2', client2.receive())
        print_msgs('client3', client3.receive())

        time.sleep(.004)
        dt = time.monotonic() - t0
        if dt > .01:
            clock.set(time.monotonic()-t00)
            t0 += .01
            test2.set(123)
