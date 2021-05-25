#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import select, socket, time
import sys, os, heapq

import gettext
locale_d= os.path.abspath(os.path.dirname(__file__)) + '/../locale'
gettext.bindtextdomain('pypilot', locale_d)
_ = gettext.gettext

import numbers
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import pyjson
from bufferedsocket import LineBufferedNonBlockingSocket
from nonblockingpipe import NonBlockingPipe

DEFAULT_PORT = 23322
max_connections = 30
default_persistent_path = os.getenv('HOME') + '/.pypilot/pypilot.conf'
server_persistent_period = 120 # store data every 120 seconds
use_multiprocessing = True # run server in a separate process

class Watch(object):
    def __init__(self, value, connection, period):
        self.value = value
        self.connections = [connection]
        self.period = period
        self.time = 0

class pypilotValue(object):
    def __init__(self, values, name, info={}, connection=False, msg=False):
        self.server_values = values
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
                        connection.write(msg, True)

                for watch in self.pwatches:
                    if t0 >= watch.time:
                        watch.time = t0
                    if watch.connections: # only insert if there are connections
                        self.server_values.insert_watch(watch)
                self.pwatches = []

        elif self.connection: # inform owner of change if we are not owner
            if 'writable' in self.info and self.info['writable']:
                name, data = msg.rstrip().split('=', 1)
                pyjson.loads(data) # validate data
                self.connection.write(msg)
                self.msg = False
            else: # inform key can not be set arbitrarily
                connection.write('error='+self.name+' is not writable\n')

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
                print(_('ERROR no connections in watch')) # should never hit
            if watching is False or watch.period < watching:
                watching = watch.period
                
        if watching is not self.watching:
            self.watching = watching
            if not watching and watching is not False:
                watching = True
            if self.connection:
                self.connection.cwatches[self.name] = watching
                if watching is False:
                    self.msg = None # server no longer tracking value

    def unwatch(self, connection, recalc):
        for watch in self.awatches:
            if connection in watch.connections:
                watch.connections.remove(connection)
                if not watch.connections:
                    self.awatches.remove(watch)
                    if recalc and watch.period is self.watching:
                        self.calculate_watch_period()
                return True
        return False
            
    def watch(self, connection, period):
        if connection == self.connection:
            connection.write('error=can not add watch for own value: ' + self.name + '\n')
            return
        
        if period is False: # period is False: remove watch
            if not self.unwatch(connection, True):
                # inform client there was no watch
                connection.write('error=cannot remove unknown watch for ' + self.name + '\n')
            return
        
        if period is True:
            period = 0 # True is same as a period of 0, for continuous watch

        # unwatch by removing
        watching = self.unwatch(connection, False)

        if not watching and self.msg and period >= self.watching:
            connection.write(self.get_msg()) # initial retrieval

        for watch in self.awatches:
            if watch.period == period: # already watching at this rate, add connection
                watch.connections.append(connection)
                if period > self.watching: # only need to update if period is relaxed
                    self.calculate_watch_period()
                break
        else:
            # need a new watch for this unique period
            watch = Watch(self, connection, period)
            if period == 0: # make sure period 0 is always at start of list
                self.awatches.insert(0, watch)
            else:
                self.awatches.append(watch)
            self.calculate_watch_period()
            if period:
                self.pwatches.append(watch)


class ServerWatch(pypilotValue):
    def __init__(self, values):
        super(ServerWatch, self).__init__(values, 'watch')

    def set(self, msg, connection):
        name, data = msg.rstrip().split('=', 1)        
        watches = pyjson.loads(data)
        values = self.server_values.values
        for name in watches:
            if not name in values:
                # watching value not yet registered, add it so we can watch it
                values[name] = pypilotValue(self.server_values, name)
            values[name].watch(connection, watches[name])

class ServerUDP(pypilotValue):
    def __init__(self, values, server):
        super(ServerUDP, self).__init__(values, 'udp_port')
        self.server = server

    def set(self, msg, connection):
        try:
            name, data = msg.rstrip().split('=', 1)
            self.msg = pyjson.loads(data)
            if not (self.msg is False) and self.msg < 1024 or self.msg > 65535:
                raise Exception('port out of range')
        except Exception as e:
            connection.write('error=invalid udp_port:' + msg + e + '\n')
            return

        # remove any identical udp connection
        for socket in self.server.sockets:
            if socket.udp_port and (socket.udp_port == self.msg or not self.msg) and socket.address[0] == connection.address[0]:
                #print('remove old udp')
                socket.udp_port = False
                socket.udp_out_buffer = ''

        connection.udp_port = self.msg # output streams on this port
        for c in self.server.sockets:
            if c == connection:
                continue
            if c.address[0] == connection.address[0] and c.udp_port == connection.udp_port:
                print(_('remove duplicate udp connection'))
                c.udp_socket.close()
                c.udp_port = False


class ServerValues(pypilotValue):
    def __init__(self, server):
        super(ServerValues, self).__init__(self, 'values')
        self.values = {'values': self, 'watch': ServerWatch(self), 'udp_port': ServerUDP(self, server)}
        self.internal = list(self.values)
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
                if name in self.internal:
                    continue
                info = self.values[name].info
                if not info: # placeholders that are watched
                    continue
                if notsingle:
                    msg += ','
                msg += '"' + name + '":' + pyjson.dumps(info)
                notsingle = True
            self.msg = msg + '}\n'
            #print('values len', len(self.msg))
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
            msg = watch.value.get_msg()
            if msg:
                for connection in watch.connections:
                    connection.write(msg, True)

            watch.time += watch.period
            if watch.time < t0:
                watch.time = t0
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
        if isinstance(connection, LineBufferedNonBlockingSocket):
            connection.write('error=remote sockets not allowed to register\n')
            return
        n, data = msg.rstrip().split('=', 1)        
        values = pyjson.loads(data)
        for name in values:
            info = values[name]
            if name in self.values:
                value = self.values[name]
                if value.connection:
                    connection.write('error=value already held: ' + name + '\n')
                    continue
                value.connection = connection
                value.info = info # update info
                value.watching = False
                if value.msg:
                    connection.write(value.get_msg()) # send value
                value.calculate_watch_period()
                self.msg = 'new'
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
                    c.write(msg)

    def HandleRequest(self, msg, connection):
        name, data = msg.split('=', 1)
        if not name in self.values:
            connection.write('error=invalid unknown value: ' + name + '\n')
            return
        self.values[name].set(msg, connection)

    def load_file(self, f):
        line = f.readline()
        while line:
            name, data = line.split('=', 1)
            self.persistent_data[name] = line
            if name in self.values:
                value = self.values[name]
                if value.connection:
                    print('does this ever hit?? ,.wqiop pasm2;')
                    connection.write(line)
                    
            self.values[name] = pypilotValue(self, name, msg=line)
            
            line = f.readline()
        f.close()
        
    def load(self):
        self.persistent_data = {}
        try:
            self.load_file(open(default_persistent_path))
        except Exception as e:
            print(_('failed to load'), default_persistent_path, e)
            # log failing to load persistent data
            persist_fail = os.getenv('HOME') + '/.pypilot/persist_fail'
            file = open(persist_fail, 'a')
            file.write(str(time.time()) + ' ' + str(e) + '\n')
            file.close()

            try:
                self.load_file(open(default_persistent_path + '.bak'))
                return
            except Exception as e:
                print(_('backup data failed as well'), e)
            return

        # backup persistent_data if it loaded with success
        file = open(default_persistent_path + '.bak', 'w')
        for name in self.persistent_data:
            file.write(self.persistent_data[name])
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
            print(_('failed to write'), default_persistent_path, e)

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

        pipe0, pipe1 = NonBlockingPipe('pypilotServer pipe' + str(len(self.pipes)), self.multiprocessing)
        self.pipes.append(pipe1)
        return pipe0
        
    def run(self):
        print('pypilotServer process', os.getpid())
        # if server is in a separate process
        self.init()
        while True:
            dt = self.values.sleep_time()
            t0 = time.monotonic()
            self.poll(dt)
            pt = time.monotonic() - t0
            #print('times', pt, dt)
            st = .04 - pt
            if st > 0:
                time.sleep(st)

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

        self.values = ServerValues(self)

        while True:
            try:
                self.server_socket.bind(('0.0.0.0', self.port))
                break
            except:
                print(_('pypilot_server: bind failed; already running a server?'))
                time.sleep(3)                

        # listen for tcp sockets
        self.server_socket.listen(5)
        fd = self.server_socket.fileno()
        self.fd_to_connection = {fd: self.server_socket}
        self.poller = select.poll()
        self.poller.register(fd, select.POLLIN)

        # setup direct pipe clients
        print('server setup has', len(self.pipes), 'pipes')
        for pipe in self.pipes:
            if self.multiprocessing:
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

    def RemoveSocket(self, socket):
        print('server, remove socket', socket.address)
        self.sockets.remove(socket)

        found = False
        for fd in self.fd_to_connection:
            if socket == self.fd_to_connection[fd]:
                del self.fd_to_connection[fd]
                self.poller.unregister(fd)
                found = True
                break

        if not found:
            print('server error: socket not found in fd_to_connection')

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
            dt = time.monotonic() - t0
            if dt > .1:
                print(_('persistent store took too long!'), time.monotonic() - t0)
                return

        if timeout:
            timeout *= 1000 # milliseconds

        timeout = .1
        events = self.poller.poll(timeout)
        while events:
            event = events.pop()
            fd, flag = event
                                    
            connection = self.fd_to_connection[fd]
            if connection == self.server_socket:
                connection, address = connection.accept()
                if len(self.sockets) == max_connections:
                    print('pypilot server: ' + _('max connections reached') + '!!!', len(self.sockets))
                    self.RemoveSocket(self.sockets[0]) # dump first socket??
                socket = LineBufferedNonBlockingSocket(connection, address)
                print(_('server add socket'), socket.address)

                self.sockets.append(socket)
                fd = socket.fileno()
                socket.cwatches = {} # {'values': True} # server always watches client values

                self.fd_to_connection[fd] = socket
                self.poller.register(fd, select.POLLIN)
            elif flag & (select.POLLHUP | select.POLLERR | select.POLLNVAL):
                if not connection in self.sockets:
                    print(_('internal pipe closed, server exiting'))
                    exit(0)
                self.RemoveSocket(connection)
            elif flag & select.POLLIN:
                if fd in self.fd_to_pipe:
                    if not connection.recvdata():
                        continue
                    line = connection.readline() # shortcut since poll indicates data is ready
                    while line:
                        self.values.HandleRequest(line, connection)                        
                        line = connection.readline()
                    continue
                if not connection.recvdata():
                    self.RemoveSocket(connection)
                    continue
                while True:
                    line = connection.readline()
                    if not line:
                        break
                    try:
                        self.values.HandleRequest(line, connection)                        
                    except Exception as e:
                        connection.write('error=invalid request: ' + line)
                        try:
                            print('invalid request from connection', e, line)
                        except Exception as e2:
                            print('invalid request has malformed string', e, e2)

        if not self.multiprocessing:
            # these pipes are not pollable as they are implemented as a simple buffer
            for pipe in self.pipes:
                while True:
                    line = pipe.readline()
                    if not line:
                        break
                    self.values.HandleRequest(line, pipe)
                        
        # send periodic watches
        self.values.send_watches()

        # send watches
        for connection in self.sockets + self.pipes:
            if connection.cwatches:
                connection.write('watch=' + pyjson.dumps(connection.cwatches) + '\n')
                connection.cwatches = {}

        # flush all sockets
        for socket in self.sockets:
            socket.flush()
        while True:
            for socket in self.sockets:
                if not socket.socket:
                    print(_('server socket closed from flush!!'))
                    self.RemoveSocket(socket)
                    break
            else:
                break
                
        for pipe in self.pipes:
            pipe.flush()

if __name__ == '__main__':
    server = pypilotServer()
    from client import pypilotClient
    from values import *
    client1 = pypilotClient(server) # direct pipe to server
    clock = client1.register(Value('clock', 0))
    test1 = client1.register(Property('test', 1234))
    print('client values1', client1.values)
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

        time.sleep(.04)
        dt = time.monotonic() - t0
        if dt > .01:
            clock.set(time.monotonic()-t00)
            t0 += .01
            #test2.set(123)
