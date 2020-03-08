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
import pyjson
import fcntl, os
from bufferedsocket import LineBufferedNonBlockingSocket

DEFAULT_PORT = 23322
max_connections = 30
default_persistent_path = os.getenv('HOME') + '/.pypilot/pypilot.conf'
server_persistent_period = 60 # store data every 60 seconds

class pypilotSocket(LineBufferedNonBlockingSocket):
    def __init__(self, connection):
        super(pypilotSocket, self).__init__(connection)
        self.cwatches = {'values': True} # always watch client values

    def flush(self):
        # write combined watches to client
        if self.cwatches:
            self.send('watch=' + pyjson.dumps(self.cwatches) + '\n')
            self.cwatches = {}
        super(pypilotSocket, self).flush()

class Watch(object):
    def __init__(self, value, connection, period):
        self.value = value
        self.connections = [connection]
        self.period = period
        self.time = 0 # send first update immediate

class pypilotValue(object):
    def __init__(self, values, name, info, connection):
        self.values = values
        self.name = name
        self.info = info
        self.lasttime = time.time()
        self.connection = connection
        self.watching = False # is False, or the period, so 0 rather than True

        self.awatches = [] # all watches
        self.pwatches = [] # periodic watches limited in period
        self.msg = False

    def get_msg(self):
        return self.msg
        
    def set(self, msg, connection):
        t0 = time.time()
        if self.connection == connection:
            # received new value from owner, inform watchers
            self.msg = msg

            for watch in self.awatches:
                if watch.period == 0:
                    for connection in watch.connections:
                        connection.send(msg)
                    break
            for watch in self.pwatches:
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

        if watching != self.watching:
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
                values[name] = pypilotValue(self.values, name, {}, False)
            values[name].watch(connection, watches[name])

class ServerValues(pypilotValue):
    def __init__(self, persistent_path):
        super(ServerValues, self).__init__(self, 'values', {}, None)
        self.persistent_path = persistent_path
        self.values = {'values': self, 'watch': ServerWatch(self)}
        self.msg = 'new'
        self.load()
        self.pqwatches = [] # priority queue of watches

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
            
    def send_watches(self):
        t0 = time.time()
        i = 0
        while i < len(self.pqwatches):
            watch = self.pqwatches[i]
            if t0 >= watch.time-watch.period and t0 < watch.time:
                break # no more are ready
            i += 1
            if not watch.connections:
                continue # forget this watch
            for connection in watch.connections:
                connection.send(watch.value.get_msg())
            watch.time = t0+watch.period
            watch.value.pwatches.append(watch) # put back on value periodic watch list
        # remove watches handled
        self.pqwatches = self.pqwatches[i:]

    def remove(self, connection):
        for name in self.values:
            value = self.values[name]
            if value.connection == connection:
                value.connection = False
                continue
            value.remove_watches(connection)
            
    def insert_watch(self, watch):
        if watch in self.pqwatches:
            print('error, watch should not be in pqwatches')
            return

        i = 0
        while i < len(self.pqwatches):
            if self.pqwatches[i].time > watch.time:
                break
            i += 1
        self.pqwatches.insert(i, watch)
            
    def set(self, msg, connection):
        name, data = msg.rstrip().split('=', 1)        
        values = pyjson.loads(data)
        for name in values:
            info = values[name]
            if name in self.values:
                value = self.values[name]
                if value.connection:
                    connection.send('error=value already held: ' + name + '\n')
                else:
                    value.connection = connection
                    value.info = info # update info
                    value.watching = (False,)
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
            name, data = line.rstrip().split('=', 1)
            self.persistent_data[name] = line
            line = f.readline()
        f.close()
        
    def load(self):
        self.persistent_data = {}
        try:
            self.load_file(open(self.persistent_path))
        except Exception as e:
            print('failed to load', self.persistent_path, e)
            # log failing to load persistent data
            persist_fail = os.getenv('HOME') + '/.pypilot/persist_fail'
            file = open(persist_fail, 'a')
            file.write(str(time.time()) + ' ' + str(e) + '\n')
            file.close()

            try:
                self.load_file(open(self.persistent_path + '.bak'))
                return
            except Exception as e:
                print('backup data failed as well', e)
            return

        # backup persistent_data if it loaded with success
        file = open(self.persistent_path + '.bak', 'w')
        file.write(pyjson.dumps(self.persistent_data)+'\n')
        file.close()

    def store(self):
        self.persistent_timeout = time.time() + server_persistent_period
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
            file = open(self.persistent_path, 'w')
            for name in self.persistent_data:
                file.write(self.persistent_data[name])
            file.close()
        except Exception as e:
            print('failed to write', self.persistent_path, e)

class pypilotServer(object):
    def __init__(self, port=DEFAULT_PORT, persistent_path=default_persistent_path):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setblocking(0)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.port = port        
        self.sockets = []
        self.persistent_timeout = time.time() + server_persistent_period
        self.values = ServerValues(persistent_path)


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

    def __del__(self):
        self.values.store()
        self.server_socket.close()
        for socket in self.sockets:
            socket.socket.close()

    def HandleRequest(self, socket, request):
        self.values.HandleRequest(request, socket)

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

        self.values.remove(socket)
        
    def poll(self):        
        t1 = time.time()
        #print('store', t1 - self.persistent_timeout)
        if t1 < self.persistent_timeout - server_persistent_period:
            print('clock skew detected, resetting persistent timeout')
            self.persistent_timeout = time.time() + server_persistent_period

        if t1 >= self.persistent_timeout:
            self.values.store()
            if time.time() - t1 > .1:
                print('persistent store took too long!', time.time() - t1)
                return

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

                socket = pypilotSocket(connection)
                self.sockets.append(socket)
                fd = socket.socket.fileno()
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
                    if True:
                        self.HandleRequest(socket, line)
                    else:
                     try:
                        self.HandleRequest(socket, line)
                     except Exception as e:
                        print('invalid request from socket', line)
                        print(e)
                        socket.send('invalid request: ' + line + '\n')

        # send periodic watches
        self.values.send_watches()

        # flush all sockets
        for socket in self.sockets:
            socket.flush()

if __name__ == '__main__':
    server = pypilotServer()

    from client import pypilotClient
    from values import *
    client1 = pypilotClient()
    clock = client1.register(Value('clock', 0))
    test1 = client1.register(Property('test', 1234))
    client1.watch('test2')

    client2 = pypilotClient()
    test2 = client2.register(Property('test2', [1,2,3,4]))
    client2.watch('clock')

    print('pypilot demo server')
    t0 = time.time()
    while True:
        server.poll()

        msgs = client1.receive()
        if msgs:
            print('client1:', msgs)

        msgs = client2.receive()
        if msgs:
            print('client2:', msgs)

        time.sleep(.1)
        dt = time.time() - t0
        if dt > 1:
            clock.set(clock.value+1)
            t0 += 1
            print('clock', clock.value)
            test2.set(123)

        
