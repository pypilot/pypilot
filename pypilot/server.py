#!/usr/bin/env python
#
#   Copyright (C) 2022 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import select, socket, time
import sys, os, heapq

import numbers
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import gettext_loader
import pyjson
from bufferedsocket import LineBufferedNonBlockingSocket
from nonblockingpipe import NonBlockingPipe

DEFAULT_PORT = 23322
from zeroconf_service import zeroconf
max_connections = 30
configfilepath = os.getenv('HOME') + '/.pypilot/'
configfilename = 'pypilot.conf'
server_persistent_period = 60 # store data every 60 seconds
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
                        if not connection:
                            print('connection FALSE', self.name)
                            continue
                        connection.write(msg, True)

                for watch in self.pwatches:
                    if t0 >= watch.time:
                        watch.time = t0
                    if watch.connections: # only insert if there are connections
                        self.server_values.insert_watch(watch)
                self.pwatches = []

        elif self.connection: # inform owner of change if we are not owner
            if not connection or 'writable' in self.info and self.info['writable']:
                name, data = msg.rstrip().split('=', 1)
                try:
                    pyjson.loads(data) # validate data
                    self.connection.write(msg)
                except Exception as e:
                    print('failed to load ', msg)
                self.msg = None
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
        watching = self.unwatch(connection, False) # or for server values (self.connection is False)
        if not watching and self.msg and (period >= self.watching or self.connection is False):
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

# special server value a client can set to specify udp data port to use
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

class ServerProfiles(pypilotValue):
    def __init__(self, values):
        super(ServerProfiles, self).__init__(values, 'profiles', info = {'type': 'Value', 'persistent': True, 'writable': True})
        self.msg = 'new'
        self.profiles = ['default']

    def get_msg(self):
        if not self.msg or self.msg == 'new':
            self.msg = 'profiles=' + pyjson.dumps(self.profiles) + '\n'
        return self.msg

    def add(self, profile):
        if profile in self.profiles:
            return
        self.msg = 'new'
        self.profiles.append(profile)
        super(ServerProfiles, self).set(self.get_msg(), False) # inform any clients watching this value        

    def set(self, msg, connection):
        self.msg = 'new'
        n, profiles = msg.rstrip().split('=', 1)
        try:
            profiles = pyjson.loads(profiles)
            sprofiles = []
            for profile in profiles:
                sprofiles.append(str(profile))
            if not sprofiles:
                profiles.append('default')  # ensure the default profile exists if there are no others
            self.profiles = sprofiles
            profile = self.server_values.values['profile']

            # if current profile is removed switch to first profile
            if not profile.profile in self.profiles:
                profile.set('profile="' + profiles[0] + '"\n', False)
                
        except Exception as e:
            print('pypilot server failed to set new visible profiles', e, msg)
            return

        super(ServerProfiles, self).set(msg, False) # inform any clients watching this value

        
class ServerProfile(pypilotValue):
    def __init__(self, values):
        super(ServerProfile, self).__init__(values, 'profile', info = {'type': 'Value', 'persistent': True, 'writable': True})
        self.profile = 'default'
        self.msg = 'new'

    def get_msg(self):
        if not self.msg or self.msg == 'new':
            self.msg = 'profile=' + pyjson.dumps(str(self.profile)) + '\n'
        return self.msg

    def set(self, msg, connection):
        n, profile = msg.rstrip().split('=', 1)
        if profile == self.profile: # no change, do nothing
            return

        try:
            strprofile = str(pyjson.loads(profile))
            strprofile.replace('"', '')
        except Exception as e:
            print('server bad profile', e, msg)
            return

        if strprofile != profile:
            msg = n + '="' + strprofile + '"\n'

        self.server_values.values['profiles'].add(strprofile)

        persistent_values = self.server_values.persistent_values
        persistent_data = self.server_values.persistent_data

        if not self.profile in persistent_data:
            persistent_data[self.profile] = {}
        prev = persistent_data[self.profile]
        if not strprofile in persistent_data:
            persistent_data[strprofile] = {}
        data = persistent_data[strprofile]
        for name, value in persistent_values.items():
            if not value.info.get('profiled'):
                continue
            if not name in prev or prev[name] != value.msg:
                prev[name] = value.msg
                self.server_values.need_store = True # ensure we store c

            if not name in data:
                vmsg = value.get_msg()  # add this value to profile copying it from previous profile
                if vmsg:
                    data[name] = vmsg
                else:
                    print("PROFILED DATA WITHOUT MSG?  is not tracked?", name)
            elif data[name] != value.msg:
                value.set(data[name], False)  # only inform clients of the updated value from profile change if it really did change
        self.msg = 'new' # invalidate
        self.profile = strprofile
        super(ServerProfile, self).set(msg, False) # inform any clients watching this value
                
class ServerValues(pypilotValue):
    def __init__(self, server):
        super(ServerValues, self).__init__(self, 'values')
        profile = ServerProfile(self)
        profiles = ServerProfiles(self)
        
        self.persistent_values = {'profile': profile, 'profiles': profiles}
        self.values = {'values': self, 'watch': ServerWatch(self), 'udp_port': ServerUDP(self, server)}
        self.values.update(self.persistent_values)
        self.pipevalues = {}
        self.msg = 'new'
        self.persistent_timeout = time.monotonic() + server_persistent_period
        self.load()
        self.need_store = False
        self.pqwatches = [] # priority queue of watches
        self.last_send_watches = 0

    def get_msg(self):
        if not self.msg or self.msg == 'new':
            msg = 'values={'
            notsingle = False
            for name in self.values:
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
            else:
                value = pypilotValue(self, name, info, connection)
                self.values[name] = value

            if info.get('persistent'):
                # when a persistant value is missing from pypilot.conf
                value.calculate_watch_period()
                if not name in self.persistent_values:
                    self.persistent_values[name] = value

            if info.get('profiled'):
                if name in self.persistent_data[None]:
                    del self.persistent_data[None][name]

            self.msg = 'new'

        msg = False # inform watching clients of updated values
        for watch in self.awatches:
            for c in watch.connections:
                if c != connection:
                    if not msg:
                        msg = 'values=' + pyjson.dumps(values) + '\n'
                    c.write(msg)

    def HandleRequest(self, msg, connection):
        if msg == '\n':
            return # silently ignore empty line used to poll connection if no data
        name, data = msg.split('=', 1)
        if not name in self.values:
            connection.write('error=invalid unknown value: ' + name + '\n')
            return

        self.values[name].set(msg, connection)

    def load_file(self, filename):
        profile = None
        self.persistent_data = {None : {}, 'default' : {}}
        print("load file",filename)
        f = open(filename)
        linei=0
        while True:
            linei+=1
            line = f.readline()
            if not line:
                break
            try:
                name, data = line.rstrip().split('=', 1)
            except Exception as e:
                print('failed to split ' + filename + ' line ', linei)
                continue
            if name[0] == '[' and data[-1] == ']': # new section
                if name[1:] != 'profile':
                    print('loading pypilot.conf, unrecognized section', name)
                    continue

                if data[0] != '"' or data[-2] != '"':
                    print('loading pypilot.conf, unrecognized profile', data)
                    continue
                
                profile = data[1:-2].replace('"', '')
                if not profile in self.persistent_data:
                    self.persistent_data[profile] = {}
                continue

            self.persistent_data[profile][name] = line
            if name in self.values:
                # loading file while running
                value = self.values[name]
                if name != value.name:
                    print("ERROR with values!", name, value.name)
                if value.msg != line:
                    if profile is None or self.values['profile'].profile == profile:
                        self.values[name].set(line, False)
            else:   
                self.values[name] = pypilotValue(self, name, msg=line)
                self.persistent_values[name] = self.values[name]

        f.close()
        
    def load(self):
        try:
            import inotify.adapters
            self.inotify = inotify.adapters.Inotify(block_duration_s=0)
            self.inotify.add_watch(configfilepath + configfilename)
            self.inotify_time = time.monotonic()
        except Exception as e:
            self.inotify = None
            print(_('failed to monitor '), configfilepath, e)
        try:
            if not os.path.exists(configfilepath):
                print(_('creating config directory: ') + configfilepath)
                os.makedirs(configfilepath)
            if not os.path.isdir(configfilepath):
                raise Exception(configfilepath + 'should be a directory')

            self.load_file(configfilepath + configfilename)
        except Exception as e:
            print(_('failed to load'), configfilename, e)
            # log failing to load persistent data
            persist_fail = configfilepath + 'persist_fail'
            file = open(persist_fail, 'a')
            file.write(str(time.time()) + ' ' + str(e) + '\n')
            file.close()

            try:
                self.load_file(configfilepath + configfilename + '.bak')
                return
            except Exception as e:
                print(_('backup data failed as well'), e)
            return

        # backup persistent_data if it loaded with success
        self.store_file(configfilepath + configfilename + '.bak')

    def poll_config(self, t0):
        if not self.inotify or t0 - self.inotify_time < 5:
            return
        
        self.inotify_time = t0
        loaded = False
        for event in self.inotify.event_gen(timeout_s=0):
            try:
                if event[1][0] == 'IN_CLOSE_WRITE':
                    if not loaded:
                        print('detected configuration file updated: reloading', configfilename)
                        self.load_file(configfilepath + configfilename)
                        loaded = True
            except Exception as e:
                print('pypilot server failed to detect or load config change', e)
                #print('pypilot server will now overwrite config file')
                #self.store_file(configfilepath + configfilename)

    def store_file(self, filename):
        if self.inotify:
            try:
                self.inotify.remove_watch(configfilepath + configfilename)
            except Exception as e:
                print("failed to remove watch", e)
                
        print("store_file", filename, '%.3f'%time.monotonic())
        file = open(filename, 'w')
        for name, value in self.persistent_data[None].items():
            file.write(value)
        for profile, data in self.persistent_data.items():
            if profile is None:
                continue
            profile.replace('"', '')
            file.write('[profile="' + profile + '"]\n')
            for name, value in data.items():
                if value:
                    file.write(value)
        file.close()

        if self.inotify:
            self.inotify.add_watch(configfilepath + configfilename)

    def store(self):
        self.persistent_timeout = time.monotonic() + server_persistent_period
        for name in self.persistent_values:
            value = self.persistent_values[name]
            if not value.info.get('persistent'):
                continue
            if value.info.get('profiled'):
                profile = self.values['profile'].profile
                if not profile in self.persistent_data:
                    self.persistent_data[profile] = {}
            else:
                profile = None

            data = self.persistent_data[profile]
            msg = value.get_msg()
            if msg and (not name in data or msg != data[name]):
                #print("need store, changed", name, data[name].rstrip(), msg.rstrip())
                data[name] = msg
                self.need_store = True

        if self.need_store:
            try:
                self.store_file(configfilepath + configfilename)
                self.need_store = False
            except Exception as e:
                print(_('failed to write'), configfilename, e)

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
            #print('server times', pt, dt)
            st = .04 - pt
            if st > 0:
                #time.sleep(st)
                pass

    def init_process(self):
        if self.multiprocessing:
            import multiprocessing
            self.process = multiprocessing.Process(target=self.run, daemon=True)
            self.process.start()
        else:
            self.init()

    def init(self):
        self.process = 'server process'
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
        self.zeroconf = zeroconf()
        self.zeroconf.start()
            
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
        if self.process != 'server process':
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

        # if config file is edited externally
        self.values.poll_config(t0)
        if not timeout:
            timeout = 400
        else:
            timeout *= 1000 # milliseconds

        # sleep between 50 and 400 milliseconds
        timeout = max(min(int(timeout), 400), 50)

        #timeout = 10
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

if __name__  == '__main__':
    server = pypilotServer()
    from client import pypilotClient
    from values import *
    client1 = pypilotClient(server) # direct pipe to server
    clock = client1.register(Value('clock', 0))
    test1 = client1.register(Property('test', 1234, profiled=True))
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
