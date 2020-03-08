#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from __future__ import print_function

import socket, select, sys, os, time
import pyjson
from bufferedsocket import LineBufferedNonBlockingSocket
from values import Value

DEFAULT_PORT = 23322

try:
    IOError
except:
    class IOError(Exception):
        pass
try:
    ourPOLLNVAL = select.POLLNVAL
except:
    print('select.POLLNVAL not defined, using 32')
    ourPOLLNVAL = 32
    
class ConnectionLost(Exception):
    pass

class Watch(object):
    def __init__(self, value, period):
        self.value = value
        self.period = period
        self.time = time.time()

class ClientWatch(Value):
    def __init__(self, values, client):
        super(ClientWatch, self).__init__('watch', {})
        self.values = values
        self.client = client

    def set(self, values):
        for name in values:
            value = self.values[name]
            period = values[name]
            if period is True or period is False:
                value.watch = period
            else:
                value.watch = Watch(self, period)
                value.pwatch = True
            if value.watch:
                self.client.send(name + '=' + value.get_msg() + '\n')

class ClientValues(Value):
    def __init__(self, client):
        self.value = False
        super(ClientValues, self).__init__('values', False)
        self.client = client
        self.values = {'values': self}
        self.values['watch'] = ClientWatch(self.values, client)
        self.pqwatches = []

    def set(self, values):
        if type(self.value) == type(False):
            self.value = values
        else:
            for name in values:
                self.value[name] = values[name]

    def send_watches(self):
        t0 = time.time()
        i = 0
        while i < len(self.pqwatches):
            watch = self.pqwatches[i]
            if t0 >= watch.time-watch.period and t0 < watch.time:
                break # no more are ready
            i += 1
            self.client.send(watch.name + '=' + watch.value.get_msg() + '\n')
            watch.time = t0+watch.period
            watch.value.pwatch = True
        # remove watches handled
        self.pqwatches = self.pqwatches[i:]
            
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

    def register(self, value):
        if value.name in self.values:
            print('warning, registering existing value:', value.name)
        self.values[value.name] = value

    def get_msg(self):
        info = {}
        for name in self.values:
            if name != 'values' and name != 'watch':
                info[name] = self.values[name].info
        self.client.wvalues = {}
        return pyjson.dumps(info)

class pypilotClient(object):
    def __init__(self, host=False):
        self.values = ClientValues(self)
        self.wvalues = {}
        self.watches = {}
        self.wwatches = {}
        self.received = []

        config = {}
        try:
            configfilepath = os.getenv('HOME') + '/.pypilot/'
            if not os.path.exists(configfilepath):
                os.makedirs(configfilepath)
            if not os.path.isdir(configfilepath):
                raise configfilepath + 'should be a directory'
        except Exception as e:
            print('os not supported')
            configfilepath = '/.pypilot/'
        self.configfilename = configfilepath + 'pypilot_client.conf'

        try:
            file = open(self.configfilename)
            config = pyjson.loads(file.readline())
            file.close()
                
        except Exception as e:
            print('failed to read config file:', self.configfilename, e)
            config = {}

        if host:
            if ':' in host:
                i = host.index(':')
                config['host'] = host[:i]
                config['port'] = host[i+1:]
            else:
                config['host'] = host
        if not 'host' in config:
            config['host'] = '127.0.0.1'

        if not 'port' in config:
            config['port'] = DEFAULT_PORT
        self.config = config
            
        if type(host) != type(''):
            print('client pipe connected')
            self.connection = host
        else:
            self.connection = False # connect later

    def onconnected(self, connection):
        # write config if connection succeeds
        try:
            file = open(self.configfilename, 'w')
            file.write(pyjson.dumps(self.config) + '\n')
            file.close()
            self.write_config = False
        except IOError:
            print('failed to write config file:', self.configfilename)
        except Exception as e:
            print('Exception writing config file:', self.configfilename, e)
        
        self.connection = LineBufferedNonBlockingSocket(connection)
        self.poller = select.poll()
        self.poller.register(self.connection.socket, select.POLLIN)

        self.wwatches = self.watches

    def poll(self, timeout = 0):
        if not self.connection:
            if not self.connect():
                return
            
        # inform server of any watches we have
        if self.values.watch and self.wvalues:
            self.connection.send('values=' + pyjson.dumps(self.wvalues) + '\n')
            self.wvalues = {}
        
        if self.wwatches:
            self.connection.send('watch=' + pyjson.dumps(self.wwatches) + '\n')
            self.wwatches = {}

        # send any delayed watched values
        self.values.send_watches()

        # flush output
        self.connection.flush()

        try:
            events = self.poller.poll(int(1000 * timeout))
        except Exception as e:
            print('exception polling', e)
            self.disconnected()
            return

        if events != []:
            event = events.pop()
            fd, flag = event
            if flag & (select.POLLERR | ourPOLLNVAL):
                disconnected()
            if flag & select.POLLIN:
                if self.connection and not self.connection.recv():
                    self.disconnected()

        # read incoming data line by line
        while True:
            line = self.connection.readline()
            if not line:
                return
            try:
                name, data = line.rstrip().split('=', 1)
                if name == 'error':
                    print('server error', data)
                    continue
                else:
                    value = pyjson.loads(data)
                    if name in self.values.values:
                        self.values.values[name].set(value)
                        continue
            except Exception as e:
                print('invalid message from server:', line)
                print('reason', e)
                raise Exception
            self.received.append((name, value))

    # polls at least as long as timeout
    def disconnected(self):
        self.connection.socket.close()
        raise ConnectionLost

    def connect(self, verbose=True):
        try:
            host_port = self.config['host'], self.config['port']
            connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            connection.settimeout(3)
            connection.connect(host_port)
        except Exception as e:
            if verbose:
                print('connect failed to %s:%d' % host_port)
                print('reason', e)
            return False
        self.onconnected(connection)
        return True
    
    def receive_single(self):
        if self.received:
            ret = self.received[0]
            self.received = self.received[1:]
            return ret
        return False

    def receive(self):
        self.poll()
        ret = {}
        for msg in self.received:
            name, value = msg
            ret[name] = value
        self.received = []
        return ret

    def send(self, msg):
        if self.connection:
            self.connection.send(msg)
    
    def set(self, name, value):
        # quote strings
        if type(value) == type('') or type(value) == type(u''):
            value = '"' + value + '"'
        elif type(value) == type(True):
            value = 'true' if value else 'false'
        self.send(name + '=' + str(value) + '\n')

    def watch(self, name, value=True):
        self.watches[name] = value
        self.wwatches[name] = value

    def register(self, value):
        self.wvalues[value.name] = value
        self.values.register(value)
        value.client = self
        return value

    def list_values(self, timeout=10):
        t0 = time.time()
        self.watch('values')
        while True:
            self.poll(timeout)
            if type(self.values.value) != type(False):
                return self.values.value
        return False

    def info(self, name):
        return self.values.value[name]

def pypilotClientFromArgs(args, period=True):
    host = False
    if len(args) > 1:
        host = args[1]

    client = pypilotClient(host)
    if not client.connect(False):
        if host:
            client = pypilotClient()
            watches = args[1:]
            client.connect()
        if not client.connection:
            print('failed to connect')
            exit(1)

    # set any value specified with path=value
    for arg in args[2:]:
        if '=' in arg:
            self.send(arg + '\n')
            arg, value = line.rstrip().split('=', 1)
        watches.append(arg)

    # args without = are watched
    for name in watches:
        client.watch(name, period)
    return client
    
# this simple test client for an autopilot server
# connects, enumerates the values, and then requests
# each value, printing them
def main():
    import signal
    def quit(sign, frame):
        exit(0)
    signal.signal(signal.SIGINT, quit)

    if '-h' in sys.argv:
        print('usage', sys.argv[0], '[host] -i -c -h [NAME[=VALUE]]...')
        print('eg:', sys.argv[0], '-i imu.compass')
        print('   ', sys.argv[0], 'servo.max_slew_speed=10')
        print('-i', 'print info about each value type')
        print('-c', 'continuous watch')
        print('-h', 'show this message')
        exit(0)

    args = list(sys.argv)
    continuous = '-c' in args
    if continuous:
        args.remove('-c')

    info = '-i' in args
    if info:
        args.remove('-i')
        
    period = True if continuous else 100 # 100 second period to just get the value once
    client = pypilotClientFromArgs(args, period)
    if not client.watches: # retrieve all values
        watches = client.list_values()
        for name in watches:
            client.watch(name, period)

    if not continuous:
        values = {}
        t0 = time.time()
        while len(values) < len(client.watches):
            dt = time.time() - t0
            if dt > 10:
                print('timeout retrieving values')
                exit(1)
                    
            client.poll(.1)
            msgs = client.receive()
            values = {**values, **msgs}
            
        names = sorted(values)
        for name in names:
            if info:
                print(name, client.info(name), '=', values[name])
            else:
                maxlen = 76
                result = name + ' = ' + str(values[name])
                if len(result) > maxlen:
                    result = result[:maxlen] + ' ...'
                print(result)
    else:
        while True:
            client.poll(1)
            msg = client.receive_single()
            if msg:
                name, data = msg
                if info:
                    print(name, client.info(name), '=', data)
                else:
                    print(name, '=', data)

if __name__ == '__main__':
    main()
