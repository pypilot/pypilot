#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import json
import socket
import select
import sys
import os
import time

from values import *
import server

try:
    import serial
except:
    pass

class ConnectionLost(Exception):
    pass

class SignalKClient(object):
    def __init__(self, f_on_connected, host=False, port=False, autoreconnect=False):
        self.autoreconnect = autoreconnect
        config = {}
        configfilename = '/home/sean/.pypilot/pypilot.conf' 
        try:
            file = open(configfilename)
            config = json.loads(file.readline())
        except IOError:
            print 'failed to load config file:', configfilename

        if not host:
            if 'host' in config:
                host = config['host']
            else:
                host = 'localhost'

        if '/dev' in host: # serial port
            device, baud = host, port
            if not baud or baud == 21311:
                baud = 9600
            connection = serial.Serial(device, baud)
            cmd = 'stty -F ' + device + ' icanon iexten'
            print 'running', cmd
            os.system(cmd)
        else:
            if not port:
                if ':' in host:
                    i = host.index(':')
                    host = host[:i]
                    port = host[i+1:]
                else:
                    port = server.DEFAULT_PORT
            try:
                connection = socket.create_connection((host, port), 1)
            except:
                print 'connect failed to %s:%d' % (host, port)
                raise

            self.host_port = host, port
        self.f_on_connected = f_on_connected
        self.onconnected(connection)

    def onconnected(self, connection):
        self.socket = server.LineBufferedNonBlockingSocket(connection)
        self.values = []
        self.msg_queue = []
        self.f_on_connected(self)

    def poll(self, timeout = 0):
        t0 = time.time()
        poller = select.poll()

        if self.socket:
            fd = self.socket.socket.fileno()
        else:
            fd = self.serial.fileno()
        poller.register(fd, select.POLLIN | select.POLLOUT | \
                        select.POLLPRI | select.POLLHUP | select.POLLERR)
        events = poller.poll(1000.0 * timeout)        
        while events != []:
            event = events.pop()
            fd, flag = event
            if flag & (select.POLLHUP | select.POLLERR):
                raise ConnectionLost
            if flag & select.POLLIN:
                if self.socket:
                    if not self.socket.recv():
                        raise ConnectionLost
            if flag & select.POLLOUT:
                if self.socket:
                    self.socket.flush()

    def send(self, request):
        self.socket.send(json.dumps(request)+'\n')

    def receive_line(self, timeout = 0):
        try:
            t0 = t1 = time.time()
            if self.socket:
                f = self.socket
            else:
                f = self.serial
            while timeout - (t1 - t0) >= 0:
                self.poll(timeout - (t1 - t0) > 0)
                line = f.readline()
                if line:
#                    msg = json.loads(line.rstrip())
                    try:
                        msg = json.loads(line.rstrip())
                    except:
                        # ignore garbage
                        print 'invalid message from server:', line
                        return False

                    return msg
            
                dt = time.time() - t1
                # maybe sleep for up to 10 ms
                if dt < .01:
                    time.sleep(.01 - dt)

                t1 = time.time()
        except ConnectionLost:
            self.socket.socket.close()
            if not self.autoreconnect:
                raise ConnectionLost
                
            connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            while True:
                print 'Disconnected.  Reconnecting in 3...'
                time.sleep(3)
                try:
                    connection.connect(self.host_port)
                    print 'Connected.'
                    break
                except:
                    continue

            self.onconnected(connection)

        return False

    def receive(self, timeout = 0):
        ret = {}
        msg = self.receive_single(timeout)
        while msg:
            name, value = msg
            ret[name] = value
            msg = self.receive_single()
        return ret

    def receive_single(self, timeout = 0):
        if len(self.msg_queue) > 0:
            msg = self.msg_queue[0]
            self.msg_queue = self.msg_queue[1:]
            return msg
        
        line = self.receive_line(timeout)
        if line:
            self.msg_queue += self.flatten_line(line)
            return self.receive_single()
        return False

    def flatten_line(self, line, name_prefix=''):
        msgs = []
        for name in line:
            msg = line[name]
            if type(msg) == type({}):
                if 'value' in msg or 'type' in msg:
                    msgs.append((name_prefix + name, msg))
                else:
                    msgs += self.flatten_line(msg, name_prefix + name + '/')
        return msgs

    def list_values(self):
        request = {'method' : 'list'}
        self.send(request)
        return self.receive(10)

    def get(self, name):
        request = {'method' : 'get', 'name' : name}
        self.send(request)

    def set(self, name, value):
        request = {'method' : 'set', 'name' : name, 'value' : value}
        self.send(request)

    def watch(self, name, value=True):
        request = {'method' : 'watch', 'name' : name, 'value' : value}
        self.send(request)

    def print_values(self):
        if len(self.values) == 0:
            self.values = self.list_values()

        if not self.values:
            return

        for name in sorted(self.values):
            self.get(name)
            result = self.receive(2)
            if result:
                print name, '=', result[name]['value']
            else:
                print 'no result', name

def SignalKClientFromArgs(argv, *cargs):
    host = False
    port = False
    if len(argv) > 1:
        if ':' in argv[1]:
            i = argv[1].index(':')
            host = argv[1][:i]
            port = int(argv[1][i+1:])
        else:
            host = argv[1]

    def on_con(client):
        for arg in argv[2:]:
            client.watch(arg)
        if len(cargs) == 1:
            cargs[0]()
            
    return SignalKClient(on_con, host, port)

# this simple test client for an autopilot server
# connects, enumerates the values, and then requests
# each value, printing them
def main():
    client = SignalKClientFromArgs(sys.argv)
    if len(sys.argv) <= 2:
        client.print_values()
        exit()

    while True:
        result = client.receive(1000)
        if result:
            print json.dumps(result)
