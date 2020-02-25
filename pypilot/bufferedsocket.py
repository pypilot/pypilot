#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from __future__ import print_function
import time, select

try:
  from pypilot.linebuffer import linebuffer
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
        self.sendfail_msg = 1
        self.sendfail_cnt = 0

    def recv(self):
        return self.b.recv()
        
    def readline(self):
        return self.b.line()

    def send(self, data):
        self.out_buffer += data
        if len(self.out_buffer) > 65536:
            print('overflow in pypilot socket')
            self.socket.close()
    
    def flush(self):
        if not self.out_buffer:
            return
        try:
            if not self.pollout.poll(0):
                if self.sendfail_cnt >= self.sendfail_msg:
                    print('pypilot socket failed to send', self.sendfail_cnt)
                    self.sendfail_msg *= 10
                self.sendfail_cnt += 1

                if self.sendfail_cnt > 100:
                    self.socket.close()
                return
            t0 = time.time()
            count = self.socket.send(self.out_buffer.encode())
            t1 = time.time()

            if t1-t0 > .1:
                print('socket send took too long!?!?', t1-t0)
            if count < 0:
                print('socket send error', count)
                self.socket.close()
            self.out_buffer = self.out_buffer[count:]
        except Exception as e:
            print('pypilot socket exception', e)
            self.socket.close()
except Exception as e:
  print('falling back to python nonblocking socket', e)
  class LineBufferedNonBlockingSocket(object):
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
            count = self.socket.send(self.out_buffer.encode())
            if count < 0:
                print('socket send error in server flush')
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
