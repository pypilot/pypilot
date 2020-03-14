#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time, select, socket, os

try:
  from pypilot.linebuffer import linebuffer
  class LineBufferedNonBlockingSocket(object):
    def __init__(self, connection, address):
        connection.setblocking(0)
        self.b = linebuffer.LineBuffer(connection.fileno())

        self.socket = connection
        self.address = address
        self.out_buffer = ''

        self.udp_port = False
        self.udp_out_buffer = ''
        self.udp_socket = False

        self.pollout = select.poll()
        self.pollout.register(connection, select.POLLOUT)
        self.sendfail_msg = 1
        self.sendfail_cnt = 0

    def fileno(self):
        if self.socket:
            return self.socket.fileno()
        return 0

    def close(self):
        if self.socket:
            self.socket.close()
            self.socket = False
        if self.udp_socket:
            self.udp_socket.close()
            self.udp_socket = False
        
    def recvdata(self):
        return self.b.recv()
        
    def readline(self):
        return self.b.line()

    def write(self, data, udp=False):
        if udp and self.udp_port:
          self.udp_out_buffer += data
          if len(self.udp_out_buffer) > 400:
            print('overflow in pypilot udp socket', self.address, len(self.udp_out_buffer))
            self.udp_out_buffer = ''
        else:
          self.out_buffer += data
          if len(self.out_buffer) > 65536:
            print('overflow in pypilot socket', self.address, len(self.out_buffer), os.getpid())
            self.out_buffer = ''
            self.close()
    
    def flush(self):
        if self.udp_out_buffer:
            if not self.udp_socket:
                  self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            count = self.udp_socket.sendto(self.udp_out_buffer.encode(), (self.address[0], self.udp_port))
            if count != len(self.udp_out_buffer):
                print('failed to send udp packet', self.address)
            self.udp_out_buffer = ''
        
        if not self.out_buffer:
            return

        try:
            if not self.pollout.poll(0):
                if self.sendfail_cnt >= self.sendfail_msg:
                    print('pypilot socket failed to send to', self.address, self.sendfail_cnt)
                    self.sendfail_msg *= 10
                self.sendfail_cnt += 1

                if self.sendfail_cnt > 100:
                    self.socket.close()
                return
            t0 = time.monotonic()
            count = self.socket.send(self.out_buffer.encode())
            t1 = time.monotonic()

            if t1-t0 > .02:
                print('socket send took too long!?!?', self.address, t1-t0, len(self.out_buffer))
            if count < 0:
                print('socket send error', self.address, count)
                self.socket.close()
            self.out_buffer = self.out_buffer[count:]
        except Exception as e:
            print('pypilot socket exception', self.address, e, os.getpid(), self.socket)
            self.close()
  
except Exception as e:
  print('falling back to python nonblocking socket, will consume more cpu', e)
  class LineBufferedNonBlockingSocket(object):
    def __init__(self, connection, address):
        connection.setblocking(0)
        self.socket = connection
        self.b = False # in python
        self.in_buffer = ''
        self.no_newline_pos = 0
        self.out_buffer = ''

    def close(self):
        self.socket.close()
        
    def fileno(self):
        return self.socket.fileno()

    def write(self, data):
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

    def recvdata(self):
        size = 4096
        try:
          data = self.socket.recv(size).decode()
        except Exception as e:
          print('error receiving data', e)
          return False

        l = len(data)
        if l == 0:
            return False

        self.in_buffer += data
        if l == size:
            return l+self.recvdata()
        return l

    def readline(self):
        while self.no_newline_pos < len(self.in_buffer):
            c = self.in_buffer[self.no_newline_pos]
            if c=='\n' or c=='\r':
                ret = self.in_buffer[:self.no_newline_pos]+'\n'
                self.in_buffer = self.in_buffer[self.no_newline_pos+1:]
                if self.no_newline_pos:
                  self.no_newline_pos = 0
                  return ret
                continue
            self.no_newline_pos += 1
        return ''
