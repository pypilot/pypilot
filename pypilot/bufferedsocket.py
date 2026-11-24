#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

import codecs
import os
import select
import socket
import time

try:
    from pypilot.linebuffer import linebuffer
except Exception as e:
    print(_('falling back to python nonblocking socket, will consume more cpu'), e)
    linebuffer = None

class LineBufferedNonBlockingSocket:
    def __init__(self, connection, address):
        connection.setblocking(0)
        if linebuffer:
            self.b = linebuffer.LineBuffer(connection.fileno())
        else:
            self.b = False  # NMEASocket.readline() uses the Python parser.
            self.in_buffer = ''
            self.decoder = codecs.getincrementaldecoder('utf-8')()
            self.no_newline_pos = 0

        self.socket = connection
        self.address = address
        self.out_buffer = b''

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

    def write(self, data, udp=False):
        if udp and self.udp_port:
            self.udp_out_buffer += data
            if len(self.udp_out_buffer) > 400:
                print(_('overflow in pypilot udp socket'), self.address, len(self.udp_out_buffer))
                self.udp_out_buffer = ''
        else:
            self.out_buffer += data.encode()
            if len(self.out_buffer) > 65536:
                print(_('overflow in pypilot socket'), self.address, len(self.out_buffer), os.getpid())
                self.out_buffer = b''
                self.close()

    def flush(self):
        if self.udp_out_buffer:
            try:
                if not self.udp_socket:
                    self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                count = self.udp_socket.sendto(self.udp_out_buffer.encode(), (self.address[0], self.udp_port))
            except Exception as e:
                print('udp socket failed to send', e)
                count = 0
                self.close()
            if count != len(self.udp_out_buffer):
                print(_('failed to send udp packet'), self.address)
            self.udp_out_buffer = ''

        if not self.out_buffer:
            return

        try:
            if not self.pollout.poll(0):
                if self.sendfail_cnt >= self.sendfail_msg:
                    print(_('pypilot socket failed to send to'), self.address, self.sendfail_cnt)
                    self.sendfail_msg *= 10
                self.sendfail_cnt += 1

                if self.sendfail_cnt > 100:
                    self.close()
                return

            t0 = time.monotonic()
            count = self.socket.send(self.out_buffer)
            #print('write', count, self.out_buffer, time.monotonic())
            t1 = time.monotonic()

            if t1-t0 > .1:
                print(_('socket send took too long!?!?'), self.address, t1-t0, len(self.out_buffer))
            if count <= 0:
                print(_('socket send error'), self.address, count)
                self.close()
                return

            self.out_buffer = self.out_buffer[count:]
            self.sendfail_cnt = 0
            self.sendfail_msg = 1
        except BlockingIOError:
            return  # Keep pending bytes for the next flush.
        except Exception as e:
            print(_('pypilot socket exception'), self.address, e, os.getpid(), self.socket)
            self.close()

    def recvdata(self):
        if self.b:
            return self.b.recv()

        size = 4096
        try:
          raw = self.socket.recv(size)
          data = self.decoder.decode(raw)
        except Exception as e:
          print(_('error receiving data'), e)
          return False

        l = len(raw)
        if l == 0:
            return False

        self.in_buffer += data
        if l == size:
            return l+self.recvdata()
        return l

    def readline(self):
        if self.b:
            return self.b.line()

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
