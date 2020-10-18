#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import select, time, os
import pyjson

class NonBlockingPipeEnd(object):
    def __init__(self, pipe, name, recvfailok, sendfailok):
        self.pipe = pipe
        self.pollin = select.poll()
        self.pollin.register(self.pipe, select.POLLIN)
        self.pollout = select.poll()
        self.pollout.register(self.pipe, select.POLLOUT)
        self.name = name
        self.sendfailcount = 0
        self.failcountmsg = 1
        self.recvfailok = recvfailok
        self.sendfailok = sendfailok

    def fileno(self):
        return self.pipe.fileno()

    def flush(self):
        pass

    def close(self):
        self.pipe.close()

    def recv(self, timeout=0):
        try:
            if self.pollin.poll(0):
                return self.pipe.recv()
            if not self.recvfailok:
                print('error pipe block on recv!', self.name)
        except:
            print('failed to recv nonblocking pipe!', self.name)
        return False

    def recvdata(self):
        return self.pollin.poll(0)

    def readline(self):
        return self.recv() # pipe has complete lines if used for text

    def write(self, value):
        self.send(value)
    
    def send(self, value, block=False):
        t0=time.time()
        if block or self.pollout.poll(0):
            t1=time.time()
            self.pipe.send(value)
            t2=time.time()
            if t2-t0 > .001:
                print('too long!', t2-t0, self.name)
            return True

        if self.sendfailok:
            return False

        self.sendfailcount += 1
        if self.sendfailcount == self.failcountmsg:
            print('pipe full (%d)' % self.sendfailcount, self.name, 'cannot send')
            self.failcountmsg *= 10
        return False


from bufferedsocket import LineBufferedNonBlockingSocket
class SocketNonBlockingPipeEnd(LineBufferedNonBlockingSocket):
    def __init__(self, socket, name, recvfailok, sendfailok):
        self.name = name
        super(SocketNonBlockingPipeEnd, self).__init__(socket, name)

    def recv(self, timeout=0):
        self.recvdata()
        line = super(SocketNonBlockingPipeEnd, self).readline()
        if not line:
            return
        try:
            d = pyjson.loads(line.rstrip())
            return d
        except Exception as e:
            print('failed to decode data socket!', self.name, e)
            print('line', line)
        return False

    def send(self, value, block=False):
        t0 = time.time()
        try:
            data = pyjson.dumps(value)
            self.write(data+'\n')
            t1 = time.time()
            if t1-t0 > .02:
                print('too long', t1-t0, self.name, len(data))
            return True
        except Exception as e:
            print('failed to encode data socket!', self.name, e)
            return False

try:
    from pypilot.linebuffer import linebuffer
except:
    import failedimports

class PipeNonBlockingPipeEnd(object):
    def __init__(self, r, w, name, recvfailok, sendfailok):
        self.name = name
        self.r, self.w = r, w
        os.set_blocking(r, False)
        os.set_blocking(w, False)
        self.b = linebuffer.LineBuffer(r)
        self.pollout = select.poll()
        self.pollout.register(self.w, select.POLLOUT)
        self.recvfailok = recvfailok
        self.sendfailok = sendfailok

    def fileno(self):
        return self.r
        
    def close(self):
        os.close(self.r)
        os.close(self.w)
        
    def recvdata(self):
        return self.b.recv()
        
    def readline(self):
        return self.b.line()
        
    def recv(self, timeout=0):
        self.recvdata()
        line = self.b.line()
        if not line:
            return
        try:
            d = pyjson.loads(line.rstrip())
            return d
        except Exception as e:
            print('failed to decode data socket!', self.name, e)
            print('line', line)
        return False

    def flush(self):
        pass

    def write(self, data):
        if not self.pollout.poll(0):
            if not self.sendfailok:
                print('failed write', self.name)
        t0 = time.time()
        os.write(self.w, data.encode())
        t1 = time.time()
        if t1-t0 > .024:
            print('too long write pipe', t1-t0, self.name, len(data))
    
    def send(self, value, block=False):
        if not self.pollout.poll(0):
            if not self.sendfailok:
                print('failed send', self.name)
        t0 = time.time()
        try:
            data = pyjson.dumps(value) + '\n'
            os.write(self.w, data.encode())
            t1 = time.time()
            self.flush()
            t2 = time.time()
            if t2-t0 > .024:
                print('too long send nonblocking pipe', t1-t0, t2-t1, self.name, len(data))
            return True
        except Exception as e:
            if not self.sendfailok:
                print('failed to encode data pipe!', self.name, e)
            return False

# non multiprocessed pipe emulates functions in a simple queue
class NoMPLineBufferedPipeEnd(object):
    def __init__(self, name):
        self.name = name
        self.lines = []

    def fileno(self):
        return 0

    def flush(self):
        pass

    def close(self):
        pass

    def write(self, data):
        self.send(data)
    
    def recv(self, timeout=0):
        return self.readline()

    def readline(self):
        if not self.lines:
            return False
        ret = self.lines[0]
        self.lines = self.lines[1:]
        return ret

    def send(self, value):
        if len(self.remote.lines) >= 1000:
            return False
        self.remote.lines.append(value)
        return True
        

def NonBlockingPipe(name, use_multiprocessing, recvfailok=True, sendfailok=False):
    if use_multiprocessing:
        if 1:
            # os pipe has lowest cpu usage
            r0, w0 = os.pipe()
            r1, w1 = os.pipe()
            return PipeNonBlockingPipeEnd(r0, w1, name+'[0]', recvfailok, sendfailok), PipeNonBlockingPipeEnd(r1, w0, name+'[1]', recvfailok, sendfailok)
        elif 1:
            # python pipe highest cpu
            import multiprocessing
            pipe = multiprocessing.Pipe()
            return NonBlockingPipeEnd(pipe[0], name+'[0]', recvfailok, sendfailok), NonBlockingPipeEnd(pipe[1], name+'[1]', recvfailok, sendfailok)
        else:
            # socket pair inbetween python and os pipe for cpu usage
            import socket
            socket = socket.socketpair()
            return SocketNonBlockingPipeEnd(socket[0], name+'[0]', recvfailok, sendfailok), SocketNonBlockingPipeEnd(socket[1], name+'[1]', recvfailok, sendfailok)

    pipe = NoMPLineBufferedPipeEnd(name+'[0]'), NoMPLineBufferedPipeEnd(name+'[1]')
    pipe[0].remote = pipe[1]
    pipe[1].remote = pipe[0]
    return pipe
