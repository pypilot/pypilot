#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  


import wifi_esp32
import sys
import socket, time, json
import errno

DEFAULT_PORT = 23322

def gettime():
    return time.ticks_us()/1e6


class pypilotClient(object):
    def __init__(self, host=False):
        self.connection = False
        self.connection_in_progress = False
        self.host = host
        self.watches = {}
        self.wwatches = {}
        self.values = {}
        self.lastlinetime = gettime()
        self.addr = False
        self.need_values = False
        self.valuesbuffer = ''

        self.udp_port = 8317
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind(('0.0.0.0', self.udp_port))
        self.udp_socket.settimeout(0)

    def disconnect(self, close=True):
        if not self.connection:
            return

        if close:
            self.connection.close()
        self.connection = False
        time.sleep(.25)
        
    def connect(self):
        if self.connection or not self.host:
            return False

        sys.stdout.write('connect to host: ' + self.host + ' ... ')
        if not self.addr or self.addr[1] != self.host:
            addr_info = socket.getaddrinfo(self.host, DEFAULT_PORT)
            self.addr = addr_info[0][-1], self.host
    
        try:
            connection = socket.socket()
        except Exception as e:
            print("couldn't create socket", e)
            import machine
            machine.reset()
        
        self.wwatches = {}
        for name, value in self.watches.items():
            self.wwatches[name] = value # resend watches
        #self.values = {}
        connection.settimeout(1)
        
        try:
            connection.connect(self.addr[0])
        except OSError as e:
            if e.args[0] is errno.EHOSTUNREACH:
                print('unreachable.. restarting')
                import machine
                machine.reset()
            if not (e.args[0] is errno.EINPROGRESS):
                print('failed to connect', e)
                connection.close()
                return False

        print('connected!')
        connection.settimeout(0)
        self.connection_in_progress = connection
        return True

    def reset_timeout(self):
        self.lastlinetime = gettime()+1.5


    def decode_line(self, line, msgs):
        #print('decode line...', line)
        self.lastlinetime = gettime()
        try:
            name, data = line.split('=', 1)
            if name == 'error':
                print('server error:', data)
            else:
                value = json.loads(data.rstrip())
                if name == 'values':
                    print('values should not hit here!!!!!!')
                else:
                    msgs[name] = value
        except Exception as e:
            print('failed decoding line', e)
            print('line', line)

            f=open('badline', 'w')
            f.write(line)
            f.close()
                    
                    
    def receive(self):
        t0 = gettime()
        if not self.connection:
            if self.connection_in_progress:
                self.connection = self.connection_in_progress
                self.connection_in_progress = False
                self.valuesbuffer = ''
                if self.udp_port:
                    self.set('udp_port', self.udp_port)
                self.requested_values = False
            else:
                self.connect()
                time.sleep(.05)
                return {}

        if not self.values and self.need_values:
            if not self.requested_values:
                self.requested_values = True
                self.wwatches['values'] = True # watch values
            
        # inform server of any watches we have changed
        if self.wwatches:
            self.set('watch', self.wwatches)
            self.wwatches = {}

        msgs = {}
        some_lines = False
        while self.udp_socket:
            try:
                data, addr = self.udp_socket.recvfrom(512)
                lines = data.decode().rstrip().split('\n')
            except OSError as e:
                if e.args[0] is errno.EAGAIN:
                    pass
                else:
                    print('os error', e)
                break
            except Exception as e:
                print('udp socket exception!?!', e)
                import machine
                machine.reset() # reboot
            for line in lines:
                self.decode_line(line, msgs)
            some_lines = not not lines

        t1 = gettime()
        while self.connection:
            line = False
            try:
                line = self.connection.readline(300)
                if not line:
                    break
            except OSError as e:
                if e.args[0] is errno.EAGAIN:
                    break
                if e.args[0] is errno.ETIMEDOUT:
                    break
                print('OSerror', e)
                self.disconnect(False)
                break

            if self.valuesbuffer:
                self.valuesbuffer += line.decode()
                line = ''
            elif line.startswith('values={'):
                self.valuesbuffer = line[8:].decode()
                line = ''

            while self.valuesbuffer:
                curly = 0
                try:
                    name, rest = self.valuesbuffer.split(':', 1)
                except Exception as e:
                    if self.valuesbuffer.startswith(' }\n'):
                        line = self.valuesbuffer[3:]
                        self.valuesbuffer = ''
                    break
 
                for i in range(len(rest)):
                    c = rest[i]
                    if c == '{':
                        curly += 1
                    elif c == '}':
                        curly -= 1                   
                    if curly == 0:
                        data = rest[:i+1]
                        fields = ['AutopilotGain', 'min', 'max', 'choices']
                        for field in fields:
                            if field in data:
                                data = json.loads(data)
                                self.reset_timeout()

                                info = {}
                                for field in fields:
                                    if field in data:
                                        info[field] = data[field]
                                if info:
                                    name = json.loads(name)
                                    #print('name ', name, ' = ', info)
                                    self.values[name] = info
                                break

                        j = i
                        while rest[i] != ',' and i < len(rest)-1:
                            i += 1
                        else:
                            i = j
                        self.valuesbuffer=' '+rest[i+1:]
                        break
                else:
                    break

            if gettime() - t0 > .5: # .5 second maximum
                break
                
            if self.valuesbuffer:
                continue
            if len(line) < 4:
                self.valuesbuffer = ''                
                continue

            if line[-1] == ord('\n'):
                    self.decode_line(line.decode(), msgs)
                    some_lines = True
            else:
                print('overflow messages!', len(line), line)
        t2 = gettime()

        #print('timez', t2-t1, t1-t0)

        if not some_lines:
            t = gettime()
            dt = t - self.lastlinetime            
            #if dt > 1.0:
            #    print('upy_client: dt', dt, t)
            if dt > 2.5:
                print('upy_client: timeout on socket', dt, 'reset wifi')
                from wifi_esp32 import connect
                connect()
                self.disconnect()
                
        return msgs

    def list_values(self):
        self.need_values = True
        return False

    def get_values(self):
        if self.valuesbuffer:
            return {}
        return self.values
    
    def watch(self, name, period=True):
        if name in self.watches and self.watches[name] is period:
            return
        self.wwatches[name] = period
        if period:
            self.watches[name] = period
        elif name in self.watches:
            del self.watches[name]
        
    def set(self, name, value):
        if not self.connection:
            return
        try:
            line = json.dumps(value)+'\n'
            self.reset_timeout()
            self.connection.send(name + '=' + line)
        except OSError as e:
            if not (e.args[0] is errno.EINPROGRESS):
                print('failed to set', e)
                self.disconnect(False)
                return False
        except Exception as e:
            print('failed to set', name, value, e)
            self.disconnect()

def main():
    client = pypilotClient('192.168.14.1')
    #client.watch('imu.heading') # fastest rate
    client.watch('imu.frequency', 1.0) # once per second
    client.watch('ap.heading', .25) # once per second

    while True:
        msgs = client.receive()
        if not msgs:
            time.sleep(.03)
            continue

        for name, value in msgs.items():
            print(name, '=', value)
            
if __name__ == '__main__':
    main() 

