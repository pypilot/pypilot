#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from __future__ import print_function
import os, sys, time, json
import spidev
from flask import Flask, render_template, session, request
from flask_socketio import SocketIO, Namespace, emit, disconnect
from signalk.client import SignalKClient

host='';
if len(sys.argv) > 1:
    host = sys.argv[1]

config_path = os.getenv('HOME') + '/.pypilot/rf.conf' 

# Set this variable to 'threading', 'eventlet' or 'gevent' to test the
# different async modes, or leave it set to None for the application to choose
# the best option based on installed packages.
async_mode = None

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, async_mode=async_mode)

@app.route('/')
def index():            
    return render_template('index.html', async_mode=socketio.async_mode, webapp_port=33333)

class RFAction(object):
    def  __init__(self, rf, name):
        self.rf = rf
        self.name = name
        self.codes = []
    
class RFActionSignalK(RFAction):
    def  __init__(self, rf, name, signalk_name, signalk_value):
        super(RFActionSignalK, self).__init__(rf, name)
        self.signalk_name = signalk_name
        self.value = signalk_value

    def trigger(self):
        if self.rf.client:
            self.rf.client.set(self.signalk_name, self.value)

class RFActionEngage(RFActionSignalK):
    def  __init__(self, rf):
        super(RFActionEngage, self).__init__(rf, 'engage', 'ap.enabled', True)

    def trigger(self):
        super(RFActionEngage, self).trigger()
        if self.rf.client:
            self.rf.client.set('ap.heading_command', self.rf.last_values['ap.heading'])
            
class RFActionHeading(RFAction):
    def __init__(self, rf, offset):
        super(RFActionHeading, self).__init__(rf, str(offset))
        self.offset = offset

    def trigger(self):
        if self.rf.client:
            if self.rf.last_values['ap.enabled']:
                self.rf.client.set('ap.heading_command',
                                   self.rf.last_values['ap.heading_command'] + self.offset)
            else: # manual mode
                self.rf.servo_timeout = time.time() + abs(self.offset)**.5/2
                self.rf.client.set('servo.command', 1 if self.offset > 0 else -1)

class RFActionTack(RFActionSignalK):
    def  __init__(self, rf, name, direction):
        super(RFActionTack, self).__init__(rf, name, 'ap.tack.state', 'begin')
        self.direction = direction
                                
    def trigger(self):
        super(RFActionEngage, self).trigger()
        if self.rf.client:
            self.rf.client.set('ap.tack.direction', self.direction)

class RFWebConfig(Namespace):
    def __init__(self, name):
        super(Namespace, self).__init__(name)
        socketio.start_background_task(target=self.background_thread)

        self.client = False
        self.last_values = {}
        self.last_code = False
        self.last_code_time = time.time()
        self.servo_timeout = time.time() + 1

        self.actions = [RFActionEngage(self),
                        RFActionSignalK(self, 'disengage', 'ap.enabled', False),
                        RFActionHeading(self, 1),
                        RFActionHeading(self, -1),
                        RFActionHeading(self, 2),
                        RFActionHeading(self, -2),
                        RFActionHeading(self, 10),
                        RFActionHeading(self, -10),
                        RFActionSignalK(self, 'compassmode', 'ap.mode', 'compass'),
                        RFActionSignalK(self, 'gpsmode', 'ap.mode', 'gps'),
                        RFActionSignalK(self, 'windmode', 'ap.mode', 'wind'),
                        RFActionTack(self, 'tackport', 'port'),
                        RFActionTack(self, 'tackstarboard', 'starboard')]
        self.read_config()

    def read_config(self):
        config = {}
        try:
            file = open(config_path)
            config = json.loads(file.read())
            file.close()
        except:
            pass

        for action in self.actions:
            if action.name in config:
                action.codes = config[action.name]

    def write_config(self):
        config = {}
        for action in self.actions:
            config[action.name] = action.codes
                                
        try:
            file = open(config_path, 'w')
            file.write(json.dumps(config) + '\n')
        except IOError:
            print('failed to save config file:', self.configfilename)
        

    def on_ping(self):
        emit('pong')

    def on_codes(self, command):
        if not self.last_code:
            return

        # remove this code from any actions
        for action in self.actions:
            while self.last_code in action.codes:
                action.codes.remove(self.last_code)

        # add the last code to the action
        for action in self.actions:
            if command == action.name:
                action.codes.append(self.last_code)
                break

        self.emit_codes()
        self.write_config()

    def emit_codes(self):
        for action in self.actions:
            codes = {'name': action.name, 'codes': map(lambda code : "%X" % code, action.codes)}
            socketio.emit('action_codes', codes)
        
    def on_connect(self):
        #self.clients[request.sid] = Connection()
        if self.client:
            socketio.emit('pypilot', 'Connected')
        self.emit_codes()
        print('Client connected', request.sid)

    def on_disconnect(self):
        print('Client disconnected', request.sid)

    def apply_code(self, code):
        socketio.emit('rfcode', "%X" % code)
        self.last_code = code
        self.last_code_time = time.time()
        for action in self.actions:
            if code in action.codes:
                socketio.emit('action', action.name)
                action.trigger()

    def background_thread(self):
        try:
            spi = spidev.SpiDev()
            spi.open(0, 1)
            spi.max_speed_hz=5000
        except Exception as e:
            print('failed to open spi device', e)
            exit(1)

        watchlist = ['ap.enabled', 'ap.heading_command']
        for name in watchlist:
            self.last_values[name] = 0

        def on_con(client):
            for name in watchlist:
                client.watch(name)
            socketio.emit('pypilot', 'Connected')

        self.client = False
        dt = 0
        lastpollheading = time.time()
        while True:
            if self.client:
                try:
                    while True:
                        msg = self.client.receive_single()
                        if not msg:
                            break
                        name, value = msg
                        self.last_values[name] = value['value']

                except Exception as e:
                    socketio.emit('pypilot', 'Disconnected' + str(e))
                    self.client = False

            if not self.client:
                socketio.sleep(3)
                try:
                    self.client = SignalKClient(on_con, host)
                    print('connected', host)
                except Exception as e:
                    print('failed to connect', e)

            t = time.time()
            dtc = t - self.last_code_time
            if dtc > 8 and self.last_code:
                self.last_code = False
                socketio.emit('rfcode', 'N/A')
                socketio.emit('action', '')

            # poll heading once per second if not enabled
            dtp = t - lastpollheading
            if self.client and dtp > 1 and not self.last_values['ap.enabled']:
                self.client.get('ap.heading')
                lastpollheading = t

            # timeout manual move
            if self.servo_timeout:
                dtt = t - self.servo_timeout
                if dtt > 0:
                    self.client.set('servo.command', 0) #stop
                    self.servo_timeout = 0
            
            x = spi.xfer([0, 0, 0, 0])
            if not any(x):
                socketio.sleep(dt)
                dt = min(.1, dt+.001)
                continue
            dt = 0
                                
            for i in range(4):
                if not x[0] and x[1] and x[2] and x[3]: # ok
                    code = (x[1]*256 + x[2])*256 + x[3]
                    if self.last_code != code or dtc > .6:
                        self.apply_code(code)
                    break
                
                x = x[1:] + [spi.xfer([0])[0]]

        spi.close()

socketio.on_namespace(RFWebConfig(''))

def main():
    path = os.path.dirname(__file__)
    os.chdir(os.path.abspath(path))
    socketio.run(app, debug=False, host='0.0.0.0', port=33333)

if __name__ == '__main__':
    main()
