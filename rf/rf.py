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
from flask_socketio import SocketIO, Namespace, emit, join_room, leave_room, \
    close_room, rooms, disconnect
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
    if request.method == 'POST':
        print('post')

        try:
            file = open(config_path, 'w')
            file.write(json.dumps(config).replace(',',  ',\n')+'\n')
            file.close()
        except Exception as e:
            print('failed writing config', e)
            
    return render_template('index.html', async_mode=socketio.async_mode, webapp_port=33333)


class RFAction():
    def  __init__(self, signalk_name, signalk_value):
        self.name = signalk_name
        if type(signalk_value) != type(lambda:None):
            self.value = lambda : signalk_value
        else:
            self.value = signalk_value
        self.codes = []

    def trigger(self, client):
        if client:
            client.set(self.name, self.value())

class RFWebConfig(Namespace):
    def __init__(self, name):
        super(Namespace, self).__init__(name)
        socketio.start_background_task(target=self.background_thread)

        self.client = False
        self.last_values = {}

        def RFActionHeading(offset):
            return RFAction('ap.heading_command', lambda : self.last_values['ap.heading_command'] + offset)            
        
        self.actions = {'engage': RFAction('ap.engaged', True),
                        'disengage': RFAction('ap.engaged', False),
                        '+1': RFActionHeading(1),
                        '-1': RFActionHeading(-1),
                        '+10': RFActionHeading(10),
                        '-10': RFActionHeading(-10)}

        config = {}
        try:
            file = open(config_path, 'w')
            config = json.loads(file.read())
            file.close()
        except:
            pass

        for name in config:
            self.actions[name].codes = config[name]

    def on_ping(self):
        emit('pong')

    def on_connect(self):
        #self.clients[request.sid] = Connection()
        if self.client:
            socketio.emit('pypilot', 'Connected')
        print('Client connected', request.sid)

    def on_disconnect(self):
        print('Client disconnected', request.sid)

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
        while True:
            if self.client:
                try:
                    while True:
                        msg = self.client.receive_single()
                        if not msg:
                            break
                        name, value = msg
                        self.last_values[name] = value

                except Exception as e:
                    socketio.emit('pypilot', 'Disconnected' + str(e))
                    self.client = False

            if not self.client:
                socketio.sleep(3)
                try:
                    self.client = SignalKClient(on_con, host)
                except Exception as e:
                    print('failed to connect', e)

            x = spi.xfer([0, 0, 0, 0])
            if not any(x):
                socketio.sleep(dt)
                dt = min(.1, dt+.001)
                continue
            dt = 0

            for i in range(4):
                if not x[0] and x[1] and x[2] and x[3]: # ok
                    code = (x[1]*256 + x[2])*256 + x[3]
                    print ('code', code)
                    for name in self.actions:
                        action =  self.actions[name]
                        if code in action.codes:
                            action.trigger(self.client)
                    socketio.emit('rfcode', "%X" % code)
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
