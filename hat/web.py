#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from flask import Flask, render_template, request, Markup
from flask_socketio import SocketIO, Namespace, emit, disconnect
import time, math, select, os

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, async_mode=None)

web_port = 33333

default_actions = \
    {"auto": ["ir030C1000", "ir030C1800", "KEY_POWER", "gpio17"],
     "menu": ["ir030D1000", "ir030D1800", "KEY_MUTE", "gpio23"],
     "port1": ["ir03201800", "ir03201000", "KEY_UP", "gpio27"],
     "starboard1": ["ir03211800", "ir03211000", "KEY_DOWN", "gpio22"],
     "select": ["ir030B1000", "ir030B1800", "KEY_SELECT", "gpio18"],
     "port10": ["ir03111800", "ir03111000", "KEY_LEFT", "gpio6"],
     "starboard10": ["ir03101800", "ir03101000", "KEY_RIGHT", "gpio5"],
     "tack": ["gpio26"]}

class WebConfig(Namespace):
    def __init__(self, name, pipe, config):
        super(Namespace, self).__init__(name)
        socketio.start_background_task(target=self.background_thread)
        self.pipe = pipe
        self.config = config
        self.status = 'N/A'

        self.last_key = False
        
        acts = ''
        names = Markup('[')
        cols = 1
        col = 0
        acts += Markup('<p>Actions for LCD interface<table border=0>')
        i = 0
        actions = config['actions']
        for name in actions:
            if i == 8:
                acts += Markup('</tr></table>')
                acts += Markup('<p><br>key: <b><span id="key0"></span></b>')
                acts += Markup('<br>action: <b><span id="action0"></span></b>')
                acts += Markup('<p>These actions do not depend on the state')
                acts += Markup(' of the display and can be used by wireless remotes.')
                acts += Markup('<table border=0>')
                col = 0
            i+=1
    
            if col == 0:
                acts += Markup('<tr>')
            acts += Markup('<td><button id="action_' + name + '">' +
                           name + '</button></td><td><span id="action' +
                           name + 'keys"></span></td>')
            if col == cols-1:
                acts += Markup('</tr>')
                col = 0
            else:
                col += 1
            names += Markup('"' + name + '", ')

        acts += Markup('</table>')
        acts += Markup('<p><br>key: <b><span id="key1"></span></b>')
        acts += Markup('<br>action: <b><span id="action1"></span></b>')

        names += Markup('""]')

        ir = Markup('<input type="radio" id="pi_ir" name="ir"')
        if config['pi.ir']:
            ir += Markup(' checked')
        ir += Markup(' /> raspberry')
        ir += Markup('<input type="radio" id="arduino_ir" name="ir"')
        if config['arduino.ir']:
            ir += Markup(' checked')
        ir += Markup(' /> arduino')
 
        nmea =  Markup('<input type="checkbox" id="arduino_nmea_in"')
        if config['arduino.nmea.in']:
            nmea += Markup(' checked')
        nmea += Markup('/> Input<input type="checkbox" id="arduino_nmea_out"')
        if config['arduino.nmea.out']:
            nmea += Markup(' checked')
        nmea += Markup('/> Output<select id="arduino_nmea_baud">')
        for baud in [4800, 38400]:
            nmea += Markup('<option value=' + str(baud))
            if baud == config['arduino.nmea.baud']:
                nmea += Markup(' selected')
            nmea += Markup('>' + str(baud) + '</option>')
        nmea += Markup('</select>')

        @app.route('/')
        def index():
            return render_template('index.html', async_mode=socketio.async_mode, web_port=web_port, actionkeys = acts, action_names = names, ir_settings = ir, nmea_settings = nmea)

    def on_ping(self):
        emit('pong')

    def on_keys(self, command):
        actions = config['actions']
        if command == 'clear':
            for name in actions:
                actions[name] = []
            self.emit_keys()
            return

        if command == 'default':
            for name in actions:
                actions[name] = []

            for name, keys in default_action_keys.items():
                actions[name] = keys.copy()

            self.emit_keys()
            return

        if not self.last_key:
            return
        
        action_keys = {}
        # remove this key from any actions
        for name, keys in self.actions.items():
            while self.last_key in keys:
                keys.remove(self.last_key)

        # add the last key to the action
        self.actions[command].append(self.last_key)
        self.emit_keys()

    def on_config(self, config):
        self.pipe.send(config)

    def emit_keys(self):
        actions = self.config['actions']
        for name, keys in actions.items():
            keys = {'name': name, 'keys': keys}
            socketio.emit('action_keys', keys)
        self.pipe.send({'actions': actions})
        
    def on_connect(self):
        self.emit_keys()
        print('web client connected', request.sid)
        socketio.emit('status', self.status)

    def on_disconnect(self):
        print('web client disconnected', request.sid)

    def background_thread(self):
        print('web process on port', web_port)
        last_key_time = time.monotonic()
        x = 0
        polls_sent = {}
        while True:
            socketio.sleep(.2)

            t = time.monotonic()
            dtc = t - last_key_time
            if dtc > 8 and self.last_key:
                self.last_key = False
                socketio.emit('key', 'N/A')
                socketio.emit('action', '')

            if not self.pipe:
                continue

            while True:
                msg = self.pipe.recv()
                if not msg:
                    break

                if 'key' in msg:
                    self.last_key = msg['key']
                    last_key_time = time.monotonic()
                for name in msg:
                    socketio.emit(name, str(msg[name]))
                if 'status' in msg:
                    self.status = msg['status']
                    socketio.emit('status', self.status)

def web_process(pipe, config):
    print('web process', os.getpid())
    path = os.path.dirname(__file__)
    os.chdir(os.path.abspath(path))
    socketio.on_namespace(WebConfig('', pipe, config))
    socketio.run(app, debug=False, host='0.0.0.0', port=web_port)
    
if __name__ == '__main__':
    web_process(None, [])
