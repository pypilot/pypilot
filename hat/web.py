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

class WebConfig(Namespace):
    def __init__(self, name, pipe, actions):
        super(Namespace, self).__init__(name)
        socketio.start_background_task(target=self.background_thread)
        self.pipe = pipe
        self.actions = actions
        self.status = 'N/A'

        self.last_key = False
        
        acts = ''

        names = Markup('[')
        cols = 1
        col = 0
        acts += Markup('<p>Actions for LCD interface<table border=0>')
        i = 0
        for action in actions:
            if i == 8:
                acts += Markup('</tr></table>')
                acts += Markup('<p><br>key: <b><span id="key0"></span></b>')
                acts += Markup('<br>action: <b><span id="action0"></span></b>')
                acts += Markup('<p>These actions do not depend on the state')
                acts += Markup(' of the display and are intended for wireless remotes')
                acts += Markup('<table border=0>')
                col = 0
            i+=1
    
            if col == 0:
                acts += Markup('<tr>')
            acts += Markup('<td><button id="action_' + action.name + '">' +
                           action.name + '</button></td><td><span id="action' +
                           action.name + 'keys"></span></td>')
            if col == cols-1:
                acts += Markup('</tr>')
                col = 0
            else:
                col += 1
            names += Markup('"' + action.name + '", ')

        acts += Markup('</table>')
        acts += Markup('<p><br>key: <b><span id="key1"></span></b>')
        acts += Markup('<br>action: <b><span id="action1"></span></b>')

        names += Markup('""]')

        @app.route('/')
        def index():
            return render_template('index.html', async_mode=socketio.async_mode, web_port=web_port, actionkeys = acts, action_names = names)

    def on_ping(self):
        emit('pong')

    def on_keys(self, command):
        if not self.last_key:
            return

        action_keys = {}
        # remove this key from any actions
        for action in self.actions:
            while self.last_key in action.keys:
                action.keys.remove(self.last_key)
                action_keys[action.name] = action.keys

        # add the last key to the action
        for action in self.actions:
            if command == action.name:
                action.keys.append(self.last_key)
                action_keys[action.name] = action.keys
                break

        self.pipe.send(action_keys)
        self.emit_keys()

    def on_nmea(self, config):
        self.nmeapipe.send(config)

    def emit_keys(self):
        for action in self.actions:
            keys = {'name': action.name, 'keys': action.keys}
            socketio.emit('action_keys', keys)
        
    def on_connect(self):
        self.emit_keys()
        print('Client connected', request.sid)
        socketio.emit('status', self.status)

    def on_disconnect(self):
        print('Client disconnected', request.sid)

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

def web_process(pipe, actions):
    path = os.path.dirname(__file__)
    os.chdir(os.path.abspath(path))
    socketio.on_namespace(WebConfig('', pipe, actions))
    socketio.run(app, debug=False, host='0.0.0.0', port=web_port)
    
if __name__ == '__main__':
    web_process(None, [])
