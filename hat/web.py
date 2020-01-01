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
        self.pypilot = 'N/A'

        acts = ''
        for action in actions:
            acts += Markup('<p><button id="' + action.name + '">' + action.name + '</button><span id="action' + action.name + 'keys"></span></p>')

        @app.route('/')
        def index():
            return render_template('index.html', async_mode=socketio.async_mode, web_port=web_port, actionkeys = acts)

    def on_ping(self):
        emit('pong')

    def on_keys(self, command):
        if not self.last_key:
            return

        # remove this key from any actions
        for action in self.actions:
            while self.last_key in action.keys:
                action.keys.remove(self.last_key)

        # add the last key to the action
        for action in self.actions:
            if command == action.name:
                action.keys.append(self.last_key)
                break

        self.emit_keys()
        self.write_config()

    def emit_keys(self):
        for action in self.actions:
            keys = {'name': action.name, 'keys': action.keys}
            socketio.emit('action_keys', keys)
        
    def on_connect(self):
        self.emit_keys()
        socketio.emit('pypilot', self.pypilot)
        print('Client connected', request.sid)

    def on_disconnect(self):
        socketio.emit('pypilot', self.pypilot)
        print('Client disconnected', request.sid)

    def background_thread(self):
        last_key = False
        last_key_time = time.time()
        print('processing clients')
        x = 0
        polls_sent = {}
        while True:
            socketio.sleep(.25)

            t = time.time()
            dtc = t - last_key_time
            if dtc > 8 and last_key:
                last_key = False
                socketio.emit('key', 'N/A')
                socketio.emit('action', '')

            if not self.pipe:
                continue
            
            msg = self.pipe.recv()
            if msg:
                if 'key' in msg:
                    last_key = msg['key']
                    last_key_time = time.time()
                for name in msg:
                    print('emit', name, str(msg[name]))
                    socketio.emit(name, str(msg[name]))
                if 'pypilot' in msg:
                    self.pypilot = msg['pypilot']

def web_process(pipe, actions):
    path = os.path.dirname(__file__)
    os.chdir(os.path.abspath(path))
    socketio.on_namespace(WebConfig('', pipe, actions))
    socketio.run(app, debug=False, host='0.0.0.0', port=web_port)
    
if __name__ == '__main__':
    web_process(None, [])
