#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from __future__ import print_function
import time, sys, os
from flask import Flask, render_template, session, request, Markup

from flask_socketio import SocketIO, Namespace, emit, join_room, leave_room, \
    close_room, rooms, disconnect
from pypilot.server import LineBufferedNonBlockingSocket
from pypilot import pyjson

pypilot_web_port=80
if len(sys.argv) > 1:
    pypilot_web_port=int(sys.argv[1])
else:
    filename = os.getenv('HOME')+'/.pypilot/web.conf'
    try:
        file = open(filename, 'r')
        config = pyjson.loads(file.readline())
        if 'port' in config:
            pypilot_web_port = config['port']
        file.close()
    except:
        print('using default port of', pypilot_web_port)


# Set this variable to 'threading', 'eventlet' or 'gevent' to test the
# different async modes, or leave it set to None for the application to choose
# the best option based on installed packages.
async_mode = None

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, async_mode=async_mode)

DEFAULT_PORT = 21311

import networking


@app.route('/')
def index():
    return render_template('index.html', async_mode=socketio.async_mode, pypilot_web_port=pypilot_web_port, tinypilot=networking.tinypilot)

class pypilotWeb(Namespace):
    def __init__(self, name):
        super(Namespace, self).__init__(name)
        socketio.start_background_task(target=self.background_thread)
        self.clients = {}

    def background_thread(self):
        print('processing clients')
        x = 0
        while True:
            socketio.sleep(.25)
            sys.stdout.flush() # update log
            for sid in self.clients:
                client = self.clients[sid]
                msgs = self.client.receive()
                socketio.emit('pypilot', msgs, room=sid)

    def on_pypilot(self, message):
        self.clients[request.sid].send(message + '\n')

    def on_ping(self):
        emit('pong')

    def on_connect(self):
        client = pypilotClient()
        socketio.emit('pypilot_connect', client.list_values())
        self.clients[request.sid] = client
        print('Client connected', request.sid)

    def on_disconnect(self):
        client = self.clients[request.sid].client
        client.connection.close()
        del self.clients[request.sid]
        print('Client disconnected', request.sid)

socketio.on_namespace(pypilotWeb(''))

def main():
    import os
    path = os.path.dirname(__file__)
    os.chdir(os.path.abspath(path))
    port = pypilot_web_port
    while True:
        try:
            socketio.run(app, debug=False, host='0.0.0.0', port=port)
            break
        except PermissionError as e:
            print('failed to run socket io on port', port, e)
            port +=8000 - 80
            print('trying port', port)

if __name__ == '__main__':
    main()
