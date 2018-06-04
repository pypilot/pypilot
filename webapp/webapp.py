#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time, sys
from flask import Flask, render_template, session, request
from flask_socketio import SocketIO, Namespace, emit, join_room, leave_room, \
    close_room, rooms, disconnect
from signalk.server import LineBufferedNonBlockingSocket

pypilot_webapp_port=80
if len(sys.argv) > 1:
    pypilot_webapp_port=int(sys.argv[1])

# Set this variable to "threading", "eventlet" or "gevent" to test the
# different async modes, or leave it set to None for the application to choose
# the best option based on installed packages.
async_mode = None

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, async_mode=async_mode)

import select, socket, json
DEFAULT_PORT = 21311


@app.route('/')
def index():
    return render_template('index.html', async_mode=socketio.async_mode, pypilot_webapp_port=pypilot_webapp_port)

class MyNamespace(Namespace):
    def __init__(self, name):
        super(Namespace, self).__init__(name)
        socketio.start_background_task(target=self.background_thread)
        self.client = False
        self.polls = {}

    def connect_signalk(self):
        print 'connect...'
        connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        socketio.emit('flush') # unfortunately needed to awaken socket for client messages
        try:
            connection.connect(('localhost', DEFAULT_PORT))
        except:
            socketio.sleep(2)
            return
        print 'connected'

        self.client = LineBufferedNonBlockingSocket(connection)
        self.client.send('{"method": "list"}\n')
        self.client.flush()

        self.poller = select.poll()
        self.poller.register(connection, select.POLLIN)
        t0 = time.time()
        self.list_values = {}
        while time.time() - t0 < 3:
            self.client.recv()
            line = self.client.readline()
            if line:
                self.list_values = line
                return

        self.client.socket.close()
        self.client = False

    def background_thread(self):
        print 'processing clients'
        x = 0
        polls_sent = {}
        while True:
            socketio.sleep(.25)
            if self.client:
                polls = {}
                for sid in self.polls:
                    for poll in self.polls[sid]:
                        polls[poll] = True
                t = time.time()
                for message in polls:
                    if not message in polls_sent or \
                       t - polls_sent[message] > 1:
                        #print 'msg', message
                        self.client.send(message + '\n')
                        polls_sent[message] = t
                    
                self.client.flush()

                events = self.poller.poll(0)
                if not events:
                    continue
                
                event = events.pop()
                fd, flag = event
                if flag & (select.POLLHUP | select.POLLERR | select.POLLNVAL) \
                   or not self.client.recv():
                    print 'disconnected'
                    self.client.socket.close()
                    socketio.emit('signalk_disconnect', self.list_values)
                    self.client = False
                    continue

                while True:
                    try:
                        line = self.client.readline()
                        if not line:
                            break
                        socketio.emit('signalk', line.rstrip())
                    except:
                        socketio.emit('log', line)
                        print 'error: ', line.rstrip()
                        break
            else:
                if self.polls:
                    self.connect_signalk()
                if self.client:
                    socketio.emit('signalk_connect', self.list_values)


    def on_signalk(self, message):
        #print 'msg',  message
        self.client.send(message + '\n')

    def on_signalk_poll(self, message):
        #print 'message', message
        if message == 'clear':
            self.polls[request.sid] = {}
            return
        self.polls[request.sid][message] = True               

    #def on_disconnect_request(self):
    #    disconnect()

    def on_ping(self):
        emit('pong')

    def on_connect(self):
        #self.clients[request.sid] = Connection()
        #print('Client connected', request.sid, len(self.clients))
        print('Client connected', request.sid)
        self.polls[request.sid] = {}
        if self.client:
            socketio.emit('signalk_connect', self.list_values)

    def on_disconnect(self):
        #client = self.clients[request.sid].client
        #if client:
        #    client.socket.close()
        del self.polls[request.sid]
        if not self.polls:
            self.client.socket.close()
            self.client = False
            print 'closed signalk client'
        print('Client disconnected', request.sid, len(self.polls))

socketio.on_namespace(MyNamespace(''))

def main():
    import os
    path = os.path.dirname(__file__)
    os.chdir(os.path.abspath(path))
    socketio.run(app, debug=False, host='0.0.0.0', port=pypilot_webapp_port)

if __name__ == '__main__':
    main()
