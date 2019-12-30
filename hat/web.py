#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from flask import Flask, render_template, session, request
from flask_socketio import SocketIO, Namespace, emit, disconnect
import time, math, multiprocessing, select, os
from signalk.pipeserver import NonBlockingPipe

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, async_mode=None)

web_port = 33333

@app.route('/')
def index():
    return render_template('index.html', async_mode=socketio.async_mode, web_port=web_port)

def generate_button_row(buttonid, name):
    return '<tr><td><button id="' + buttonid + '">' + name + '</button></td><td><span id="action' + buttonid + 'codes"></span></td></tr>'

class WebConfig(Namespace):
    def __init__(self, name):
        super(Namespace, self).__init__(name)
        socketio.start_background_task(target=self.background_thread)

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
            codes = {'name': action.name, 'codes': action.codes}
            socketio.emit('action_codes', codes)
        
    def on_connect(self):
        self.emit_codes()
        print('Client connected', request.sid)

    def on_disconnect(self):
        print('Client disconnected', request.sid)

    def background_thread(self):

        last_code = False
        last_code_time = time.time()
        print('processing clients')
        x = 0
        polls_sent = {}
        while True:
            socketio.sleep(.25)

            t = time.time()
            dtc = t - last_code_time
            if dtc > 8 and last_code:
                last_code = False
                socketio.emit('code', 'N/A')
                socketio.emit('action', '')

            continue
            msg = self.pipe.recv()
            if 'code' in v:
                last_code = v['code']
                last_code_time = time.time()
            for name in msg:
                socketio.emit(name, str(msg[name]))

socketio.on_namespace(WebConfig(''))

def web_process(pipe):
    path = os.path.dirname(__file__)
    os.chdir(os.path.abspath(path))
    socketio.run(app, debug=False, host='0.0.0.0', port=web_port)
    
class Web(object):
    def __init__(self):
        self.pipe, pipe = NonBlockingPipe('pipe')
        self.process = multiprocessing.Process(target=web_process, args=(pipe,))
        self.process.start()
        import atexit, signal
        def cleanup():
            os.kill(self.process.pid, signal.SIGTERM)

        atexit.register(cleanup) # get backtrace

if __name__ == '__main__':
    web_process(None)
