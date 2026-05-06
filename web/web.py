#!/usr/bin/env python
#   Copyright (C) 2026 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

import os
import sys
import time
import threading
from datetime import datetime, timezone

from engineio.payload import Payload
from flask import Flask, render_template, request
from flask_socketio import Namespace, SocketIO, emit
from markupsafe import Markup

Payload.max_decode_packets = 500

from pypilot import pyjson
from pypilot.client import pypilotClient

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import gettext_helper
import tinypilot

config = {'port': 8000, 'language': 'default'}
configfilename = os.path.expanduser('~')+'/.pypilot/web.conf'

try:
    file = open(configfilename)
    config.update(pyjson.loads(file.readline()))
    file.close()

except OSError:
    print('failed to read config', configfilename)

def write_config():
    try:
        file = open(configfilename, 'w')
        file.write(pyjson.dumps(config) + '\n')
        file.close()
    except OSError:
        print('failed to write config')

if len(sys.argv) > 1:
    pypilot_web_port=int(sys.argv[1])
else:
    pypilot_web_port = config['port']

print('using port', pypilot_web_port)

# Set this variable to 'threading', 'eventlet' or 'gevent' to test the
# different async modes, or leave it set to None for the application to choose
# the best option based on installed packages.

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins="*")
_ = gettext_helper.load(app, config)

@app.route('/logs')
def logs():
    log_links = ''
    try:
        logdirs = os.listdir('/var/log')
        for name in logdirs:
            if os.path.exists(os.path.join('/var/log', name, 'current')):
                log_links+='<br><a href="log/'+name+'">'+name+'</a>'
    except Exception as e:
        print('failed to enumerate log directory', e)

    return render_template('logs.html', async_mode=socketio.async_mode, log_links=Markup(log_links))

@app.route('/log/<name>')
def log(name):
    log = ''
    try:
        f = open('/var/log/' + name + '/current')
        log = f.read()
        f.close()
    except Exception as e:
        log = _('failed to read log file') + ' "' + name + '": ' + str(e)
    r = app.make_response(log)
    r.mimetype = 'text/plain'
    return r


@app.route('/wifi', methods=['GET', 'POST'])
def wifi():
    networking = os.path.expanduser('~') + '/.pypilot/networking.txt'
    wifi = {'mode': 'Master', 'ssid': 'pypilot', 'key': '',
            'client_ssid': 'openplotter', 'client_key': '12345678', 'client_address': '10.10.10.60'}
    try:
        f = open(networking)
        while True:
            l = f.readline().rstrip()
            if not l:
                break
            try:
                name, value = l.split('=')
                wifi[name] = value.rstrip()
            except Exception as e:
                print('failed to parse line in networking.txt', e, l)
        f.close()
    except OSError:
        pass

    if request.method == 'POST':
        try:
            for name in request.form:
                wifi[name] = str(request.form[name])

            def make_valid_key(key):
                if len(key) < 8:
                    return ''
                return key[:63]
            wifi['key'] = make_valid_key(wifi['key'])
            wifi['client_key'] = make_valid_key(wifi['client_key'])
            f = open(networking, 'w')
            for name in wifi:
                f.write(name+'='+wifi[name]+'\n')
            f.close()

            os.system('/opt/networking.sh')
        except Exception as e:
            print('exception!', e)

    try:
        leases = '<table id="leases">'
        leases += '<tr><th>IP Address</th><th>Mac Address</th><th>Name</th><th>Lease ends on</th></tr>'
        DNSMASQ_LEASES_FILE = "/var/lib/misc/dnsmasq.leases"
        f = open(DNSMASQ_LEASES_FILE)
        for line in f:
            elements = line.split()
            if len(elements) == 5:
                if elements[3] == "*":
                    continue

                ts = int(elements[0])
                if ts:
                    ts = datetime.fromtimestamp(ts, timezone.utc).strftime('%Y-%m-%d %H:%M:%S')
                else:
                    ts = 'Never'

                leases += '<tr>'
                leases += '<td>' + elements[2] + '</td>'
                leases += '<td>' + elements[1] + '</td>'
                leases += '<td>' + elements[3] + '</td>'
                leases += '<td>' + ts + '</td>'
                leases += '</tr>'
        leases += '</table>'
    except Exception as e:
        print('lease fail', e)
        leases = ''
    if 'Master' not in wifi['mode']:
        leases = ''

    return render_template('wifi.html', async_mode=socketio.async_mode, wifi=Markup(wifi), leases=Markup(leases))

@app.route('/calibrationplot')
def calibrationplot():
    return render_template('calibrationplot.html', async_mode=socketio.async_mode,pypilot_web_port=pypilot_web_port)

@app.route('/client')
def client():
    return render_template('client.html', async_mode=socketio.async_mode,pypilot_web_port=pypilot_web_port)


@app.route('/')
def index():
    return render_template('index.html', async_mode=socketio.async_mode, pypilot_web_port=pypilot_web_port, tinypilot=tinypilot.tinypilot, translations=gettext_helper.translations, language=config['language'], languages=Markup(gettext_helper.LANGUAGES))

class pypilotWeb(Namespace):
    def __init__(self, name):
        super().__init__(name)
        self.clients = {}
        self.clients_lock = threading.Lock()
        socketio.start_background_task(target=self.background_thread)

    def background_thread(self):
        print('processing clients')

        while True:
            # This could be optimized with backend-specific select()/poll() on
            # pypilotClient file descriptors, but socketio has no portable fd wait API.
            # Keep receive() nonblocking and use socketio.sleep() for portability.
            socketio.sleep(.25)
            sys.stdout.flush()

            outgoing = []
            t0 = time.monotonic()

            with self.clients_lock:
                for sid, client in list(self.clients.items()):
                    try:
                        values = client.list_values()
                        if values:
                            outgoing.append(('pypilot_values', values, sid))

                        connected = client.connection

                        # Important: receive() must be nonblocking because this lock is global.
                        msgs = client.receive()
                        if msgs:
                            outgoing.append(('pypilot', msgs, sid))

                        if not connected:
                            outgoing.append(('pypilot_disconnect', None, sid))

                    except Exception as e:
                        print('client error', e)
                        self._remove_sid(sid)

            for event, data, sid in outgoing:
                if data is None:
                    socketio.emit(event, room=sid)
                else:
                    socketio.emit(event, pyjson.dumps(data), room=sid)
            t1 = time.monotonic()

            if t1-t0 > .25:
                print('took too long to process clients', t1-t0)


    def on_pypilot(self, message):
        sid = request.sid
        #print('message', message)
        with self.clients_lock:
            client = self.clients.get(sid)
            if client:
                try:
                    client.send(message + '\n')
                except Exception as e:
                    print('send error', sid, e)
                    self._remove_sid(sid)

    def on_ping(self):
        emit('pong')

    def on_connect(self):
        sid = request.sid
        print('Client connected', sid)

        with self.clients_lock:
            self.clients[sid] = pypilotClient()

    def on_disconnect(self):
        sid = request.sid
        print('Client disconnected', sid)

        with self.clients_lock:
            self._remove_sid(sid)

    # only call if you are holding the lock!
    def _remove_sid(self, sid):
        client = self.clients.pop(sid, None)
        if client:
            try:
                client.disconnect()
            except Exception as e:
                print('disconnect error', sid, e)

    def on_language(self, language):
        config['language'] = language
        write_config()

socketio.on_namespace(pypilotWeb(''))

def main():
    import os
    path = os.path.dirname(__file__)
    os.chdir(os.path.abspath(path))
    port = pypilot_web_port
    print('async mode', socketio.async_mode)
    while True:
        try:
            socketio.run(app, debug=False, host='0.0.0.0', port=port, allow_unsafe_werkzeug=True)
            break
        except PermissionError as e:
            print('failed to run socket io on port', port, e)
            port += 8000 - 80
            print('trying port', port)

if __name__ == '__main__':
    main()
