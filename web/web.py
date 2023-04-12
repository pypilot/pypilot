#!/usr/bin/env python
#
#   Copyright (C) 2021 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import sys, os
from flask import Flask, render_template, session, request, Markup

from flask_socketio import SocketIO, Namespace, emit, join_room, leave_room, \
    close_room, rooms, disconnect

from pypilot.client import pypilotClient
from pypilot import pyjson

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import tinypilot

config = {'port': 8000, 'language': 'default'}
configfilename = os.getenv('HOME')+'/.pypilot/web.conf'

try:
    file = open(configfilename, 'r')
    config.update(pyjson.loads(file.readline()))
    file.close()

except:
    print('failed to read config', configfilename)

def write_config():
    try:
        file = open(configfilename, 'w')
        file.write(pyjson.dumps(config) + '\n')
        file.close()
    except:
        print('failed to write config')

if len(sys.argv) > 1:
    pypilot_web_port=int(sys.argv[1])
else:
    pypilot_web_port = config['port']

print('using port', pypilot_web_port)
        

# Set this variable to 'threading', 'eventlet' or 'gevent' to test the
# different async modes, or leave it set to None for the application to choose
# the best option based on installed packages.
async_mode = None

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, async_mode=async_mode)

try:
    from flask_babel import Babel, gettext
    babel = Babel(app)

    LANGUAGES = os.listdir(os.path.dirname(os.path.abspath(__file__)) + '/translations')

    @babel.localeselector
    def get_locale():
        if 'language' in config:
            language = config['language']
        else:
            language = 'default'
        if language == 'default' or not language in LANGUAGES:
            return request.accept_languages.best_match(LANGUAGES)
        return language
    
except Exception as e:
    print('failed to import flask_babel, translations not possible!!', e)
    def _(x): return x
    app.jinja_env.globals.update(_=_)
    babel = None


@app.route('/logs')
def logs():
    log_links = ''
    try:
        logdirs = os.listdir('/var/log')
        for name in logdirs:
            if os.path.exists(os.path.join('/var/log', name, 'current')):
                log_links+='<br><a href="log/'+name+'">'+name+'</a>';
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
    networking = '/home/tc/.pypilot/networking.txt'
    wifi = {'mode': 'Master', 'ssid': 'pypilot', 'key': '',
            'client_ssid': 'openplotter', 'client_key': '12345678', 'client_address': '10.10.10.60'}
    try:
        f = open(networking, 'r')
        while True:
            l = f.readline()
            if not l:
                break
            try:
                name, value = l.split('=')
                wifi[name] = value.rstrip()
            except Exception as e:
                print('failed to parse line in networking.txt', l)
        f.close()
    except:
        pass

    if request.method == 'POST':
        try:
            for name in request.form:
                wifi[name] = str(request.form[name])

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
                
                from datetime import datetime
                ts = int(elements[0])
                if ts:
                    ts = datetime.utcfromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
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
    if not 'Master' in wifi['mode']:
        leases = ''

    return render_template('wifi.html', async_mode=socketio.async_mode, wifi=Markup(wifi), leases=Markup(leases))

@app.route('/calibrationplot')
def calibrationplot():
    return render_template('calibrationplot.html', async_mode=socketio.async_mode,pypilot_web_port=pypilot_web_port)

@app.route('/client')
def client():
    return render_template('client.html', async_mode=socketio.async_mode,pypilot_web_port=pypilot_web_port)

translations = []
static = False
with open(os.path.dirname(os.path.abspath(__file__)) + '/pypilot_web.pot') as f:
    for line in f:
        if line.startswith('#: static'):
            static = True
        elif len(line.strip()) == 0:
            static = False
        elif static and line.startswith('msgid'):
            s = line[7:-2]
            if s:
                translations.append(s)

@app.route('/')
def index():
    return render_template('index.html', async_mode=socketio.async_mode, pypilot_web_port=pypilot_web_port, tinypilot=tinypilot.tinypilot, translations=translations, language=config['language'], languages=Markup(LANGUAGES))

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
            sids = list(self.clients)
            for sid in sids:
                if not sid in self.clients:
                    print('client was removed')
                    continue # was removed

                client = self.clients[sid]
                values = client.list_values()
                if values:
                    socketio.emit('pypilot_values', pyjson.dumps(values), room=sid)
                if not client.connection:
                    socketio.emit('pypilot_disconnect', room=sid)
                msgs = client.receive()
                if msgs:
                    # convert back to json (format is nicer)
                    #print('msgs', msgs, sid)
                    socketio.emit('pypilot', pyjson.dumps(msgs), room=sid)

    def on_pypilot(self, message):
        #print('message', message)
        self.clients[request.sid].send(message + '\n')

    def on_ping(self):
        emit('pong')

    def on_connect(self):
        print('Client connected', request.sid)
        client = pypilotClient()
        self.clients[request.sid] = client

    def on_disconnect(self):
        print('Client disconnected', request.sid)
        client = self.clients[request.sid]
        client.disconnect()
        del self.clients[request.sid]

    def on_language(self, language):
        config['language'] = language
        write_config()

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
            port += 8000 - 80
            print('trying port', port)

if __name__ == '__main__':
    main()
