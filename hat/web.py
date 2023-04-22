#!/usr/bin/env python
#
#   Copyright (C) 2023 Sean D'Epagnier
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

# strap pins for remotes give different codes depending on if the remote is intended to be used with the display or not
REMOTE = 0x5
PANEL =  0xF

# both types of remotes
STANDBY, COMPASS, PLUS1, MINUS1, GPS, PLUS10, MINUS10, WIND = list(map(lambda x : 2 ** x, range(8)))
AUTO, MENU, PLUS1, MINUS1, MODE, PLUS10, MINUS10            = list(map(lambda x : 2 ** x, range(7)))

# definitions for multiple keys pressed
TACK_PORT = MINUS10 | MINUS1 
TACK_STARBOARD = PLUS10 | PLUS1
NAV_MODE = COMPASS | GPS
TRUE_WIND_MODE = GPS | WIND

def lmap(*cargs):
    return list(map(*cargs))

def rf_code(pinc, pind):
    def wd_code(wd):
        code = ~pind, pind, (pinc ^ wd)
        code4 = code[0] & 0x7f, code[1] & 0x7f, code[2] & 0x7f, (code[0]&0x80)>>1 | (code[1]&0x80)>>2 | (code[2]&0x80)>>3
        return 'rf%02X%02X%02X%02X' % (code4[0], code4[1], code4[2], code4[3])
    return wd_code

def generate_codes(channel):
    pins = {'-10_': (PANEL, MINUS10), '-1_': (PANEL, MINUS1),
            '+10_': (PANEL, PLUS10), '+1_': (PANEL, PLUS1),
            'auto_': (PANEL, AUTO), 'menu_': (PANEL, MENU),
            'mode_': (PANEL, MODE),

            '-10': (REMOTE, MINUS10), '-1': (REMOTE, MINUS1),
            '+10': (REMOTE, PLUS10), '+1': (REMOTE, PLUS1),
            'standby': (REMOTE, STANDBY), 'compass mode': (REMOTE, COMPASS),
            'gps mode': (REMOTE, GPS), 'wind mode': (REMOTE, WIND),

            'tack port': [(REMOTE, TACK_PORT), (PANEL, TACK_PORT)],
            'tack starboard': [(REMOTE, TACK_STARBOARD), (PANEL, TACK_STARBOARD)],
            'nav mode': (REMOTE, NAV_MODE),
            'true wind mode': (REMOTE, TRUE_WIND_MODE)}
    codes = {}
    for name, pins in pins.items():        
        if type(pins) != type([]):
            pins = [pins]
        for (pinc, pind) in pins:
            pinc ^= channel<<2
            codes[name] = codes.get(name, []) + lmap(rf_code(pinc, pind), [0xac, 0xdc])
    return codes


all_code_channels = {}
for channel in range(8):
    for name, codes in generate_codes(channel).items():
        for code in codes:
            all_code_channels[code] = channel


default_actions = \
    {'-10_': ['ir03111800','ir03111000','KEY_LEFT'  ,'gpio06'],
     '-1_':  ['ir03201800','ir03201000','KEY_UP'    ,'gpio27'],
     '+1_':  ['ir03211800','ir03211000','KEY_DOWN'  ,'gpio22'],
     '+10_': ['ir03101800','ir03101000','KEY_RIGHT' ,'gpio05'],
     'auto_':['ir030C1000','ir030C1800','KEY_POWER' ,'gpio17'],
     'menu_':['ir030D1000','ir030D1800','KEY_MUTE'  ,'gpio23'],
     'mode_':['ir030B1000','ir030B1800','KEY_SELECT','gpio18'],

     'tack port':      ['gpio27_06'],
     'tack starboard': ['gpio22_05'],
    }

try:
    from flask_babel import Babel, gettext
    babel = Babel(app)

    LANGUAGES = os.listdir(os.path.dirname(os.path.abspath(__file__)) + '/translations')

    @babel.localeselector
    def get_locale():
        return request.accept_languages.best_match(LANGUAGES)
    
except Exception as e:
    print('failed to import flask_babel, translations not possible!!', e)
    def _(x): return x
    app.jinja_env.globals.update(_=_)
    babel = None

class WebConfig(Namespace):
    def __init__(self, name, pipe, config):
        super(Namespace, self).__init__(name)
        socketio.start_background_task(target=self.background_thread)
        self.pipe = pipe
        self.config = config
        self.status = 'N/A'
        self.profiles = False

        self.last_key = False

        ind = 0
        acts = ['', '']
        names = Markup('[')
        cols = 1
        col = 0
        acts[ind] += Markup('<table border=0>')
        i = 0
        actions = config['actions']
        for name in actions:
            if name.startswith('profile '):
                continue

            n = name.replace(' ', '_')
            n = n.replace('+', 'plus')
            if i == 7:
                acts[ind] += Markup('</tr></table>')
                ind = 1
                acts[ind] += Markup('<table border=0>')
                col = 0
            i+=1
    
            if col == 0:
                acts[ind] += Markup('<tr>')
            acts[ind] += Markup('<td><button id="action_' + n + '">' +
                                name + '</button></td><td><span id="action' +
                                n + 'keys"></span></td>')
            if col == cols-1:
                acts[ind] += Markup('</tr>')
                col = 0
            else:
                col += 1
            names += Markup('"' + n + '", ')

        acts[ind] += Markup('</table>')
        names += Markup('""]')

        adc_channels = Markup('<select id="adc_channels">')
        cadc = config.get('adc_channels', [])

        for i in range(4):
            adc_channels += Markup('<option value="' + str(i) + '"');
            if i == len(cadc):
                adc_channels += Markup(' selected')
            adc_channels += Markup('>' + str(i) + '</option>');
        adc_channels += Markup('</select>')
        for i in range(3):
            #adc_channels += Markup('<br>')
            adc_channels += Markup('<div id="adc_channel_' + str(i) + '">')
            adc_channels += Markup('Channel ' + str(i))
            adc_channels += Markup('<select id="adc_channel_' + str(i) + '_select">')
            options = ['none', 'steering', 'custom']
            for option in options:
                adc_channels += Markup('<option value="' + option + '"')
                if i < len(cadc) and cadc[i] == option:
                    adc_channels += Markup(' selected')
                adc_channels += Markup('>' + option + '</option>')
            adc_channels += Markup('</select>')
            adc_channels += Markup('</div>')
        
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

        remote = Markup('<input type="checkbox" id="remote"')
        if config['host'] != 'localhost':
            remote += Markup(' checked')
        remote += Markup(' /><input type="text" id="host" value="' + config['host'] + '" />')

        @app.route('/')
        def index():
            return render_template('index.html', async_mode=socketio.async_mode, web_port=web_port, actionkeys = acts, action_names = names, adc_channels = adc_channels, ir_settings = ir, nmea_settings = nmea, remote_settings = remote)

    def on_ping(self):
        emit('pong')

    def on_keys(self, command):
        actions = self.config['actions']
        if command == 'clear':
            for name in actions:
                actions[name] = []
            self.emit_keys()
            return

        if command == 'default':
            for name in actions:
                actions[name] = []

            for name, keys in default_actions.items():
                if not name in actions:
                    actions[name] = []
                actions[name] += keys.copy()

            for name, keys in generate_codes(0).items():
                if not name in actions:
                    actions[name] = []
                actions[name] += keys.copy()

            self.emit_keys()
            return

        if command.startswith('clearcodes'):
            command = command[10:]
            if command in actions:
                actions[command] = []
            self.emit_keys()
            return

        if not self.last_key:
            return

        # remove this key from any actions
        for name, keys in actions.items():
            while self.last_key in keys:
                keys.remove(self.last_key)

        # add the last key to the action
        if command != 'none':
            if not command in actions:
                actions[command] = []
            actions[command].append(self.last_key)
        self.emit_keys()

    def on_config(self, config):
        self.pipe.send(config)

    def emit_keys(self):
        actions = self.config['actions']
        for name, keys in actions.items():
            keys = {'name': name.replace(' ', '_'), 'keys': keys}
            socketio.emit('action_keys', keys)
        self.pipe.send({'actions': actions})

    def on_connect(self):
        if self.profiles:
            socketio.emit('profiles', self.profiles)
        self.emit_keys()

        print('web client connected', request.sid)

    def on_program_rf_codes(self, channel):
        print('program rf', channel)
        if not channel in range(8):
            return
        actions = self.config['actions']

        # remove any programming for any of these codes
        rf_codes = generate_codes(channel)
        for name, keys in rf_codes.items():
            for key in keys:
                for name, keys in actions.items():
                    while key in keys:
                        keys.remove(key)

        # add programming for this code
        for name, keys in rf_codes.items():
            if not name in actions:
                actions[name] = []
            actions[name] += keys
        self.emit_keys()


    def on_disconnect(self):
        print('web client disconnected', request.sid)

    def background_thread(self):
        print('web process on port', web_port)
        last_key_time = time.monotonic()
        x = 0
        polls_sent = {}
        while True:
            socketio.sleep(.5)

            if self.last_key:
                dtc = time.monotonic() - last_key_time
                if dtc > 8:
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

                if msg.get('action') == 'none': # if we have a known remote c
                    if self.last_key in all_code_channels:
                        channel = all_code_channels[self.last_key]
                        print('found rf codes', channel)
                        socketio.emit('found_rf_codes', channel)
                    
                for name in msg:
                    d = msg[name]
                    if name != 'profiles':
                        d = str(d)
                    socketio.emit(name, d)
                if 'status' in msg:
                    self.status = msg['status']
                    socketio.emit('status', self.status)
                if 'profiles' in msg:
                    self.profiles = msg['profiles']
                    self.emit_keys()

def web_process(pipe, config):
    print('web process', os.getpid())
    path = os.path.dirname(__file__)
    os.chdir(os.path.abspath(path))
    socketio.on_namespace(WebConfig('', pipe, config))
    socketio.run(app, debug=False, host='0.0.0.0', port=web_port)
    
if __name__ == '__main__':
    config = {'host': 'localhost', 'actions': {},
              'pi.ir': True, 'arduino.ir': False,
              'arduino.nmea.in': False, 'arduino.nmea.out': False,
              'arduino.nmea.baud': 4800,
              'lcd': {},
              'actions': default_actions.copy()}
    web_process(None, config)
