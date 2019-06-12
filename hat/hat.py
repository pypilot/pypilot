#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from __future__ import print_function
import time, os, sys
import json
from signalk.client import SignalKClient
import lcd, gpio, arduino, lirc, buzzer

class Action(object):
    def  __init__(self, hat, name):
        self.hat = hat
        self.name = name
        self.keys = []

class ActionKeypad(Action):
    def __init__(self, lcd, index, name):
        super(ActionKeypad, self).__init__(None, name)
        self.lcd = lcd
        self.index = index

    def trigger(self, down, count):
        self.lcd.keypadup[self.index] = not down
        self.lcd.keypad[self.index] = count
        
class ActionSignalK(Action):
    def  __init__(self, hat, name, signalk_name, signalk_value):
        super(ActionSignalK, self).__init__(lcd, name)
        self.signalk_name = signalk_name
        self.value = signalk_value

    def trigger(self):
        if self.hat.client:
            self.hat.client.set(self.signalk_name, self.value)

class ActionEngage(ActionSignalK):
    def  __init__(self, hat):
        super(ActionEngage, self).__init__(hat, 'engage', 'ap.enabled', True)

    def trigger(self):
        super(ActionEngage, self).trigger()
        # set heading to current heading
        if self.hat.client:
            self.hat.client.set('ap.heading_command', self.last_val('ap.heading'))
            
class ActionHeading(Action):
    def __init__(self, hat, offset):
        super(ActionHeading, self).__init__(hat, str(offset))
        self.offset = offset

    def trigger(self):
        if self.hat.client:
            if self.last_val('ap.enabled'):
                self.client.set('ap.heading_command',
                                self.last_val['ap.heading_command'] + self.offset)
            else: # manual mode
                self.servo_timeout = time.time() + abs(self.offset)**.5/2
                self.client.set('servo.command', 1 if self.offset > 0 else -1)

class ActionTack(ActionSignalK):
    def  __init__(self, hat, name, direction):
        super(ActionTack, self).__init__(hat, name, 'ap.tack.state', 'begin')
        self.direction = direction
                                
    def trigger(self):
        super(ActionEngage, self).trigger()
        if self.hat.client:
            self.hat.client.set('ap.tack.direction', self.direction)

def web_process(pipe, actions):
    import web
    web.web_process(pipe, actions)

class Web(object):
    def __init__(self, actions):
        import multiprocessing
        from signalk.pipeserver import NonBlockingPipe
        self.pipe, pipe = NonBlockingPipe('pipe', True)
        self.process = multiprocessing.Process(target=web_process, args=(pipe,actions))
        self.process.start()
        import atexit, signal
        def cleanup():
            os.kill(self.process.pid, signal.SIGTERM)

        atexit.register(cleanup) # get backtrace
            
class Hat(object):
    def __init__(self):
        # read config
        try:
            configfile = '/proc/device-tree/hat/custom_0'
            f = open(configfile)
            self.hatconfig = json.loads(f.read())

            f.close()
        except Exception as e:
            print('failed to load:', e)
            print('assuming original 26 pin tinypilot')
            self.hatconfig = False

        self.config = {'lcd': {}}
        self.configfilename = os.getenv('HOME') + '/.pypilot/hat.conf' 
        print('loading config file:', self.configfilename)
        try:
            file = open(self.configfilename)
            config = json.loads(file.read())
            file.close()
            for name in config:
                self.config[name] = config[name]
        except Exception as e:
            print('config failed:', e)

        self.lastpollheading = time.time()
        self.servo_timeout = time.time() + 1
        
        self.longsleep = 30
        self.last_msg = {}

        self.client = False
        self.lcd = lcd.LCD(self)
        self.gpio = gpio.gpio()
        self.arduino = arduino.arduino(self.config)
        self.lirc = lirc.lirc()
        self.buzzer = buzzer.buzzer(self.config)
        
        # keypad for lcd interface
        self.actions = []
        keypadnames = ['Auto', 'Menu', 'Up', 'Down', 'Select', 'Left', 'Right']
        
        for i in range(7):
            self.actions.append(ActionKeypad(self.lcd, i, keypadnames[i]))

        # stateless actions for autopilot control
        self.actions += [ActionEngage(self),
                         ActionSignalK(self, 'disengage', 'ap.enabled', False),
                         ActionHeading(self, 1),
                         ActionHeading(self, -1),
                         ActionHeading(self, 2),
                         ActionHeading(self, -2),
                         ActionHeading(self, 10),
                         ActionHeading(self, -10),
                         ActionSignalK(self, 'compassmode', 'ap.mode', 'compass'),
                         ActionSignalK(self, 'gpsmode', 'ap.mode', 'gps'),
                         ActionSignalK(self, 'windmode', 'ap.mode', 'wind'),
                         ActionTack(self, 'tackport', 'port'),
                         ActionTack(self, 'tackstarboard', 'starboard')]

        for action in self.actions:
            if action.name in self.config:
                action.keys = self.config[action.name]

        self.web = Web(self.actions)

    def write_config(self):
        config = {}
        for action in self.actions:
            config[action.name] = action.keys
                                
        try:
            file = open(config_path, 'w')
            file.write(json.dumps(config) + '\n')
        except IOError:
            print('failed to save config file:', self.configfilename)

    def connect(self):
        for name in ['gps.source', 'wind.source']:
            self.last_msg[name] = 'none'
        self.last_msg['ap.enabled'] = False
        self.last_msg['ap.heading_command'] = 0
        self.last_msg['imu.heading_offset'] = 0

        host = ''
        if len(sys.argv) > 1:
            host = sys.argv[1]
        
        def on_con(client):
            self.value_list = client.list_values(10)

            self.watchlist = ['ap.enabled', 'ap.heading_command'] + self.lcd.watchlist
            for name in self.watchlist:
                client.watch(name)

            for request in self.lcd.initial_gets:
                client.get(request)

        try:
            self.client = SignalKClient(on_con, host)
            if self.value_list:
                print('connected')
                self.web.pipe.send({'pypilot': 'Connected'})
            else:
                client.disconnect()
                print('no value list!')
                self.client = False
                self.web.pipe.send({'pypilot': 'Disconnected'})
        except Exception as e:
            print(e)
            self.client = False
            time.sleep(1)

    def apply_code(self, key, down, count):
        self.web.pipe.send({'key': key})

        for action in self.actions:
            if key in action.keys:
                self.web.pipe.send({'action': action.name})
                action.trigger(down, count)
            
    def poll(self):
        if not self.client:
            self.connect()

        while self.client:
            result = False
            try:
                result = self.client.receive_single()
            except Exception as e:
                print('disconnected', e)
                self.client = False

            if not result:
                break

            name, data = result

            if 'value' in data:
                self.last_msg[name] = data['value']

            for token in ['min', 'max', 'choices', 'AutopilotGain']:
                if token in data:
                    #print('name', name, token, ' = ', data[token])
                    if not name in self.value_list:
                        self.client.value_list[name] = {}
                    self.client.value_list[name][token] = data[token]

        for i in [self.lcd, self.buzzer]:
            i.poll()

        anycode = False # are any keys read?
        for i in [self.gpio, self.arduino, self.lirc]:
            i.poll()
            while True:
                r = i.read()
                if not r:
                    break
                key, down, count = r
                self.apply_code(*r)

                if not down:
                    self.longsleep = 0
                else:
                    self.longsleep += 1
                anycode = True

        if anycode == False:
            self.longsleep += 10

        while self.longsleep > 20:
            dt = self.lcd.frameperiod / 10.0
            time.sleep(dt)
            self.longsleep -= 1

        # poll heading once per second if not enabled
        t = time.time()
        dtp = t - self.lastpollheading
        if self.client and dtp > 1 and not self.last_msg['ap.enabled']:
            self.client.get('ap.heading')
            self.lastpollheading = t

        # timeout manual move
        if self.servo_timeout:
            dtt = t - self.servo_timeout
            if dtt > 0:
                if self.client:
                    self.client.set('servo.command', 0) # stop
                self.servo_timeout = 0

def main():
    hat = Hat()
    if hat.lcd.use_glut:
        from OpenGL.GLUT import glutMainLoop, glutIdleFunc
        glutIdleFunc(hat.poll)
        glutMainLoop()
    else:
        while True:
            hat.poll()

if __name__ == '__main__':
    main()
