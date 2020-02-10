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

    def trigger(self, count):
        self.lcd.keypadup[self.index] = not count
        self.lcd.keypad[self.index] = count
        
class ActionSignalK(Action):
    def  __init__(self, hat, name, signalk_name, signalk_value):
        super(ActionSignalK, self).__init__(hat, name)
        self.signalk_name = signalk_name
        self.value = signalk_value

    def trigger(self, count):
        if self.hat.client and not count:
            self.hat.client.set(self.signalk_name, self.value)

class ActionEngage(ActionSignalK):
    def  __init__(self, hat):
        super(ActionEngage, self).__init__(hat, 'engage', 'ap.enabled', True)

    def trigger(self, count):
        super(ActionEngage, self).trigger(count)
        # set heading to current heading
        if self.hat.client and not count:
            self.hat.client.set('ap.heading_command', self.hat.last_msg['ap.heading'])
            
class ActionHeading(Action):
    def __init__(self, hat, offset):
        super(ActionHeading, self).__init__(hat, str(offset))
        self.offset = offset

    def trigger(self, count):
        if not self.hat.client:
            return
        if self.hat.last_msg['ap.enabled']:
            if not count:
                self.hat.client.set('ap.heading_command',
                                    self.hat.last_msg['ap.heading_command'] + self.offset)
        else: # manual mode
            self.servo_timeout = time.time() + abs(self.offset)**.5/2
            self.hat.client.set('servo.command', 1 if self.offset > 0 else -1)
            
class ActionTack(ActionSignalK):
    def  __init__(self, hat, name, direction):
        super(ActionTack, self).__init__(hat, name, 'ap.tack.state', 'begin')
        self.direction = direction
                                
    def trigger(self):
        super(ActionEngage, self).trigger()
        if self.hat.client:
            self.hat.client.set('ap.tack.direction', self.direction)

def web_process(pipe, keyspipe, actions):
    while True:
        try:
            import web
            web.web_process(pipe, keyspipe, actions)
        except Exception as e:
            print('failed to run web server:', e)
            #time.sleep(5)
            exit(0)            

class Web(object):
    def __init__(self, hat):
        self.process = False
        self.status = 'Not Connected'
        self.hat = hat
        import atexit, signal
        def cleanup(signal_number, frame=None):
            print('cleanup web process')
            if self.process:
                self.process.terminate()
            sys.stdout.flush()
            if signal_number:
                raise KeyboardInterrupt # to get backtrace on all processes
                

        atexit.register(lambda : cleanup(None)) # get backtrace
        for s in range(1, 12):
            if s != 9:
                signal.signal(s, cleanup)

    def send(self, value):
        if self.process:
            self.pipe.send(value)

    def set_status(self, value):
        self.status = value
        self.send({'status': value})

    def poll(self):
        if not self.process:
            import multiprocessing
            from signalk.pipeserver import NonBlockingPipe
            self.pipe, pipe = NonBlockingPipe('webpipe', True)
            keyspipe, self.keyspipe = NonBlockingPipe('webkeyspipe', True)
            self.process = multiprocessing.Process(target=web_process, args=(pipe, keyspipe, self.hat.actions))
            self.process.start()
            self.send({'status': self.status})
            print('web on', self.process.pid)

        if not self.process.is_alive():
            self.process = False
            return

        msg = self.keyspipe.recv()
        if msg:
            for name in msg:
                for action in self.hat.actions:
                    if name == action.name:
                        action.keys = msg[name]

            self.hat.write_config()

class Hat(object):
    def __init__(self):
        # read config
        try:
            configfile = '/proc/device-tree/hat/custom_0'
            f = open(configfile)
            self.hatconfig = json.loads(f.read())

            f.close()
        except Exception as e:
            print('failed to load', configfile, ':', e)
            print('assuming original 26 pin tinypilot')
            self.hatconfig = False

        self.config = {'remote': False, 'host': 'pypilot', 'actions': {}, 'lcd': {}}
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
        self.arduino = arduino.arduino(self.hatconfig)
        self.lirc = lirc.lirc()
        self.buzzer = buzzer.buzzer(self.hatconfig)
        
        # keypad for lcd interface
        self.actions = []
        keypadnames = ['auto', 'menu', 'up', 'down', 'select', 'left', 'right']
        
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
            if action.name in self.config['actions']:
                action.keys = self.config['actions'][action.name]

        self.web = Web(self)

    def write_config(self):
        actions = {}
        for action in self.actions:
            actions[action.name] = action.keys

        self.config['actions'] = actions
                                
        try:
            file = open(self.configfilename, 'w')
            file.write(json.dumps(self.config) + '\n')
        except IOError:
            print('failed to save config file:', self.configfilename)

    def connect(self):
        for name in ['gps.source', 'wind.source']:
            self.last_msg[name] = 'none'
        self.last_msg['ap.enabled'] = False
        self.last_msg['ap.heading_command'] = 0
        self.last_msg['imu.heading_offset'] = 0

        if self.config['remote']:
            host = self.config['host']
        else:
            host = 'localhost'
        
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
                self.web.set_status('connected')
            else:
                self.client.disconnect()
                print('no value list!')
                self.client = False
                self.web.set_status('disconnected')
        except Exception as e:
            print('hat exception', e)
            self.client = False
            time.sleep(1)

    def apply_code(self, key, count):
        if count:
            self.longsleep = 0

        if count == 1:
            self.web.send({'key': key})
        for action in self.actions:
            if key in action.keys:
                if not count:
                    self.web.send({'action': action.name})
                action.trigger(count)
            
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
                    # print('name', name, token, ' = ', data[token])
                    if not name in self.value_list:
                        self.client.value_list[name] = {}
                    self.client.value_list[name][token] = data[token]

        for i in [self.lcd, self.buzzer, self.web]:
            i.poll()

        for i in [self.gpio, self.arduino, self.lirc]:
            i.poll()
            while True:
                if not i.events:
                    break

                r = i.events[0]
                i.events = i.events[1:]
                self.apply_code(*r)

        time.sleep(.1)
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
