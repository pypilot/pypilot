#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time, os, sys
import json
from pypilot.client import pypilotClient

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import lcd, gpio, arduino, lircd, buzzer

class Action(object):
    def  __init__(self, hat, name):
        self.hat = hat
        self.name = name
        self.keys = []

class ActionNone(Action):
    def __init__(self):
        super(ActionNone, self).__init__(None, 'none')

    def trigger(self, count):
        pass
        
class ActionKeypad(Action):
    def __init__(self, lcd, index, name):
        super(ActionKeypad, self).__init__(None, name)
        self.lcd = lcd
        self.index = index

    def trigger(self, count):
        self.lcd.keypadup[self.index] = not count
        self.lcd.keypad[self.index] = count
        
class ActionPypilot(Action):
    def  __init__(self, hat, name, pypilot_name, pypilot_value):
        super(ActionPypilot, self).__init__(hat, name)
        self.pypilot_name = pypilot_name
        self.value = pypilot_value

    def trigger(self, count):
        if self.hat.client and not count:
            self.hat.client.set(self.pypilot_name, self.value)

class ActionEngage(ActionPypilot):
    def  __init__(self, hat):
        super(ActionEngage, self).__init__(hat, 'engage', 'ap.enabled', True)

    def trigger(self, count):
        super(ActionEngage, self).trigger(count)
        # set heading to current heading
        if self.hat.client and not count and 'ap.heading' in self.hat.last_msg:
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
            self.servo_timeout = time.monotonic() + abs(self.offset)**.5/2
            self.hat.client.set('servo.command', 1 if self.offset > 0 else -1)
            
class ActionTack(ActionPypilot):
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
            print('cleanup web process', signal_number)
            if signal_number == signal.SIGCHLD:
                pid = os.waitpid(-1, 0)
                if not self.process or pid != self.process.pid:
                    # flask makes process at startup that dies
                    return
            if self.process:
                print('pid kill ', self.process.pid)
                os.kill(self.process.pid, signal.SIGTERM) # get backtrace
                self.process = False
            sys.stdout.flush()
            if signal_number:
                raise KeyboardInterrupt # to get backtrace on all processes

        atexit.register(lambda : cleanup(None)) # get backtrace
        for s in range(1, 16):
            if s != 9 and s != 13:
                signal.signal(s, cleanup)
        signal.signal(signal.SIGCHLD, cleanup)
        import atexit
        atexit.register(lambda : cleanup('atexit'))

    def send(self, value):
        if self.process:
            self.pipe.send(value)

    def set_status(self, value):
        if self.status == value:
            return
        print('status', value)
        self.status = value
        self.send({'status': value})

    def poll(self):
        if not self.process:
            import multiprocessing
            from pypilot.nonblockingpipe import NonBlockingPipe
            self.pipe, pipe = NonBlockingPipe('webpipe', True)
            keyspipe, self.keyspipe = NonBlockingPipe('webkeyspipe', True)
            self.process = multiprocessing.Process(target=web_process, args=(pipe, keyspipe, self.hat.actions), daemon=True)
            self.process.start()
            self.send({'status': self.status})

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

        self.servo_timeout = time.monotonic() + 1
        
        self.last_msg = {}

        self.client = False
        self.connect()
        self.lcd = lcd.LCD(self)
        self.gpio = gpio.gpio()
        self.arduino = arduino.arduino(self.hatconfig)
        self.lirc = lircd.lirc()
        self.buzzer = buzzer.buzzer(self.hatconfig)
        
        # keypad for lcd interface
        self.actions = []
        keypadnames = ['auto', 'menu', 'up', 'down', 'select', 'left', 'right']
        
        for i in range(7):
            self.actions.append(ActionKeypad(self.lcd, i, keypadnames[i]))

        # stateless actions for autopilot control
        self.actions += [ActionEngage(self),
                         ActionPypilot(self, 'disengage', 'ap.enabled', False),
                         ActionHeading(self, 1),
                         ActionHeading(self, -1),
                         ActionHeading(self, 2),
                         ActionHeading(self, -2),
                         ActionHeading(self, 10),
                         ActionHeading(self, -10),
                         ActionPypilot(self, 'compassmode', 'ap.mode', 'compass'),
                         ActionPypilot(self, 'gpsmode', 'ap.mode', 'gps'),
                         ActionPypilot(self, 'windmode', 'ap.mode', 'wind'),
                         ActionTack(self, 'tackport', 'port'),
                         ActionTack(self, 'tackstarboard', 'starboard'),
                         ActionNone()]

        for action in self.actions:
            if action.name in self.config['actions']:
                action.keys = self.config['actions'][action.name]

        self.web = Web(self)

    def connect(self):
        if self.client:
            self.client.disconnect()

        self.last_msg['ap.enabled'] = False
        self.last_msg['ap.heading_command'] = 0

        if self.config['remote']:
            host = self.config['host']
        else:
            host = 'localhost'

        self.client = pypilotClient(host)
        self.watchlist = ['ap.enabled', 'ap.heading_command']
        for name in self.watchlist:
            self.client.watch(name)

    def write_config(self):
        actions = {}
        for action in self.actions:
            if action.name != 'none':
                actions[action.name] = action.keys

        self.config['actions'] = actions
                                
        try:
            file = open(self.configfilename, 'w')
            file.write(json.dumps(self.config) + '\n')
        except IOError:
            print('failed to save config file:', self.configfilename)

    def apply_code(self, key, count):
        if count == 1:
            self.web.send({'key': key})
        for action in self.actions:
            if key in action.keys:
                if not count:
                    self.web.send({'action': action.name})
                action.trigger(count)
                return
        self.web.send({'action': 'none'})
                
    def poll(self):
        if self.client.connection:
            self.web.set_status('connected')
        else:
            self.web.set_status('disconnected')
            
        msgs = self.client.receive()
        for name, value in msgs.items():
            self.last_msg[name] = value

        for i in [self.lcd, self.buzzer, self.web]:
            i.poll()

        for i in [self.gpio, self.arduino, self.lirc]:
            i.poll()
            while True:
                if not i.events:
                    break

                r = i.events[0]
                i.events = i.events[1:]
                if i == self.arduino:
                    if r[0].startswith('ir'):
                        lircd.LIRC_version = 0 # disable lircd if we got ir from arduino
                    
                self.apply_code(*r)

        time.sleep(.1)

        # receive heading once per second if autopilot is not enabled
        self.client.watch('ap.heading', False if self.last_msg['ap.enabled'] else 1)

        # timeout manual move
        if self.servo_timeout:
            if time.monotonic() > self.servo_timeout:
                if self.client:
                    self.client.set('servo.command', 0) # stop
                self.servo_timeout = 0

def main():
    hat = Hat()
    if hat.lcd and hat.lcd.use_glut:
        from OpenGL.GLUT import glutMainLoop, glutIdleFunc
        glutIdleFunc(hat.poll)
        glutMainLoop()
    else:
        while True:
            hat.poll()

if __name__ == '__main__':
    main()
