#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time, os, sys, signal
import json
from pypilot.client import pypilotClient
import arduino, web

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import lcd, gpio, lircd

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
        self.lcd.keypad[self.index].update(count, count)
        
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
        if self.hat.client:
            self.hat.client.set('ap.tack.direction', self.direction)
        super(ActionTack, self).trigger()

class Process():
    def __init__(self, hat):
        self.process = False
        self.hat = hat
    
    def send(self, value):
        if self.process:
            self.pipe.send(value)

    def create(self, process, arg):
        def cleanup(signal_number, frame=None):
            print('cleanup web process', signal_number)
            if signal_number == signal.SIGCHLD:
                pid = os.waitpid(-1, 0)
                if not self.process or pid[0] != self.process.pid:
                    print('proce', self.process, pid, self.process.pid)
                    # flask makes process at startup that dies
                    return
            if self.process:
                try:
                    os.kill(self.process.pid, signal.SIGTERM) # get backtrace
                except Exception as e:
                    if e.args[0] != 3: # no such process, already died
                        raise e                        
                self.process = False
            sys.stdout.flush()
            if signal_number:
                exit(0)

        for s in range(3, 16):
            if s != 9 and s != 13:
                signal.signal(s, cleanup)
        signal.signal(signal.SIGCHLD, cleanup)

        import multiprocessing
        from pypilot.nonblockingpipe import NonBlockingPipe
        self.pipe, pipe = NonBlockingPipe('webpipe', True)
        self.process = multiprocessing.Process(target=process, args=(pipe, arg), daemon=True)
        self.process.start()

    def poll_ready(self):
        if not self.process:
            self.create()            

        if not self.process.is_alive():
            self.process = False
            return False
        return True

    def poll(self):
        if not self.poll_ready():
            return
        return self.pipe.recv()
            
class Web(Process):
    def __init__(self, hat):
        super(Web, self).__init__(hat)
        self.status = 'Not Connected'

    def set_status(self, value):
        if self.status == value:
            return
        print('status', value)
        self.status = value
        self.send({'status': value})

    def create(self):
        def web_process(pipe, actions):
            while True:
                try:
                    web.web_process(pipe, actions)
                except Exception as e:
                    print('failed to run web server:', e)
                    while True:
                time.sleep(5)
        super(self, Web).create(web_process, self.hat.actions)
        self.send({'status': self.status})
        
    def poll(self):
        if not self.poll_ready():
            return
        
        msg = self.pipe.recv()
        if msg:
            for name in msg:
                for action in self.hat.actions:
                    if name == action.name:
                        action.keys = msg[name]
                else:
                    if name == 'baud':
                        self.hat.arduino.set_baud(msg[name])

            self.hat.write_config()


class Arduino(Process)
    def __init__(self, hat):
        self.process = False
        self.status = 'Not Connected'
        self.hat = hat

    def set_baud(self, baud):
        self.send(('baud', baud))

    def set_buzzer(self, duration, frequency):
        self.send(('buzzer', (duration, frequency)))

    def create(self):
        def web_process(pipe, config):
            while True:
                try:
                    arduino.arduino_process(pipe, config)
                except Exception as e:
                    print('failed to run web server:', e)
                    while True:
                time.sleep(5)
        super(self, Arduino).create(arduino_process, config)
            
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

        self.config = {'remote': False, 'host': 'pypilot', 'actions': {}, 'lcd': {}, 'arduino': {}}
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
        
        self.lcd = lcd.LCD(self)
        self.gpio = gpio.gpio()
        self.arduino = arduino.arduino(self.config, self.hatconfig, host)
        self.lirc = lircd.lirc()
        
        # keypad for lcd interface
        self.actions = []
        keypadnames = ['auto', 'menu', 'port1', 'starboard1', 'select', 'port10', 'starboard10', 'tack', 'nudge_port', 'nudge_starboard']
        
        for i in range(len(keypadnames)):
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
        if key == 'baudrate': # statistics
            print('baudrate', count)
            self.web.send({'baudrate': str(count)})
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

        t0 = time.time()
        for i in [self.lcd, self.web]:
            i.poll()
            #print('dt', time.time()-t0)

        for i in [self.gpio, self.arduino, self.lirc]:
            events = i.poll()
            for event in events:
                print('event', event)
                self.apply_code(*event)

        time.sleep(.01)

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
