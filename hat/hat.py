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
        self.lcd.lastframetime = 0 # trigger immediate refresh
        
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
        self.hat = hat
        self.create()            
    
    def send(self, value):
        if self.process:
            self.pipe.send(value)

    def create(self, process, arg):
        import multiprocessing
        from pypilot.nonblockingpipe import NonBlockingPipe
        self.pipe, pipe = NonBlockingPipe(str(self), True)
        self.process = multiprocessing.Process(target=process, args=(pipe, arg), daemon=True)
        self.process.start()
            
class Web(Process):
    def __init__(self, hat):
        self.status = 'Not Connected'
        super(Web, self).__init__(hat)

    def set_status(self, value):
        if self.status == value:
            return
        print('status', value)
        self.status = value
        self.send({'status': value})

    def create(self):
        def process(pipe, action_keys):
            print('web process on ', os.getpid())
            try:
                import web
                web.web_process(pipe, action_keys)
            except Exception as e:
                print('failed to run process:', e)

        action_keys = {}
        for action in self.hat.actions:
            action_keys[action.name] = action.keys
        super(Web, self).create(process, action_keys)
        self.send({'status': self.status})
        
    def poll(self):
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

class Arduino(Process):
    def __init__(self, hat):
        super(Arduino, self).__init__(hat)
        self.status = 'Not Connected'

    def set_baud(self, baud):
        self.send(('baud', baud))

    def set_backlight(self, value, polarity):
        self.send(('backlight', (value, polarity)))

    def set_buzzer(self, duration, frequency):
        self.send(('buzzer', (duration, frequency)))

    def create(self):
        def process(pipe, config):
            import arduino
            print('arduino process on ', os.getpid())
            while True:
                arduino.arduino_process(pipe, config)
                time.sleep(5)
        super(Arduino, self).create(process, self.hat.config)

    def poll(self):
        msgs = []
        while True:
            msg = self.pipe.recv()
            if not msg:
                break
            msgs += msg
        return msgs
            
class Hat(object):
    def __init__(self):
        # read config
        self.config = {'remote': False, 'host': '127.0.0.1', 'actions': {}, 'lcd': {}, 'arduino': {}}
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

        try:
            configfile = '/proc/device-tree/hat/custom_0'
            f = open(configfile)
            hatconfig = json.loads(f.read())
            f.close()
            print('loaded device tree hat config')
        except Exception as e:
            print('failed to load', configfile, ':', e)
            hatconfig = {'lcd':{'driver':'nokia5110',
                                'port':'/dev/spidev0.0'},
                         'lirc':'gpio4'}
            if True: # for test
                hatconfig['lcd']['driver'] = 'jlx12864'
                hatconfig['arduino'] = {'device':'/dev/spidev0.1',
                                        'resetpin':16,
                                        'hardware':0.21}
            print('assuming original 26 pin tinypilot with nokia5110 display')
        self.config['hat'] = hatconfig

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
        self.arduino = Arduino(self)

        # use raspberry pi lirc if there is no arduino
        if not 'arduino' in hatconfig and 'lirc' in hatconfig:
            self.lirc = lircd.lirc()
        else:
            self.lirc = False
        self.keytimes = {}
        
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

        def cleanup(signal_number, frame=None):
            print('got signal', signal_number, 'cleaning up', os.getpid())
            childpids = []
            processes = [self.arduino, self.web]
            for process in processes:
                if process.process:
                    childpids.append(process.process.pid)
            if signal_number == signal.SIGCHLD:
                pid = os.waitpid(-1, 0)
                if not pid[0] in childpids:
                    print('flask ret', pid, childpids)
                    # flask makes process at startup that dies
                    return
                print('child process', pid, childpids)
            while childpids:
                pid = childpids.pop()
                #print('kill!', pid, childpids, os.getpid())
                try:
                    os.kill(pid, signal.SIGTERM) # get backtrace
                except ProcessLookupError:
                    pass # ok, process is already terminated
                sys.stdout.flush()
            for process in processes:
                process.process = False
            if signal_number != 'atexit':
                raise KeyboardInterrupt # to get backtrace on all processes
            sys.stdout.flush()

        for s in range(1, 16):
            if s != 9 and s != 13:
                signal.signal(s, cleanup)
        signal.signal(signal.SIGCHLD, cleanup)
        import atexit
        atexit.register(lambda : cleanup('atexit'))

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
            self.web.send({'baudrate': count})
            return
        self.web.send({'key': key})
        for action in self.actions:
            if key in action.keys:
                if not count:
                    self.web.send({'action': action.name})
                    if key in self.keytimes:
                        del self.keytimes[key]
                else:
                    self.keytimes[key] = time.monotonic()
                action.trigger(count)
                return
        self.web.send({'action': 'none'})
                
    def poll(self):
        if self.client.connection:
            self.web.set_status('connected')
        else:
            self.web.set_status('disconnected')
            
        t0 = time.monotonic()
        msgs = self.client.receive()
        t1 = time.monotonic()
        for name, value in msgs.items():
            self.last_msg[name] = value

        for i in [self.lcd, self.web]:
            i.poll()
        t2 = time.monotonic()
        for i in [self.gpio, self.arduino, self.lirc]:
            try:
                if not i:
                    continue
                events = i.poll()
                for event in events:
                    self.apply_code(*event)
            except Exception as e:
                print('WARNING, failed to poll!!', e)
        t3 = time.monotonic()
        for key, t in self.keytimes.items():
            dt = time.monotonic() - t
            if dt > .6:
                print('keyup event lost, releasing key from timeout', key, dt)
                self.apply_code(key, 0)
                break

        # receive heading once per second if autopilot is not enabled
        self.client.watch('ap.heading', False if self.last_msg['ap.enabled'] else 1)

        # timeout manual move
        if self.servo_timeout:
            if time.monotonic() > self.servo_timeout:
                if self.client:
                    self.client.set('servo.command', 0) # stop
                self.servo_timeout = 0
        t4 = time.monotonic()
        dt = t3-t0
        period = max(.2 - dt, .01)
        time.sleep(period)
        #print('times', t1-t0, t2-t1, t3-t2, t4-t3, period, dt)

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
