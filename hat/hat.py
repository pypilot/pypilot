#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time, os, sys, signal, select
from pypilot import pyjson
from pypilot.client import pypilotClient
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import lcd, gpio, lircd

class Action(object):
    def  __init__(self, hat, name):
        self.hat = hat
        self.name = name

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

    def create(self, process):
        import multiprocessing
        from pypilot.nonblockingpipe import NonBlockingPipe
        self.pipe, pipe = NonBlockingPipe(str(self), True)
        self.process = multiprocessing.Process(target=process, args=(pipe, self.hat.config), daemon=True)
        self.process.start()
            
class Web(Process):
    def __init__(self, hat):
        self.status = 'Not Connected'
        super(Web, self).__init__(hat)

    def set_status(self, value):
        if self.status == value:
            return
        self.status = value
        self.send({'status': value})

    def create(self):
        def process(pipe, config):
            while True:
                if os.system('sudo chrt -pi 0 %d 2> /dev/null > /dev/null' % os.getpid()):
                    print('warning, failed to make hat web process idle, trying renice')
                if os.system("renice 20 %d" % os.getpid()):
                    print('warning, failed to renice hat web process')
                if os.getenv('USER') == 'tc':
                    time.sleep(30) # delay loading web and wait until modules are loaded
                try:
                    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
                    import web
                    web.web_process(pipe, config)
                except Exception as e:
                    print('web failed to run process:', e)

        super(Web, self).create(process)
        self.send({'status': self.status})
        
    def poll(self):
        msg = self.pipe.recv()
        if msg:
            for name in msg:
                value = msg[name]
                self.hat.update_config(name, value);
            self.hat.write_config()

class Arduino(Process):
    def __init__(self, hat):
        super(Arduino, self).__init__(hat)
        self.voltage = {'vcc': 5, 'vin': 3.3}
        self.status = 'Not Connected'
        self.backlight = self.hat.config['lcd']['backlight']

    def config(self, name, value):
        self.send((name, value))

    def set_buzzer(self, duration, frequency):
        self.send(('buzzer', (duration, frequency)))

    def create(self):
        def process(pipe, config):
            import arduino
            print('arduino process on ', os.getpid())
            if os.system("renice -5 %d" % os.getpid()):
                print('warning, failed to renice hat arduino process')
            while True:
                arduino.arduino_process(pipe, config)
                time.sleep(15)
        super(Arduino, self).create(process)

    def poll(self):
        ret = []
        backlight = self.hat.config['lcd']['backlight']
        if backlight != self.backlight:
            self.backlight = backlight
            self.send(('backlight', backlight))
        while True:
            msgs = self.pipe.recv()
            if not msgs:
                break
            for msg in msgs:
                key, code = msg
                if key == 'baudrate': # statistics
                    self.hat.web.send({'baudrate': code})
                elif key == 'voltage': # statistics
                    self.hat.web.send({'voltage': '5v = %.3f, 3.3v = %.3f' % (code['vcc'], code['vin'])})
                    self.voltage = code
                else:
                    ret.append(msg)
        return ret
            
class Hat(object):
    def __init__(self):
        # read config
        self.config = {'remote': False, 'host': '127.0.0.1', 'actions': {},
                       'pi.ir': True, 'arduino.ir': False,
                       'arduino.nmea.in': False, 'arduino.nmea.out': False,
                       'arduino.nmea.baud': 4800,
                       'lcd': {}}
        self.configfilename = os.getenv('HOME') + '/.pypilot/hat.conf' 
        print('loading config file:', self.configfilename)
        try:
            file = open(self.configfilename)
            config = pyjson.loads(file.read())
            file.close()
            for name in config:
                self.config[name] = config[name]
        except Exception as e:
            print('config failed:', e)

        try:
            configfile = '/proc/device-tree/hat/custom_0'
            f = open(configfile)
            hat_config = pyjson.loads(f.read())
            f.close()
            print('loaded device tree hat config')
            if not 'hat' in self.config or hat_config != self.config['hat']:
                self.config['hat'] = hat_config
                print('writing device tree hat to hat.conf')
                self.write_config()
        except Exception as e:
            print('failed to load', configfile, ':', e)
            
        if not 'hat' in self.config:
            if os.path.exists('/dev/spidev0.0'):
                print('assuming original 26 pin tinypilot with nokia5110 display')
                self.config['hat'] = {'lcd':{'driver':'nokia5110',
                                            'port':'/dev/spidev0.0'},
                                    'lirc':'gpio4'}
            self.write_config()

        self.servo_timeout = time.monotonic() + 1
        
        self.last_msg = {}
        self.last_msg['ap.enabled'] = False
        self.last_msg['ap.heading_command'] = 0

        if self.config['remote']:
            host = self.config['host']
        else:
            host = 'localhost'
        self.client = pypilotClient(host)
        self.client.registered = False
        self.watchlist = ['ap.enabled', 'ap.heading_command']
        for name in self.watchlist:
            self.client.watch(name)
        self.lcd = lcd.LCD(self)
        self.gpio = gpio.gpio()
        self.arduino = Arduino(self)

        self.poller = select.poll()
        self.poller.register(self.arduino.pipe, select.POLLIN)

        self.lirc = lircd.lirc(self.config)
        self.lirc.registered = False
        self.keytimes = {}
        self.keytimeouts = {}

        # keypad for lcd interface
        self.actions = []
        keypadnames = ['auto', 'menu', 'port1', 'starboard1', 'select', 'port10', 'starboard10', 'tack', 'dodge_port', 'dodge_starboard']
        
        for i in range(len(keypadnames)):
            self.actions.append(ActionKeypad(self.lcd, i, keypadnames[i]))

        # stateless actions for autopilot control
        self.actions += [ActionEngage(self),
                         ActionPypilot(self, 'disengage', 'ap.enabled', False),
                         ActionHeading(self,  1),
                         ActionHeading(self, -1),
                         ActionHeading(self,  2),
                         ActionHeading(self, -2),
                         ActionHeading(self,  5),
                         ActionHeading(self, -5),
                         ActionHeading(self,  10),
                         ActionHeading(self, -10),
                         ActionPypilot(self, 'compassmode', 'ap.mode', 'compass'),
                         ActionPypilot(self, 'gpsmode', 'ap.mode', 'gps'),
                         ActionPypilot(self, 'windmode', 'ap.mode', 'wind'),
                         ActionPypilot(self, 'center', 'servo.position', 0),
                         ActionTack(self, 'tackport', 'port'),
                         ActionTack(self, 'tackstarboard', 'starboard'),
                         ActionNone()]

        for action in self.actions:
            if not action.name in self.config['actions']:
                self.config['actions'][action.name] = []

        self.web = Web(self)

        def cleanup(signal_number, frame=None):
            print('got signal', signal_number, 'cleaning up', os.getpid())
            childpids = []
            processes = [self.arduino, self.web]
            for process in processes:
                if process.process:
                    childpids.append(process.process.pid)
            if signal_number == signal.SIGCHLD:
                pid = os.waitpid(-1, os.WNOHANG)
                if not pid[0] in childpids:
                    print('subprocess returned', pid, childpids)
                    # flask or system makes process at startup that dies
                    return
                print('child process', pid, childpids)
            while childpids:
                pid = childpids.pop()
                #print('kill!', pid, childpids, os.getpid())
                try:
                    os.kill(pid, signal.SIGTERM) # get backtrace
                except ProcessLookupError:
                    pass # ok, process is already terminated
                #os.waitpid(pid, 0)
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
        try:
            file = open(self.configfilename, 'w')
            file.write(pyjson.dumps(self.config) + '\n')
        except IOError:
            print('failed to save config file:', self.configfilename)

    def update_config(self, name, value):
        if name in self.config and self.config[name] == value:
            return
        
        if name.startswith('arduino.') and self.arduino:
            self.arduino.config(name, value)

        self.config[name] = value
            
    def apply_code(self, key, count):
        if key in self.keytimeouts:
            timeoutcount = self.keytimeouts[key]
            if count > timeoutcount:
                return # ignore as we already timed out from this key
            del self.keytimeouts[key]
            if count == 0:
                return # already applied count 0
        self.web.send({'key': key})
        actions = self.config['actions']
        for action in self.actions:
            if not action.name in actions:
                actions[action.name] = []
            keys = actions[action.name]
            if key in keys:
                if not count:
                    self.web.send({'action': action.name})
                    if key in self.keytimes:
                        del self.keytimes[key]
                else:
                    self.keytimes[key] = time.monotonic(), count
                action.trigger(count)
                return
        self.web.send({'action': 'none'})

    def check_voltage(self):
        if not self.arduino:
            return False

        vin, vcc = self.arduino.voltage['vin'], self.arduino.voltage['vcc']
        if vin < 3 or vin > 3.6:
            return '3v3 Voltage Bad' + (': %.2f' % vin)
        if vcc < 4.5 or vcc > 5.5:
            return '5v Voltage Bad' + (': %.2f' % vcc)
        return False
                
    def poll(self):            
        t0 = time.monotonic()
        for i in [self.gpio, self.arduino, self.lirc]:
#            try:
            if 1:
                if not i:
                    continue
                events = i.poll()
                for event in events:
                    print('apply', event, time.monotonic())
                    self.apply_code(*event)
#            except Exception as e:
#                print('WARNING, failed to poll!!', e, i)

        t1 = time.monotonic()
        msgs = self.client.receive()
        t2 = time.monotonic()
        for name, value in msgs.items():
            self.last_msg[name] = value

        for i in [self.lcd, self.web]:
            i.poll()
        t3 = time.monotonic()
        for key, tc in self.keytimes.items():
            t, c = tc
            dt = t3 - t
            if dt > .6:
                print('keyup event lost, releasing key from timeout', key, t3, dt)
                self.apply_code(key, 0)
                self.keytimeouts[key] = c # don't apply this code if we eventually receive it
                break

        # receive heading once per second if autopilot is not enabled
        self.client.watch('ap.heading', False if self.last_msg['ap.enabled'] else 1)

        # timeout manual move
        if self.servo_timeout:
            if time.monotonic() > self.servo_timeout:
                if self.client:
                    self.client.set('servo.command', 0) # stop
                self.servo_timeout = 0

        # set web status
        if self.client.connection:
            self.web.set_status('connected')
            if not self.client.registered:
                #self.poller.register(self.client.connection.fileno(), select.POLLIN)
                #self.poller.register(self.lcd.client.connection.fileno(), select.POLLIN)
                self.client.registered = True
        else:
            self.client.registered = False
            self.web.set_status('disconnected')

        t4 = time.monotonic()
        dt = t3-t0
        period = max(.2 - dt, .01)

        if not self.lirc.registered:
            fileno = self.lirc.fileno()
            if fileno:
                self.poller.register(fileno, select.POLLIN)
                self.lirc.registered = True
        
        e=self.poller.poll(1000*period)
        #print('hattime', time.monotonic(), e)
        #print('hat times', t1-t0, t2-t1, t3-t2, t4-t3, period, dt)

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
