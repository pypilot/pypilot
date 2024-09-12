
#!/usr/bin/env python
#
#   Copyright (C) 2023 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time
print('hat start', time.monotonic())
import os, sys, signal, select
from pypilot import pyjson
from pypilot.client import pypilotClient
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gpio
import lcd
import arduino

print('hat import done', time.monotonic())

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
        self.lcd.keypad(self.index, count)
        
class ActionPypilot(Action):
    def  __init__(self, hat, name, pypilot_name, pypilot_value=None):
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
        # set heading to current heading
        if self.hat.client and not count and 'ap.heading' in self.hat.last_msg:
            self.hat.client.set('ap.heading_command', self.hat.last_msg['ap.heading'])
        super(ActionEngage, self).trigger(count)

class ActionMode(ActionEngage):
    def  __init__(self, hat, mode):
        super(ActionMode, self).__init__(hat)
        self.mode = mode
        self.name = mode + ' mode'

    def trigger(self, count):
        if self.hat.client and not count:
            self.hat.client.set('ap.mode', self.mode)
        
        super(ActionMode, self).trigger(count)
            
class ActionHeading(Action):
    def __init__(self, hat, offset):
        super(ActionHeading, self).__init__(hat, ('-' if offset < 0 else '+') + str(abs(offset)))
        self.offset = offset

    def trigger(self, count):
        if not self.hat.client:
            return

        if self.hat.last_msg['ap.enabled']:
            if not count: # only do something if a key is released
                off = self.offset
                if 'wind' in self.hat.last_msg['ap.mode']:
                    off = -off
                self.hat.client.set('ap.heading_command',
                                    int(self.hat.last_msg['ap.heading_command']) + off)
        elif count >= 0: # manual mode
            if count:
                self.hat.servo_timeout = time.monotonic() + .5
                self.hat.servo_command = -1 if self.offset > 0 else 1
            else:
                self.hat.servo_timeout = 0
                self.hat.servo_command = 0
                
            self.hat.client.set('servo.command', self.hat.servo_command)
            self.hat.client.poll() # reduce lag

class ActionTack(ActionPypilot):
    def  __init__(self, hat, name, direction):
        super(ActionTack, self).__init__(hat, name, 'ap.tack.state', 'begin')
        self.direction = direction
                                
    def trigger(self, count):
        if self.hat.client and not count:
            state =  self.hat.last_msg['ap.tack.state']
            if state == 'none':
                self.hat.client.set('ap.tack.direction', self.direction)
                self.hat.client.set('ap.tack.state', 'begin')
            else:
                direction =  self.hat.last_msg['ap.tack.direction']
                if direction != self.direction: # cancel tack by tacking "other way"
                    self.hat.client.set('ap.tack.state', 'none')

class ActionDodge(ActionPypilot):
    def  __init__(self, hat, name, direction):
        super(ActionDodge, self).__init__(hat, name, 'servo.command')
        self.direction = direction

    def trigger(self, count):
        if self.hat.client:
            value = self.direction if count else 0
            self.hat.client.set(self.pypilot_name, value)

class ActionProfile(ActionPypilot):
    def __init__(self, hat, profile):
        super(ActionProfile, self).__init__(hat, 'profile ' + profile, 'profile', profile)
        
class ActionProfileRelative(ActionPypilot):
    def __init__(self, hat, name, offset):
        super(ActionProfileRelative, self).__init__(hat, 'profile ' + name, 'profile')
        self.offset = offset

    def trigger(self, count):
        profile = self.hat.last_msg['profile']
        profiles = self.hat.last_msg['profiles']

        if profile in profiles:
            index = (profiles.index(profile) + self.offset) % len(profiles)
            self.value = profiles[index]
            super(ActionProfileRelative, self).trigger(count)

class ActionCommand(Action):
    def __init__(self, name, command):
        super(ActionCommand, self).__init__(None, name)
        self.command = command

    def trigger(self, count):
        os.system(self.command)

class Process():
    def __init__(self, hat):
        self.hat = hat
        self.create()            
    
    def send(self, value):
        if self.process:
            self.pipe.send(value, maxdt=.1)

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
                if os.getenv('USER') == 'tc' and time.monotonic() < 360:
                    time.sleep(30) # delay loading web and wait until modules are loaded
                else:
                    time.sleep(5) # delay less on other platforms
                try:
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
            if 'host' in msg:
                print('host changed, exiting', msg['host'])
                exit(0) # respawn
            elif 'adc_channels' in msg:
                if self.hat.arduino:
                    self.hat.arduino.config('arduino.adc_channels', msg['adc_channels'])

class Arduino(Process):
    def __init__(self, hat):
        super(Arduino, self).__init__(hat)
        self.voltage = {'vcc': 5, 'vin': 3.3}
        self.status = 'Not Connected'
        self.need_restart = 0

    def create(self):
        def process(pipe, config):
            print('arduino process on', os.getpid())
            if os.system("renice -5 %d" % os.getpid()):
                print('warning, failed to renice hat arduino process')
            time.sleep(3)
            while True:
                arduino.arduino_process(pipe, config)
                time.sleep(15)
        super(Arduino, self).create(process)

    def poll(self):
        ret = []
        if self.need_restart:
            t0 = time.monotonic()
            if self.need_restart < t0:
                if os.system('which avrdude') == 0:
                    print('avrdude available, resetting hat to update firmware')
                    exit(0)
                self.need_restart = t0 + 10 # try again in 10 seconds

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
                    self.hat.lcd.send(msg)
                elif key == 'analog':
                    config = self.hat.config['arduino.adc_channels']
                    adc_count = len(config)
                    for i in range(adc_count):
                        if i >= len(config):
                            break
                        adc = code[i]
                        if config[i] == 'none':
                            break
                        elif config[i] == 'control':
                            self.hat.client.send('servo.command', (adc-512)/512)
                            break
                        elif config[i] == 'user':
                            print("GOT USER ADC", adc)
                            break
                    pass
                elif key == 'version':
                    old_version = self.hat.config.get('arduino_firmware_version')
                    if old_version != code: 
                        print('actual hat version update from', old_version, 'to', code)
                        self.hat.config['arduino_firmware_version'] = code;
                        self.hat.write_config()
                    available_version = self.hat.config.get('arduino_firmware_version_available', 0)
                    if available_version > code:
                        print('new firmware version ',
                              available_version,
                              ' available to update current version ', code)
                        # restart once avrdude is available
                        self.need_restart = time.monotonic()
                else:
                    ret.append(msg)
        return ret

class LCD(Process):
    def __init__(self, hat):
        super(LCD, self).__init__(hat)
    
    def create(self):
        def process(pipe, config):
            import lcd
            print('lcd process on', os.getpid())
            self.lcd = lcd.LCD(self.hat.config)
            self.lcd.pipe = pipe

            if self.lcd.use_glut:
                from OpenGL.GLUT import glutMainLoop, glutIdleFunc
                glutIdleFunc(self.lcd.poll)
                glutMainLoop()
            else:
                while True:
                    self.lcd.poll()
            
        super(LCD, self).create(process)

    def keypad(self, index, count):
        self.send((index, count))
        
    def poll(self):
        ret = []
        while True:
            msg = self.pipe.recv()
            if not msg:
                break
            key, code = msg
            if key == 'write_config':
                self.hat.config['lcd'] = code
                self.hat.write_config()
            elif key == 'buzzer' or key == 'backlight':
                if self.hat.arduino:
                    self.hat.arduino.send(msg)
            
class Hat(object):
    def __init__(self):
        # default config
        self.config = {'host': 'localhost', 'actions': {},
                       'arduino.adc_channels': [],
                       'pi.ir': True, 'arduino.ir': False,
                       'arduino.nmea.in': False, 'arduino.nmea.out': False,
                       'arduino.nmea.baud': 4800,
                       'lcd': {}}
        self.configfilename = os.getenv('HOME') + '/.pypilot/hat.conf'

        # read config
        self.client = False
        if self.read_config(self.configfilename):
            self.write_config('.bak')
        else:
            print('failed to read config file, trying backup config')
            self.read_config(self.configfilename + '.bak')

        host = self.config['host']
        print('host', host, time.monotonic())
        self.client = pypilotClient(host)
        self.client.registered = False
            
        # read hardware config
        try:
            configfile = '/proc/device-tree/hat/custom_0'
            f = open(configfile)
            hat_config = pyjson.loads(f.read())
            f.close()
            print('loaded device tree hat config', time.monotonic())
            if not 'hat' in self.config or hat_config != self.config['hat']:
                self.config['hat'] = hat_config
                print('writing device tree hat to hat.conf')
                self.write_config()
        except Exception as e:
            print('failed to load', configfile, ':', e)
            
        if not 'hat' in self.config:
            print('assuming original 26 pin tinypilot with nokia5110 display')
            self.config['hat'] = {'lcd':{'driver':'default',
                                         'port':'/dev/spidev0.0'},
                                  'lirc':'gpio4'}
            self.write_config()

        # possibly update firmware, before initializing lcd (which shares SPI bus)
        arduino.update_firmware(self.config)

        self.servo_timeout = 0
        self.servo_command = 0
        self.last_msg = {'ap.enabled': False,
                         'ap.heading_command': 0,
                         'ap.mode': '',
                         'profile': None,
                         'profiles': []}

        if len(sys.argv) > 1:
            self.config['host'] = sys.argv[1]
            self.write_config()

        self.poller = select.poll()        
        self.gpio = gpio.gpio()
        self.poller.register(self.gpio.pipe[1], select.POLLIN)
        
        self.lcd = LCD(self)
        #time.sleep(1)

        self.watchlist = ['ap.enabled', 'ap.heading_command', 'ap.mode']
        self.watchlist += ['profile', 'profiles']
        self.watchlist += ['ap.tack.state', 'ap.tack.direction']

        for name in self.watchlist:
            self.client.watch(name)

        # receive heading once per second for mode changes
        self.client.watch('ap.heading', 1)
            
        if 'arduino' in self.config['hat']:
            self.arduino = Arduino(self)
            self.poller.register(self.arduino.pipe, select.POLLIN)
        else:
            self.arduino = False

        self.lcd.poll()

        import lircd
        self.lirc = lircd.lirc(self.config)
        self.lirc.registered = False
        self.keytime = False

        self.inputs = [self.gpio, self.arduino, self.lirc]

        # keypad for lcd interface
        self.actions = []
        self.profile_actions = []

        keypadnames = ['-10_', '-1_', '+1_', '+10_', 'auto_', 'menu_', 'mode_']
        for i in range(len(keypadnames)):
            self.actions.append(ActionKeypad(self.lcd, i, keypadnames[i]))

        # stateless actions for autopilot control
        self.actions += [ActionHeading(self, -1),
                         ActionHeading(self,  1),
                         ActionHeading(self, -2),
                         ActionHeading(self,  2),
                         ActionHeading(self, -5),
                         ActionHeading(self,  5),
                         ActionHeading(self, -10),
                         ActionHeading(self,  10),
                         ActionPypilot(self, 'center', 'servo.position', 0),
                         ActionTack(self, 'tack port', 'port'),
                         ActionTack(self, 'tack starboard', 'starboard'),
                         ActionDodge(self, 'dodge port', -1),
                         ActionDodge(self, 'dodge starboard', 1),
                         ActionPypilot(self, 'standby', 'ap.enabled', False),
                         ActionEngage(self)
        ]

        if 'modes' in self.config:
            for mode in self.config['modes']:
                self.actions.append(ActionMode(self, mode))

        # actions determined by the server (different pilots) not yet populated here
        for name in self.config['actions']:
            if name.startswith('pilot '):
                self.actions.append(ActionPypilot(self, name, 'ap.pilot', name.replace('pilot ', '', 1)))

        # execute an arbitrary command, eg: shutdown
        self.actions.append(ActionCommand('shutdown', 'shutdown -h now'))
                
        # useful to unassign a key
        self.actions.append(ActionNone())

        # config['actions'] must be kept in order as web interface depends on it...
        cfg = self.config['actions']
        self.config['actions'] = {}
        for action in self.actions:
            if not action.name in cfg:
                self.config['actions'][action.name] = []
            else:
                self.config['actions'][action.name] = cfg[action.name]

        for name in cfg:
            if name.startswith('profile '):
                self.config['actions'][name] = cfg[name]

        self.web = Web(self)

        def cleanup(signal_number, frame=None):
            print('got signal', signal_number, 'cleaning up', os.getpid())
            childpids = []
            processes = [self.arduino, self.web, self.lcd]
            for process in processes:
                if process and process.process:
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
                try:
                    sys.stdout.flush()
                except Exception as e:
                    print('failed to flush stdout', e)
            for process in processes:
                if process:
                    process.process = False

            raise KeyboardInterrupt # to get backtrace on all processes
            try:
                sys.stdout.flush()
            except Exception as e:
                print('failed to flush stdout2', e)

        for s in range(1, 16):
            if s != 9 and s != 13:
                signal.signal(s, cleanup)
        signal.signal(signal.SIGCHLD, cleanup)

    def read_config(self, filename):
        print('loading config file:', filename)
        try:
            file = open(filename)
            config = pyjson.loads(file.read())
            file.close()
            for name in config:
                self.config[name] = config[name]
            return True
        except Exception as e:
            print('config failed:', e)
            try:
                file = open(filename)
                print('bad config: ', file.read())
                file.close()
            except Exception as e:
                print('config read exception printing exception', e)
        return False
        
    def write_config(self, suffix=''):
        print('write config', suffix)
        actions = self.config['actions']
        for name in list(actions):
            if not actions[name] and name[:6] != 'pilot ':
                del actions[name]

        if self.client and not 'modes' in self.config:
            values = self.client.get_values()
            if 'ap.mode' in values:
                self.config['modes'] = values['ap.mode']['choices']
            
        try:
            f = open(self.configfilename+suffix, 'w')
            f.write(pyjson.dumps(self.config) + '\n')
            f.close()
        except IOError:
            print('failed to save config file:', self.configfilename+suffix)

    def update_config(self, name, value):
        if name in self.config and self.config[name] == value:
            return
        
        if self.arduino:
            if name == 'actions' or name.startswith('arduino.'):
                self.arduino.send((name, value))

        self.config[name] = value

    def apply_code(self, key, count):
        if count == 1 or count == -1:
            self.web.send({'key': key})

        actions = self.config['actions']
        for action in self.actions:
            if not action.name in actions:
                actions[action.name] = []
            keys = actions[action.name]
            if key in keys:
                if count <= 0:
                    self.web.send({'action': action.name})
                    if not self.keytime:
                        return # do not apply keyup if already applied
                    self.keytime = False
                else:
                    self.keytime = key, time.monotonic()

                if count >= 0:
                    action.trigger(count)
                return

        self.web.send({'action': 'none'})

    def update_values(self):
        values = self.client.list_values()
        if values:
            if 'ap.pilot' in values:                
                pilots = values['ap.pilot']['choices']
                update = False
                for pilot in pilots:
                    name = 'pilot '+pilot
                    if not name in self.config['actions']:
                        print('adding pilot', pilot)
                        self.config['actions'][name] = []
                        update = True
                for name in list(self.config['actions']):
                    if name.startswith('pilot '):
                        pilot = name.replace('pilot ', '', 1)
                        if not pilot in pilots:
                            print('removing pilot', pilot)
                            del self.config['actions'][name]
                            update = True
                if update:
                    self.write_config()
                    print('shutting down since pilots updated')
                    exit(0) #respawn

    def poll(self):            
        t0 = time.monotonic()
        for i in self.inputs:
            try:
                if not i:
                    continue

                keycount = False
                events = i.poll()
                for event in events:
                    key, count = event
                    if count: # if key is released
                        keycount = key, count
                    else:
                        if keycount:
                            self.apply_code(*keycount)
                            keycount = False
                        if self.keytime:
                            self.apply_code(self.keytime[0], 0)
                            self.keytime = False
                if keycount:
                    self.apply_code(*keycount)

            except Exception as e:
                self.inputs.remove(i)
                print('WARNING, failed to poll!!', e, i)
                del i
                return

        t1 = time.monotonic()
        msgs = self.client.receive()
        t2 = time.monotonic()
        for name, value in msgs.items():
            self.last_msg[name] = value

        ena = 'ap.enabled' #need to know if we are enabled for udp control
        if self.arduino and ena in msgs:
            self.arduino.send((ena, msgs[ena]))

        if 'profiles' in msgs:
            profiles = msgs['profiles']
            self.web.send({'profiles': profiles + ['prev', 'next']})
            for action in self.profile_actions:
                self.actions.remove(action)
            self.profile_actions = []
            for profile in profiles:
                action = ActionProfile(self, profile)
                self.profile_actions.append(action)
                self.actions.append(action)
            for action in ActionProfileRelative(self, 'prev', -1), \
                          ActionProfileRelative(self, 'next', 1):
                self.profile_actions.append(action)
                self.actions.append(action)

        for i in [self.lcd, self.web]:
            i.poll()
        t3 = time.monotonic()
        if self.keytime:
            key, t = self.keytime
            dt = t3 - t
            if dt > .4:
                print('keyup event lost, releasing key from timeout', key, t3, dt)
                self.apply_code(key, 0)
                self.keytime = False
                self.keycounts = {}

        # timeout manual move
        if self.servo_timeout:
            if time.monotonic() > self.servo_timeout:
                if self.client:
                    self.client.set('servo.command', 0) # stop
                self.servo_timeout = 0
            #else:
             #   self.client.set('servo.command', self.servo_command) # continue
            self.client.poll() # reduce lag

        # set web status
        if self.client.connection:
            self.web.set_status('connected')
            if not self.client.registered:
                #self.poller.register(self.client.connection.fileno(), select.POLLIN)
                self.client.registered = True

            self.update_values()                    
        else:
            self.client.registered = False
            self.web.set_status('disconnected')

        t4 = time.monotonic()
        dt = t3-t0
        period = .01 if self.servo_timeout else .05 if self.gpio.keypin else max(1 - dt, .01)

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
    print('hat init complete', time.monotonic())
    while True:
        hat.poll()

if __name__ == '__main__':
    main()
