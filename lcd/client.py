#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import sys, os, time, math

use_glut = 'DISPLAY' in os.environ
if use_glut:
    import glut

import gettext
import json
_ = lambda x : x # initially no translation
    
orangepi = False
try:
    import RPi.GPIO as GPIO
    print 'have gpio for raspberry pi'

except ImportError:
    try:
        import OPi.GPIO as GPIO
        orangepi = True
        print 'have gpio for orange pi'
    except:
        print 'No gpio available'
        GPIO = None

try:
    import pylirc as LIRC
    print 'have lirc for remote control'
except:
    print 'no lirc available'
    LIRC = None

from signalk.client import SignalKClient

from ugfx import ugfx
import font

class LCDMenu():
    def __init__(self, lcd, name, items, prev=False):
        self.lcd = lcd
        self.selection = 0
        self.name = name
        if not lcd.have_select:
            items.append((_('return'), self.lcd.menu_back))
        self.items = items
        self.prev = prev
        self.display_hook = False

    # return oldest menu
    def adam(self):
        if self.prev:
            return self.prev.adam()
        return self

    def display(self):
        self.lcd.fittext(rectangle(0, 0, 1, .25), self.name)

        firstitem = .25
        y = firstitem
        for item in self.items:
            size = self.lcd.fittext(rectangle(0, y, 1, .15), item[0])[0] + .25
            if len(item) > 2:
                sliderarea = rectangle(size, y+.05, (1-size), .07)
                self.lcd.rectangle(sliderarea, .015)
                sliderarea.width *= item[2]()
                self.lcd.rectangle(sliderarea)
                self.lcd.client.get(item[3])
            y += .15

        y = .15*self.selection + firstitem + .03
        self.lcd.invertrectangle(rectangle(0, y, 1, .13))
        if self.display_hook:
            self.display_hook()

class RangeEdit():
    def __init__(self, name, desc, id, signalk, lcd, minval, maxval, step):
        self.name = name
        if type(desc) == type(u''):
            self.desc = lambda : desc
        else:
            self.desc = desc
        self.id = id
        self.signalk = signalk
        self.range = minval, maxval, step
        self.lcd = lcd
        self.value = lcd.last_msg[id] if signalk else lcd.config[id]
        self.lastmovetime = 0
     
    def move(self, delta):
        if self.signalk: #config items rounded to integer
            v = self.value + delta*self.range[2]
        else:
            if delta > 0:
                delta = max(1, delta)
            else:
                delta = min(-1, delta)
            v = self.value + delta*self.range[2]
            v = round(v)
            
        v = min(v, self.range[1])
        v = max(v, self.range[0])
        self.value = v
        if self.signalk:
            self.lcd.client.set(self.id, v)
        else:
            self.lcd.config[self.id] = v
        self.lastmovetime = time.time()

    def display(self):
        self.lcd.surface.fill(black)
        self.lcd.fittext(rectangle(0, 0, 1, .3), self.name, True)
        self.lcd.fittext(rectangle(0, .3, 1, .3), self.desc(), True)

        # update name
        if time.time()-self.lastmovetime > 1:
            if self.signalk:
                self.value = self.lcd.last_msg[self.id]
        
        v = self.value
        try:
            v = str(round(10000*v)/10000)
            while len(v) < 6:
                v+='0'
        except:
            pass
        
        self.lcd.fittext(rectangle(0, .6, 1, .18), v)

        sliderarea = rectangle(0, .8, 1, .1)
        try:
            self.lcd.rectangle(sliderarea, .015)
            xp = (float(v) - self.range[0]) / (self.range[1] - self.range[0])
            sliderarea.width *= xp
            self.lcd.rectangle(sliderarea)
        except:
            pass
        # poll for updates
        if self.signalk:
            self.lcd.client.get(self.id)

white = ugfx.color(255, 255, 255)
black = ugfx.color(0, 0, 0)

class rectangle():
    def __init__(self, x, y, width, height):
        self.x, self.y, self.width, self.height = x, y, width, height

AUTO, MENU, UP, DOWN, SELECT, LEFT, RIGHT = range(7)
keynames = {'auto': AUTO, 'menu': MENU, 'up': UP, 'down': DOWN, 'select': SELECT, 'left': LEFT, 'right': RIGHT}
gains = ['ap.P', 'ap.I', 'ap.D', 'ap.FF']

class LCDClient():
    def __init__(self, screen):
#        w, h, self.bw = 44, 84, 1
#        w, h, self.bw = 64, 128, 1
#        w, h, self.bw = 320, 480, 0

        self.config = {}
        self.configfilename = os.getenv('HOME') + '/.pypilot/lcdclient.conf' 
        self.config['contrast'] = 60
        self.config['invert'] = False
        self.config['flip'] = False
        self.config['language'] = 'en'
        self.config['bigstep'] = 10
        self.config['smallstep'] = 1

        print 'loading load config file:', self.configfilename
        try:
            file = open(self.configfilename)
            config = json.loads(file.readline())
            for name in config:
                self.config[name] = config[name]
        except:
            print 'failed to load config file:', self.configfilename
        if screen:
            w, h = screen.width, screen.height
            mul = int(math.ceil(w / 48.0))

            self.bw = 1 if w < 256 else False

            width = min(w, 48*mul)
            self.surface = ugfx.surface(width, width*h/w, screen.bypp, None)
            self.frameperiod = .25 # 4 frames a second possible

        else:
            self.surface = None
            self.frameperiod = 1

        self.set_language(self.config['language'])
        self.range_edit = False

        self.modes = {'compass': self.have_compass,
                      'gps':     self.have_gps,
                      'wind':    self.have_wind,
                      'true wind': self.have_true_wind};
        self.modes_list = ['compass', 'gps', 'wind', 'true wind'] # in order

        self.initial_gets = gains + ['servo.min_speed', 'servo.max_speed', 'servo.max_current', 'servo.period', 'imu.alignmentCounter']

        self.have_select = False
        self.create_mainmenu()

        self.longsleep = 30
        self.display_page = self.display_connecting
        self.connecting_dots = 0

        self.client = False

        self.keystate = {}
        self.keypad = [False, False, False, False, False, False, False, False]
        self.keypadup = list(self.keypad)

        self.blink = black, white
        self.control = False
        self.wifi = False
        self.overcurrent_time = 0
        
        if orangepi:
            self.pins = [11, 16, 13, 15, 12]
        else:
            self.pins = [17, 23, 27, 22, 18]

        if GPIO:
            if orangepi:
                for pin in self.pins:
                    cmd = 'gpio -1 mode ' + str(pin) + ' up'
                    os.system(cmd)
                GPIO.setmode(GPIO.BOARD)
            else:
                GPIO.setmode(GPIO.BCM)

            for pin in self.pins:
                try:
                    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                    pass
                except RuntimeError:
                    os.system("sudo chown tc /dev/gpiomem")
                    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                    
                def cbr(channel):
                    self.longsleep = 0

                GPIO.add_event_detect(pin, GPIO.BOTH, callback=cbr, bouncetime=20)

        global LIRC
        if LIRC:
            try:
                LIRC.init('pypilot')
                self.lirctime = False
            except:
                print 'failed to initialize lirc. is .lircrc missing?'
                LIRC = None

    def get(self, name):
        if self.client:
            self.client.get(name)

    def set_language(self, name):
        language = gettext.translation('pypilot_lcdclient',
                                       os.path.abspath(os.path.dirname(__file__)) + '/locale',
                                       languages=[name], fallback=True)
        global _
        _ = language.ugettext
        self.config['language'] = name

    def save_config(self):
        try:
            file = open(self.configfilename, 'w')
            file.write(json.dumps(self.config) + '\n')
        except IOError:
            print 'failed to save config file:', self.configfilename
            
    def create_mainmenu(self):
        def value_edit(name, desc, signalk_name, value=False):
            min = self.value_list[signalk_name]['min']
            max = self.value_list[signalk_name]['max']
            step = (max-min)/100.0

            def thunk():
                self.range_edit=RangeEdit(name, desc, signalk_name,
                                          True, self, min, max, step)
                return self.range_edit.display

            if value:
                return name, thunk, lambda : (self.last_msg[signalk_name]-min) / (max - min), signalk_name
            return name, thunk
        def config_edit(name, desc, config_name, min, max, step):
            def thunk():
                self.range_edit=RangeEdit(name, desc, config_name,
                                          False, self, min, max, step)
                return self.range_edit.display
            return name, thunk

        def gain():
            self.menu = LCDMenu(self, _('Gain'),
                                [value_edit('P', _('position gain'), 'ap.P', True),
                                 value_edit('I', _('integral gain'), 'ap.I', True),
                                 value_edit('D', _('rate gain'),     'ap.D', True),
                                 value_edit('P2', _('position2 gain'), 'ap.P2', True),
                                 value_edit('D2', _('rate2 gain'),     'ap.D2',  True)], self.menu)
            return self.display_menu

        def level():
            self.client.set('imu.alignmentCounter', 100)
            return self.display_page

        def calibrate():
            def getheading():
                self.get('imu.heading')
                try:
                    return '%.1f' % self.last_msg['imu.heading']
                except:
                    return str(self.last_msg['imu.heading'])

            self.menu = LCDMenu(self, _('Calibrate'),
                                [(_('level'), level),
                                 value_edit(_('heading'), getheading, 'imu.heading_offset'),
                                 (_('info'), lambda : self.display_calibrate_info)],
                                self.menu)
            self.menu.display_hook = self.display_calibrate
            return self.display_menu

        def settings():
            def mode():
                def set_mode(name):
                    def thunk():
                        self.client.set('ap.mode', name)
                        self.menu = self.menu.adam()
                        return self.display_control
                    return thunk
                modes = []
                index = 0
                selection = 0
                for mode in self.modes:
                    if self.modes[mode]():
                        modes.append((mode, set_mode(mode)))
                        if mode == self.last_msg['ap.mode']:
                            selection = index
                        index+=1
                self.menu = LCDMenu(self, _('Mode'), modes, self.menu)
                self.menu.selection = selection
            
                return self.display_menu

            def motor():
                self.menu = LCDMenu(self, _('Motor'),
                                    [value_edit(_('min speed'), _('relative'), 'servo.min_speed'),
                                     value_edit(_('max speed'), _('relative'), 'servo.max_speed'),
                                     value_edit(_('max current'), _('amps'), 'servo.max_current'),
                                     value_edit(_('period'), _('seconds'), 'servo.period')],
                                    self.menu)
                return self.display_menu

            
            def filter():
                self.menu = LCDMenu(self, _('Filter'),
                                    [value_edit(_('heading'), _('relative'), 'imu.heading_lowpass_constant'),
                                     value_edit(_("heading'"), _('relative'), 'imu.headingraterate_lowpass_constant'),
                                     value_edit(_("heading''"), _('relative'), 'imu.headingraterate_lowpass_constant')],
                                    self.menu)
                return self.display_menu

            def control():
                self.menu = LCDMenu(self, _('Control'),
                                    [config_edit(_('small step'), _('degrees'), 'smallstep', 1, 5, 1),
                                     config_edit(_('big step'), _('degrees'), 'bigstep', 5, 20, 5)],
                                    self.menu)
                return self.display_menu

            def contrast():
                self.range_edit = self.contrast_edit
                return self.range_edit.display
            
            def invert():
                self.config['invert'] = not self.config['invert']
                self.save_config()
                return self.display_menu

            def flip():
                self.config['flip'] = not self.config['flip']
                self.save_config()
                return self.display_menu

            def display():
                self.menu = LCDMenu(self, _('Display'),
                                    [config_edit(_('contrast'), '', 'contrast', 30, 90, 1),
                                     (_('invert'), invert),
                                     (_('flip'), flip)],
                                    self.menu)
                return self.display_menu

            def language():
                def set_language(name):
                    def thunk():
                        self.set_language(name)
                        self.save_config()
                        self.create_mainmenu()
                        return language()
                    return thunk

                languages = [(_('English'), 'en'), (_('French'), 'fr'),
                             (_('Spanish'), 'es')]
                index, selection = 0, 0
                for lang in languages:
                    if lang[1] == self.config['language']:
                        selection = index
                    index+=1
                self.menu = LCDMenu(self, _('Language'), map(lambda lang : (lang[0], set_language(lang[1])), languages), self.menu)
                self.menu.selection = selection
            
                return self.display_menu
            
            self.menu = LCDMenu(self, _('Settings'),
                                [(_('mode'), mode),
                                 (_('motor'), motor),
                                 (_('filter'), filter),
                                 (_('control'), control),
                                 (_('display'), display),
                                 (_('language'), language)],
                                self.menu)
            return self.display_menu

        self.menu = LCDMenu(self, _('Menu'),
                            [(_('gain'), gain),
                             (_('calibrate'), calibrate),
                             (_('settings'), settings),
                             (_('info'), lambda : self.display_info)])
        self.info_page = 0
        return self.display_menu

    def text(self, pos, text, size, crop=False):
        pos = int(pos[0]*self.surface.width), int(pos[1]*self.surface.height)
        size = int(size*self.surface.width/48)
        size = font.draw(self.surface, pos, text, size, self.bw, crop)
        return float(size[0])/self.surface.width, float(size[1])/self.surface.height

    def fittext(self, rect, text, wordwrap=False, fill='none'):
        #print 'fittext', text, wordwrap, fill
        if fill != 'none':
            self.surface.box(*(self.convrect(rect) + [fill]))
        metric_size = 16
        if wordwrap:
            words = text.split(' ')
            spacewidth = font.draw(self.surface, False, ' ', metric_size, self.bw)[0]
            if len(words) < 2: # need at least 2 words to wrap
                return self.fittext(rect, text, False, fill)
            metrics = map(lambda word : (word, font.draw(self.surface, False, word, metric_size, self.bw)), words)

            widths = map(lambda metric : metric[1][0], metrics)
            maxwordwidth = apply(max, widths)
            totalwidth = sum(widths) + spacewidth * (len(words) - 1)
            size = 0
            # not very efficient... just tries each x position
            # for wrapping to maximize final font size
            for wrappos in range(maxwordwidth, totalwidth+1):
                posx, posy = 0, 0
                curtext = ''
                lineheight = 0
                maxw = 0
                for metric in metrics:
                    word, (width, height) = metric
                    if posx > 0:
                        width += spacewidth
                    if posx + width > wrappos:
                        curtext += '\n'
                        posx = 0
                        posy += lineheight
                        lineheight = 0

                    if posx > 0:
                        curtext += ' '
                    curtext += word
                    lineheight = max(lineheight, height)
                    posx += width
                    maxw = max(maxw, posx)
                maxh = posy + lineheight
                    
                s = maxw, maxh
                if s[0] == 0 or s[1] == 0:
                    continue
                sw = self.surface.width * float(rect.width) / s[0]
                sh = self.surface.height * float(rect.height) / s[1]
                cursize = int(min(sw*metric_size, sh*metric_size))
                if cursize > size:
                    size = cursize
                    text = curtext
        else:
            s = font.draw(self.surface, False, text, metric_size, self.bw)
            if s[0] == 0 or s[1] == 0:
                return 0, 0
            sw = self.surface.width * float(rect.width) / s[0]
            sh = self.surface.height * float(rect.height) / s[1]
            size = int(min(sw*metric_size, sh*metric_size))

        pos = int(rect.x*self.surface.width), int(rect.y*self.surface.height)
        size = font.draw(self.surface, pos, text, size, self.bw)
        return float(size[0])/self.surface.width, float(size[1])/self.surface.height

    def line(self, x1, y1, x2, y2):
        self.surface.line(x1, y1, x2, y2, black)

    def convbox(self, x1, y1, x2, y2):
        w, h = self.surface.width - 1, self.surface.height - 1
        return [int(x1*w), int(y1*h), int(x2*w), int(y2*h)]

    def invertrectangle(self, rect):
        apply(self.surface.invert, self.convbox(rect.x, rect.y, rect.x+rect.width, rect.y+rect.height))
    def convrect(self, rect):
        return self.convbox(rect.x, rect.y, rect.x+rect.width, rect.y+rect.height)

    def rectangle(self, rect, width = False):
        if not width:
            apply(self.surface.box, self.convrect(rect) + [white])
        else:
            box = self.convrect(rect)
            apply(self.surface.invert, box)
            if width:
                w, h = self.surface.width - 1, self.surface.height - 1
                px_width = int(max(1, min(w*width, h*width)))
                self.surface.invert(box[0]+px_width, box[1]+px_width, box[2]-px_width, box[3]-px_width)
        
    def connect(self):
        self.display_page = self.display_connecting
        
        watchlist = ['ap.enabled', 'ap.mode', 'ap.heading_command',
                     'gps.source', 'wind.source', 'servo.controller', 'servo.flags',
                     'imu.compass_calibration']
        poll_list = ['ap.heading']
        nalist = watchlist + poll_list + gains + \
        ['imu.pitch', 'imu.heel', 'ap.runtime', 'ap.version',
         'imu.heading',
         'imu.alignmentCounter',
         'imu.compass_calibration_age',
         'imu.heading_lowpass_constant', 'imu.headingrate_lowpass_constant',
         'imu.headingraterate_lowpass_constant',
         'servo.watts', 'servo.amp_hours', 'servo.max_current',
         'servo.min_speed', 'servo.max_speed']
        self.last_msg = {}
        for name in nalist:
            self.last_msg[name] = _('N/A')
        for name in ['gps.source', 'wind.source']:
            self.last_msg[name] = 'none'
        self.last_msg['ap.heading_command'] = 0
        self.last_msg['imu.heading_offset'] = 0

        host = ''
        if len(sys.argv) > 1:
            host = sys.argv[1]
        
        def on_con(client):
            self.value_list = {}
            request = {'method' : 'list'}
            client.send(request)
            for name in watchlist:
                client.watch(name)

        try:
            self.client = SignalKClient(on_con, host)
            self.display_page = self.display_control
            print 'connected'

            for request in self.initial_gets:
                self.get(request)
        except:
            self.client = False
            time.sleep(1)

    def round_last_msg(self, name, places):
        n = 10**places
        try:
            return str(round(self.last_msg[name]*n)/n)
        except:
            return str(self.last_msg[name])
            
    def have_compass(self):
        return True

    def have_gps(self):
        return self.last_msg['gps.source'] != 'none'

    def have_wind(self):
        return self.last_msg['wind.source'] != 'none'

    def have_true_wind(self):
        return self.have_gps() and self.have_wind()

    def display_wifi(self):
        wifi = False
        try:
            wlan0 = open('/sys/class/net/wlan0/operstate')
            line = wlan0.readline().rstrip()
            wlan0.close()
            if line == 'up':
                wifi = True
        except:
            pass

        if self.wifi != wifi:
            wifirect = rectangle(.3, .9, .6, .12)
            if wifi:           
                self.fittext(wifirect, 'WIFI')
            else:
                self.surface.box(*(self.convrect(wifirect) + [black]))
            self.wifi = wifi
    
    def display_control(self):
        if not self.control:
            self.surface.fill(black)
            self.control = {'heading': '   ', 'heading_command': '   ', 'mode': False}
        
        def draw_big_number(pos, num, lastnum):
            def nr(x):
                try:
                    s = str(int(round(x)))
                    while len(s) < 3:
                        s = ' ' + s
                    return s
                except:
                    return x

            num = nr(num)
            if lastnum:
                lastnum = nr(lastnum)

            if num == 'N/A' and lastnum != num:
                r = rectangle(pos[0], pos[1], 1, .4)
                self.fittext(r, num, False, black)
                return

            if self.surface.width < 256:
                size = 34
            else:
                size = 30

            for i in range(3):
                try:
                    if num[i] == lastnum[i]:
                        continue
                except:
                    pass
                x = pos[0]+float(i)/3
                self.surface.box(*(self.convrect(rectangle(x, pos[1], .34, .4)) + [black]))
                self.text((x, pos[1]), num[i], size, True)

        if type(self.last_msg['ap.heading']) == type(False):
            r = rectangle(0, 0, 1, .8)
            self.fittext(r, _('ERROR\ncompass or gyro failure!'), True, black)
            self.control['heading_command'] = 'no imu'
        else:
            draw_big_number((0,0), self.last_msg['ap.heading'], self.control['heading'])
            self.control['heading'] = self.last_msg['ap.heading']

        mode = self.last_msg['ap.mode']
        if 'OVERCURRENT' in self.last_msg['servo.flags']:
            self.overcurrent_time = time.time()

        if self.last_msg['servo.controller'] == 'none':
            if self.control['heading_command'] != 'no controller':
                self.fittext(rectangle(0, .4, 1, .35), _('WARNING no motor controller'), True, black)
                self.control['heading_command'] = 'no controller'
        elif time.time() - self.overcurrent_time < 5: # 5 seconds
            if self.control['heading_command'] != 'overcurrent':
                self.fittext(rectangle(0, .4, 1, .35), _('OVER CURRENT'), True, black)
                self.control['heading_command'] = 'overcurrent'
        elif 'OVERTEMP' in self.last_msg['servo.flags']:
            if self.control['heading_command'] != 'overtemp':
                self.fittext(rectangle(0, .4, 1, .35), _('OVER TEMP'), True, black)
                self.control['heading_command'] = 'overtemp'
        elif mode == 'gps' and not self.have_gps():
            if self.control['heading_command'] != 'no gps':
                self.fittext(rectangle(0, .4, 1, .4), _('GPS not detected'), True, black)
                self.control['heading_command'] = 'no gps'
        elif (mode == 'wind' or mode == 'true wind') and not self.have_wind():
            if self.control['heading_command'] != 'no wind':
                self.fittext(rectangle(0, .4, 1, .4), _('WIND not detected'), True, black)
                self.control['heading_command'] = 'no wind'
        else:
            # no warning, display the desired course or 'standby'
            if self.last_msg['ap.enabled'] != True:
                if self.control['heading_command'] != 'standby':
                    r = rectangle(0, .4, 1, .34)
                    self.fittext(r, _('standby'), False, black)
                    self.control['heading_command'] = 'standby'
            else:
                if self.control['heading_command'] != self.last_msg['ap.heading_command']:
                    draw_big_number((0,.4), self.last_msg['ap.heading_command'], self.control['heading_command'])
                    self.control['heading_command'] = self.last_msg['ap.heading_command']

                    self.control['mode'] = False # refresh mode

        def modes():
            return [self.have_compass(), self.have_gps(), self.have_wind(), self.have_true_wind()]
        warning = False
        if mode == 'compass':
            warning = False
            cal = self.last_msg['imu.compass_calibration']
            if cal == 'N/A':
                ndeviation = 0
            else:
                ndeviation = cal[1][0]
            def warncal(s):
                r = rectangle(0, .75, 1, .15)
                self.fittext(r, s, True, white)
                self.invertrectangle(r)
                self.control['mode'] = 'warning'
            if ndeviation == 0 and False:
                warncal(_('No Cal'))
                warning = True
            if ndeviation > 6:
                warncal(_('Bad Cal'))
                warning = True

        if not warning and \
           (self.control['mode'] != mode or self.control['modes'] != modes()):
            self.control['mode'] = mode
            self.control['modes'] = modes()

            #print 'mode', self.last_msg['ap.mode']
            modes = {'compass': ('C', self.have_compass, rectangle(0, .74, .25, .16)),
                     'gps':     ('G', self.have_gps,     rectangle(.25, .74, .25, .16)),
                     'wind':    ('W', self.have_wind,    rectangle(.5, .74, .25, .16)),
                     'true wind': ('T', self.have_true_wind, rectangle(.75, .74, .25, .16))}

            self.surface.box(*(self.convrect(rectangle(0, .74, 1, .18)) + [black]))
            for mode in modes:
                if modes[mode][1]():
                    self.fittext(modes[mode][2], modes[mode][0])
                if self.last_msg['ap.mode'] == mode:
                    r = modes[mode][2]
                    marg = .02
                    self.rectangle(rectangle(r.x-marg, r.y+marg, r.width-marg, r.height), .015)

            #self.control['mode'] = False # refresh mode
        self.display_wifi()

    def display_menu(self):
        self.surface.fill(black)
        self.menu.display()

    def display_calibrate(self):
        counter = self.last_msg['imu.alignmentCounter']
        if counter:
            r = rectangle(0, 0, 1, .25)
            r.height = .2
            self.fittext(r, ' %d%%' % (100-counter), False, black)
            r.width = 1-float(counter)/100
            r.height = .25
            self.invertrectangle(r)
        self.get('imu.alignmentCounter')
            
        self.fittext(rectangle(0, .86, .5, .14), self.round_last_msg('imu.pitch', 1))
        self.fittext(rectangle(.5, .86, .5, .14), self.round_last_msg('imu.heel', 1))
        self.get('imu.pitch')
        self.get('imu.heel')

    def display_connecting(self):
        self.surface.fill(black)
        self.fittext(rectangle(0, 0, 1, .4), _('connect to server'), True)
        dots = ''
        for i in range(self.connecting_dots):
            dots += '.'
        size = self.text((0, .4), dots, 12)
        self.connecting_dots += 1
        if size[0] >= 1:
            self.connecting_dots = 0
        self.display_wifi()
            
    def display_info(self):
        self.surface.fill(black)
        self.fittext(rectangle(0, 0, 1, .2), _('Info'))

        if self.info_page > 1:
            self.info_page = 0

        y = .2
        if self.info_page == 0:
            spacing = .11
            v = self.round_last_msg('servo.watts', 3)
            runtime = self.last_msg['ap.runtime'][:7]
            ah = self.round_last_msg('servo.amp_hours', 3)
            items = [_('Watts'), v, _('Amp Hours'), ah, _('runtime'), runtime]
            self.get('servo.watts')
            self.get('servo.amp_hours')
            self.get('ap.runtime')
        else:
            spacing = .18
            ver = self.last_msg['ap.version']

            items = [_('version'), ver, _('author'), 'Sean D\'Epagnier']
            self.get('ap.version')

        even, odd = 0, .05
        for item in items:
            self.fittext(rectangle(0, y, 1, spacing+even), item, True)
            y += spacing + even
            even, odd = odd, even


    def display_calibrate_info(self):
        self.surface.fill(black)
        self.fittext(rectangle(0, 0, 1, .3), _('Calibrate Info'), True)
        
        deviation = _('N/A')
        try:
            cal = self.last_msg['imu.compass_calibration']
            ndeviation = cal[0][1]
            #print ndeviation
            names = [(0, _('incomplete')), (.01, _('excellent')), (.02, _('good')),
                     (.04, _('fair')), (.06, _('poor')), (1000, _('bad'))]
            for n in names:
                if ndeviation <= n[0]:
                    deviation = n[1]
                    break
        except:
            pass
        
        self.fittext(rectangle(0, .3, 1, .15), _('compass'))
        self.fittext(rectangle(0, .42, 1, .23), deviation)
        self.fittext(rectangle(0, .65, .4, .15), _('age'))
        self.fittext(rectangle(0, .8, 1, .2), self.last_msg['imu.compass_calibration_age'][:7])
            
        #self.get('imu.compass_calibration')
        self.get('imu.compass_calibration_age')

    def display(self):
        self.display_page()

        if self.display_page != self.display_control:
            self.control = False
            self.wifi = False

        # status cursor
        w, h = self.surface.width, self.surface.height
        self.blink = self.blink[1], self.blink[0]
        size = h / 40
        self.surface.box(w-size-1, h-size-1, w-1, h-1, self.blink[0])

    def set(self, name, value):
        if self.client:
            self.client.set(name, value)

    def menu_back(self):
        if self.menu.prev:
            self.menu = self.menu.prev
            return self.display_menu
        return self.display_control
            
    def process_keys(self):                           
        if self.keypadup[AUTO]: # AUTO
            if self.last_msg['ap.enabled'] == False and self.display_page == self.display_control:
                self.set('ap.heading_command', self.last_msg['ap.heading'])
                self.set('ap.enabled', True)
            else:
                self.set('servo.command', 0) #stop
                self.set('ap.enabled', False)
        
            self.display_page = self.display_control

        if self.keypadup[SELECT]:
            if self.display_page == self.display_control:
                for t in range(len(self.modes_list)):
                    next_mode = self.modes_list[0]
                    self.modes_list = self.modes_list[1:] + [next_mode]
                    if next_mode != self.last_msg['ap.mode'] and \
                       self.modes[next_mode]():
                        self.client.set('ap.mode', next_mode)
                        break
            else:
                self.menu = self.menu.adam() # reset to main menu
                self.display_page = self.display_control

        # for up and down keys providing acceration
        sign = -1 if self.keypad[DOWN] or self.keypadup[DOWN] or self.keypad[LEFT] or self.keypadup[LEFT] else 1
        updownup = self.keypadup[UP] or self.keypadup[DOWN]
        updownheld = self.keypad[UP] > 10 or self.keypad[DOWN] > 10
        speed = float(1 if updownup else min(10, .004*max(self.keypad[UP], self.keypad[DOWN])**2.5))
        updown = updownheld or updownup
        if self.keypadup[LEFT] or self.keypadup[RIGHT]:
            updown = True
            speed = 10

        if self.display_page == self.display_control:                
            if self.keypadup[MENU] and self.surface: # MENU
                self.display_page = self.display_menu
            elif updown: # LEFT/RIGHT
                if self.last_msg['ap.enabled']:
                    if self.keypadup[LEFT] or self.keypadup[RIGHT]:
                        speed = self.config['bigstep']
                    else:
                        speed = self.config['smallstep']
                    cmd = self.last_msg['ap.heading_command'] + sign*speed
                    self.set('ap.heading_command', cmd)
                else:
                    self.set('servo.command', sign*(speed+8.0)/20)

        elif self.display_page == self.display_menu:
            if self.keypadup[UP]:
                self.menu.selection -= 1
                if self.menu.selection < 0:
                    self.menu.selection = len(self.menu.items)-1
            elif self.keypadup[DOWN]:
                self.menu.selection += 1
                if self.menu.selection == len(self.menu.items):
                    self.menu.selection = 0
            elif self.keypadup[MENU]:
                self.display_page = self.menu.items[self.menu.selection][1]()

        elif self.display_page == self.display_info or \
             self.display_page == self.display_calibrate_info:
            if self.keypadup[MENU]:
                self.display_page = self.display_menu
            if self.keypadup[UP] or self.keypadup[DOWN]:
                self.info_page += 1

        elif self.range_edit and self.display_page == self.range_edit.display:
            if self.keypadup[MENU]:
                self.display_page = self.display_menu
                if not self.range_edit.signalk:
                    self.save_config()
            elif updown:
                self.range_edit.move(sign*speed*.1)
        elif self.display_page == self.display_connecting:
            pass # no keys handled for this page
        else:
            print 'unknown display page', self.display_page

        for key in range(len(keynames)):
            if self.keypadup[key]:
                self.keypad[key] = self.keypadup[key] = False

    def key(self, k, down):
        if k >= 0 and k < len(self.pins):
            if down:
                self.keypad[k] = True
            else:
                self.keypadup[k] = True

    def glutkeydown(self, k, x, y):
        self.glutkey(k);

    def glutkeyup(self, k, x, y):
        self.glutkey(k, False)

    def glutkey(self, k, down=True):
        if k == 'q' or k == 27:
            exit(0)
        if k == ' ':
            key = keynames['auto']
        elif k == '\n':
            key = keynames['menu']
        elif k == '\t':
            key = keynames['select']
        else:
            key = ord(k) - ord('1')
        self.key(key, down)

    def glutspecialdown(self, k, x, y):
        self.glutspecial(k);

    def glutspecialup(self, k, x, y):
        self.glutspecial(k, False)

    def glutspecial(self, k, down=True):
        if k == glut.GLUT_KEY_UP:
            self.key(keynames['up'], down)
        elif k == glut.GLUT_KEY_DOWN:
            self.key(keynames['down'], down)
        elif k == glut.GLUT_KEY_LEFT:
            self.key(keynames['left'], down)
        elif k == glut.GLUT_KEY_RIGHT:
            self.key(keynames['right'], down)

    def idle(self):
        self.get('ap.heading')

        if any(self.keypadup):
            self.longsleep = 0

        if any(self.keypad):
            self.longsleep += 1
        else:
            self.longsleep += 10
        while self.longsleep > 20:
            dt = self.frameperiod / 10.0
            time.sleep(dt)
            self.longsleep -= 1

        # read from keys
        for pini in range(len(self.pins)):
            pin = self.pins[pini]
            value = True

            if False:
                f = open('/sys/class/gpio/gpio%d/value' % pin)
                a = f.readline()
                value = bool(int(a))
            else:
                if GPIO:
                    value = GPIO.input(pin)

            if not value and self.keypad[pini] > 0:
                self.keypad[pini] += 1
                
            if pini in self.keystate and self.keystate[pini] != value:
                if value:
                    self.keypadup[pini] = True
                else:
                    self.keypad[pini] = 1

            self.keystate[pini] = value

        if LIRC:
            if self.lirctime and time.time()- self.lirctime > .35:
                self.keypad[self.lirckey] = 0
                self.keypadup[self.lirckey] = True
                self.lirctime = False
                #print 'keypad', self.keypad, self.keypadup
                
            while True:
                code = LIRC.nextcode(1)
                if not code:
                    break
                repeat = code[0]['repeat']

                lirc_mapping = {'up': UP, 'down': DOWN, 'left': LEFT, 'right': RIGHT,
                                'power': AUTO, 'select': SELECT, 'mute': MENU, 'tab': MENU}
                code = code[0]['config']
                if not code in lirc_mapping:
                    continue
                pini = lirc_mapping[code]
                if not self.surface and (pini == MENU or pini == SELECT):
                    continue

                if repeat == 1: # ignore first repeat
                    #if self.lirctime:
                    #    self.keypad[self.lirckey] = self.keypadup[self.lirckey] = False
                    pass
                else:
                    if repeat == 0:
                        if self.lirctime:
                            self.keypad[self.lirckey] = 0
                            self.keypadup[self.lirckey] = True
                        self.keypad[pini] = 0
                    self.lirckey = pini;
                    self.keypad[pini] += 1
                    self.lirctime = time.time()

        self.process_keys()

        while True:
            result = False
            if not self.client:
                self.connect()
                break
            try:
                result = self.client.receive_single()
            except Exception as e:
                print 'disconnected', e
                self.client = False

            if not result:
                break

            name, data = result
            #print name, ' = ', data, 

            if 'value' in data:
                self.last_msg[name] = data['value']

            for token in ['min', 'max']:
                if token in data:
                    #print 'name', name, token, ' = ', data[token]
                    if not name in self.value_list:
                        self.value_list[name] = {}
                    self.value_list[name][token] = data[token]


def main():
    print 'init...'
    screen = None
    for arg in sys.argv:
        if 'nokia5110' in arg:
            sys.argv.remove(arg)
            print 'using nokia5110'
            screen = ugfx.nokia5110screen()

    if not screen:
        if use_glut:
            screen = glut.screen((120, 210))
            #screen = glut.screen((64, 128))
            #screen = glut.screen((48, 84))
        else:
            screen = ugfx.screen("/dev/fb0")
            if screen.width == 416 and screen.height == 656:
                # no actual device or display
                print 'no actual screen, running headless'
                screen = None

            if screen.width > 480:
                screen.width = 480
                screen.height= min(screen.height, 640)

    lcdclient = LCDClient(screen)
    if screen:
        # magnify to fill screen
        mag = min(screen.width / lcdclient.surface.width, screen.height / lcdclient.surface.height)
        if mag != 1:
            print "magnifying lcd surface to fit screen"
            magsurface = ugfx.surface(screen)

        invsurface = ugfx.surface(lcdclient.surface)
        
    def idle():
        if screen:
            lcdclient.display()

            surface = lcdclient.surface
            if lcdclient.config['invert']:
                invsurface.blit(surface, 0, 0)
                surface = invsurface
                surface.invert(0, 0, surface.width, surface.height)

            if mag != 1:
                magsurface.magnify(surface, mag)
                surface = magsurface
                #        mag = 2
                #surface.magnify(mag)

            screen.blit(surface, 0, 0, lcdclient.config['flip'])
            screen.refresh()

            if 'contrast' in lcdclient.config:
                screen.contrast = int(lcdclient.config['contrast'])

        lcdclient.idle()

    if use_glut:
        from OpenGL.GLUT import glutMainLoop
        from OpenGL.GLUT import glutIdleFunc
        from OpenGL.GLUT import glutKeyboardFunc
        from OpenGL.GLUT import glutKeyboardUpFunc
        from OpenGL.GLUT import glutSpecialFunc
        from OpenGL.GLUT import glutSpecialUpFunc

        glutKeyboardFunc(lcdclient.glutkeydown)
        glutKeyboardUpFunc(lcdclient.glutkeyup)
        glutSpecialFunc(lcdclient.glutspecialdown)
        glutSpecialUpFunc(lcdclient.glutspecialup)
        glutIdleFunc(idle)
#        glutIgnoreKeyRepeat(True)
        glutMainLoop()
    else:
        while True:
            idle()

if __name__ == '__main__':
    main()
