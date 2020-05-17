#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from pypilot.client import pypilotClient
import sys, os, time, math

import gettext
import json
_ = lambda x : x # initially no translation
    
from pypilot import quaternion

from pypilot.hat.ugfx import ugfx
import font

class LCDMenu():
    def __init__(self, lcd, name, items, prev=False):
        self.lcd = lcd
        self.selection = 0
        self.name = name
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
        fit = self.lcd.fittext(rectangle(0, 0, 1, .25), self.name)
        sy = y = fit[1] + .03
        items = min(int((1 - y)/.15), len(self.items))
        scroll = max(self.selection - int(items/2), 0)
        scroll = min(scroll, len(self.items) - items)
        for item in self.items[scroll:]:
            size = self.lcd.fittext(rectangle(0, y, 1, .15), item[0])[0] + .25
            if len(item) > 2: # more than just a text item
                val = item[2]()
                if type(val) == type(False): # check box
                    if val: # draw if value is true
                        self.lcd.invertrectangle(rectangle(.8, y+.07, .1, .07))
                else: # slider, draw box showing value
                    sliderarea = rectangle(size, y+.05, (1-size), .07)
                    self.lcd.rectangle(sliderarea, .015)
                    sliderarea.width *= val
                    self.lcd.rectangle(sliderarea)
            y += .15
            if y >= 1:
                break

        y = .15*(self.selection-scroll) + sy
        self.lcd.invertrectangle(rectangle(0, y+.03, 1, .12))
        if self.display_hook:
            self.display_hook()

class RangeEdit():
    def __init__(self, name, desc, id, pypilot, lcd, minval, maxval, step):
        self.name = name
        if type(desc) == type('') or type(desc) == type(u''):
            self.desc = lambda : desc
        else:
            self.desc = desc
        self.id = id
        self.pypilot = pypilot
        self.range = minval, maxval, step
        self.lcd = lcd
        self.value = lcd.last_val(id) if pypilot else lcd.config[id]
        self.lastmovetime = 0
     
    def move(self, delta):
        if self.pypilot: #config items rounded to integer
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
        if self.pypilot:
            self.lcd.set(self.id, v)
        else:
            self.lcd.config[self.id] = v
        self.lastmovetime = time.monotonic()

    def display(self):
        self.lcd.surface.fill(black)
        self.lcd.fittext(rectangle(0, 0, 1, .3), self.name, True)
        self.lcd.fittext(rectangle(0, .3, 1, .3), self.desc(), True)

        # update name
        if time.monotonic()-self.lastmovetime > 1:
            if self.pypilot:
                self.value = self.lcd.last_val(self.id)
        
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


white = ugfx.color(255, 255, 255)
black = ugfx.color(0, 0, 0)

class rectangle():
    def __init__(self, x, y, width, height):
        self.x, self.y, self.width, self.height = x, y, width, height

AUTO, MENU, UP, DOWN, SELECT, LEFT, RIGHT = range(7)

class LCD():
    def __init__(self, hat):
        self.hat = hat

        self.config = hat.config['lcd']
        default = {'contrast': 60, 'invert': False, 'backlight': 200,
                   'flip': False, 'language': 'en', 'bigstep': 10,
                   'smallstep': 1};

        for name in default:
            if not name in self.config:
                self.config[name] = default[name]

        # set the driver to the one from hat eeprom
        driver = 'default'
        if self.hat.hatconfig:
            driver = self.hat.hatconfig['lcd']['driver']
            
        for pdriver in ['nokia5110', 'jlx12864', 'glut', 'framebuffer', 'none']:
            if pdriver in sys.argv:
                sys.argv.remove(pdriver)
                driver = pdriver
                break
            
        print('using lcd driver', driver)

        use_glut = 'DISPLAY' in os.environ
        self.use_glut = False
        if driver == 'none':
            screen = None
        elif driver == 'nokia5110' or (driver == 'default' and not use_glut):
            screen = ugfx.spiscreen(0)
        elif driver == 'jlx12864':
            screen = ugfx.spiscreen(1)
        elif driver == 'glut' or (driver == 'default' and use_glut):
            self.use_glut = True
            print('using glut')
            import glut
            #screen = glut.screen((120, 210))
            screen = glut.screen((64, 128))
            #screen = glut.screen((48, 84))
            #screen = glut.screen((96, 168))
            
            from OpenGL.GLUT import glutKeyboardFunc, glutKeyboardUpFunc
            from OpenGL.GLUT import glutSpecialFunc, glutSpecialUpFunc
            
            glutKeyboardFunc(self.glutkeydown)
            glutKeyboardUpFunc(self.glutkeyup)
            glutSpecialFunc(self.glutspecialdown)
            glutSpecialUpFunc(self.glutspecialup)
#        glutIgnoreKeyRepeat(True)
        elif driver == 'framebuffer':
            print('using framebuffer')
            screen = ugfx.screen("/dev/fb0")
            if screen.width > 480:
                screen.width = 480
                screen.height= min(screen.height, 640)
            
        if screen:
            w, h = screen.width, screen.height
            mul = int(math.ceil(w / 48.0))

            self.bw = 1 if w < 256 else False

            width = min(w, 48*mul)
            self.surface = ugfx.surface(width, int(width*h/w), screen.bypp, None)

            # magnify to fill screen
            self.mag = min(screen.width / self.surface.width, screen.height / self.surface.height)
            if self.mag != 1:
                print('magnifying lcd surface to fit screen')
                self.magsurface = ugfx.surface(screen)

            self.invsurface = ugfx.surface(self.surface)
            
            self.frameperiod = .25 # 4 frames a second possible
            self.lastframetime = 0
        else:
            self.surface = None
            self.frameperiod = 1

        self.screen = screen
        
        self.set_language(self.config['language'])
        self.range_edit = False

        self.modes = {'compass': self.have_compass,
                      'gps':     self.have_gps,
                      'wind':    self.have_wind,
                      'true wind': self.have_true_wind};

        self.modes_list = ['compass', 'gps', 'wind', 'true wind'] # in order
        self.watchlist = {'ap.pilot': True}

        self.create_mainmenu()

        self.display_page = self.display_control
        self.connecting_dots = 0

        self.keypad = [False, False, False, False, False, False, False, False]
        self.keypadup = list(self.keypad)

        self.blink = black, white
        self.control = False # used to keep track of what is drawn on screen to avoid redrawing it
        self.wifi = False

        self.client = False
        self.watches = {}
        self.connect()

    def connect(self):
        self.last_msg = {}
        self.last_msg['gps.source'] = 'none'
        self.last_msg['wind.source'] = 'none'
        
        if self.client:
            self.client.disconnect()

        self.client = pypilotClient(self.hat.client.config['host'])
        ret = self.client.list_values()

    def set_language(self, name):
        try:
            language = gettext.translation('pypilot_hat',
                                           os.path.abspath(os.path.dirname(__file__)) + '/locale',
                                           languages=[name], fallback=True)
            global _
            _ = language.ugettext
        except Exception as e:
            print('no languages', e)
        self.config['language'] = name

    def write_config(self):
        self.hat.write_config

    def value_list(self):
        v = self.client.values.value
        if v:
            return v
        return {}
            
    def create_mainmenu(self):
        def value_edit(name, desc, pypilot_name, value=False):
            min = self.value_list()[pypilot_name]['min']
            max = self.value_list()[pypilot_name]['max']
            step = (max-min)/100.0

            def thunk():
                self.range_edit = RangeEdit(name, desc, pypilot_name,
                                            True, self, min, max, step)
                return self.range_edit.display

            if value:
                def last_val_num(name):
                    ret = self.last_val(name)
                    if ret == 'N/A':
                        return 0
                    return ret

                return name, thunk, lambda : (last_val_num(pypilot_name)-min) / (max - min), pypilot_name
            return name, thunk

        def value_check(name, pypilot_name):
            def thunk():
                self.set(pypilot_name, not self.last_val(pypilot_name))
                return self.display_menu
            return name, thunk, lambda : self.last_val(pypilot_name), pypilot_name

        def config_edit(name, desc, config_name, min, max, step):
            def thunk():
                self.range_edit = RangeEdit(name, desc, config_name,
                                          False, self, min, max, step)
                return self.range_edit.display
            return name, thunk

        def pilot():
            try:
                pilots = self.value_list()['ap.pilot']['choices']
            except:
                pilots = []

            def set_pilot(name):
                def thunk():
                    self.set('ap.pilot', name)
                    self.menu = self.menu.adam()
                    return self.display_menu
                return thunk
                
            self.menu = LCDMenu(self, _('Pilot'), list(map(lambda name : (name, set_pilot(name)), self.value_list()['ap.pilot']['choices'])), self.menu)
            index = 0
            for pilot in pilots:
                if pilot == self.last_val('ap.pilot'):
                    self.menu.selection = index
                index+=1

            return self.display_menu

        def curgains():
            ret = []
            for name, value in self.value_list().items():
                if 'AutopilotGain' in value:
                    if 'ap.pilot.' in name:
                        s = name.split('.')
                        if self.last_val('ap.pilot') == s[2]:
                            ret.append(name)
                    else:
                        ret.append(name)

            ret.sort() # sort of get PID in order (reverse alphabet)
            ret.reverse()
            return ret
        
        def gain():
            def gain_edit(gain):
                n = gain[gain.rfind('.')+1:]
                return value_edit(n, n, gain, True)
            
            self.menu = LCDMenu(self, _('Gain'),
                                [(_('pilot'), pilot)] +
                                list(map(gain_edit, curgains())),
                                 self.menu)
            return self.display_menu

        def level():
            self.set('imu.alignmentCounter', 100)
            return self.display_page

        def calibrate_rudder_feedback():
            options = []
            rudder = self.last_val('rudder')
            if rudder != 'N/A' and rudder and \
               'rudder.calibration_state' in self.value_list():
                options = self.value_list()['rudder.calibration_state']['choices']
                options.remove('idle')

            self.menu = LCDMenu(self, _('Rudder') + '\n' + _('Feedback'),
                                list(map(lambda option : (option, lambda : self.set('rudder.calibration_state', option)), options)), self.menu)

            def display_rudder():
                fit = self.fittext(rectangle(0, .5, 1, .25), str(self.last_val('rudder.angle')))
                self.watch('rudder.angle')

            self.menu.display_hook = display_rudder
            return self.display_menu
        
        def calibrate():
            def getheading():
                try:
                    return '%.1f' % self.last_val('imu.heading')
                except:
                    return str(self.last_val('imu.heading'))

            self.menu = LCDMenu(self, _('Calibrate'),
                                [(_('level'), level),
                                 value_edit(_('heading'), getheading, 'imu.heading_offset'),
                                 value_check(_('lock'), 'imu.compass.calibration.locked'),
                                 (_('rudder'), calibrate_rudder_feedback),
                                 (_('info'), lambda : self.display_calibrate_info)],
                                self.menu)
            self.menu.display_hook = self.display_calibrate
            return self.display_menu
        
        def settings():
            def mode():
                def set_mode(name):
                    def thunk():
                        self.set('ap.mode', name)
                        self.menu = self.menu.adam()
                        return self.display_control
                    return thunk
                modes = []
                index = 0
                selection = 0
                for mode in self.modes:
                    if self.modes[mode]():
                        modes.append((mode, set_mode(mode)))
                        if mode == self.last_val('ap.mode'):
                            selection = index
                        index+=1
                self.menu = LCDMenu(self, _('Mode'), modes, self.menu)
                self.menu.selection = selection
            
                return self.display_menu

            def motor():
                self.menu = LCDMenu(self, _('Motor'),
                                    [value_edit(_('min speed'), _('relative'), 'servo.speed.min'),
                                     value_edit(_('max speed'), _('relative'), 'servo.speed.max'),
                                     value_edit(_('max current'), _('amps'), 'servo.max_current'),
                                     value_edit(_('period'), _('seconds'), 'servo.period')],
                                    self.menu)
                return self.display_menu
            
            def wifi():
                self.wifi = True
                if not self.wifi:
                    def display_no_wifi():
                        self.surface.fill(black)
                        self.fittext(rectangle(0, 0, 1, 1), _('No Wifi detected'), True)
                    return display_no_wifi

                networking = '/home/tc/.pypilot/networking.txt'
                default = {'mode': 'Master', 'ssid': 'pypilot', 'key':'', 'client_ssid': 'pypilot', 'client_key': ''} # defaults
                self.wifi_settings = default
                try:
                    f = open(networking, 'r')
                    while True:
                        l = f.readline()
                        if not l:
                            break
                        for setting in self.wifi_settings:
                            if l.startswith(setting+'='):
                                self.wifi_settings[setting] = l[len(setting)+1:].strip()
                    f.close()
                except:
                    pass

                def update():
                    try:
                        f = open(networking, 'w')
                        for setting in self.wifi_settings:
                            f.write(setting+'='+self.wifi_settings[setting]+'\n')
                        f.close()
                    except Exception as e:
                        print('exception writing', networking, ':', e)
                    os.system('/opt/networking.sh')

                def select_wifi_ap_toggle():
                    def thunk():
                        ap = self.wifi_settings['mode'] == 'Master'
                        self.wifi_settings['mode'] = 'Managed' if ap else 'Master'
                        update()
                        return self.display_menu
                    return [thunk, lambda : self.wifi_settings['mode'] == 'Master']

                def select_wifi_defaults():
                    self.wifi_settings = default
                    update()
                    return self.display_menu

                def wifi_remote():
                    def thunk():
                        self.hat.config['remote'] = not self.hat.config['remote']
                        self.hat.write_config()
                        self.hat.connect()
                        self.connect()
                        return self.display_menu
                    return [thunk, lambda : self.hat.config['remote']]

                self.menu = LCDMenu(self, _('WIFI'),
                                    [['AP'] + select_wifi_ap_toggle(),
                                     [_('defaults'), select_wifi_defaults],
                                     [_('remote')] + wifi_remote()], self.menu)
                return self.display_menu


            def control():
                self.menu = LCDMenu(self, _('Control'),
                                    [(_('wifi'), wifi),
                                     config_edit(_('small step'), _('degrees'), 'smallstep', 1, 5, 1),
                                     config_edit(_('big step'), _('degrees'), 'bigstep', 5, 20, 5)],
                                    self.menu)
                return self.display_menu

            def invert():
                self.config['invert'] = not self.config['invert']
                self.write_config()
                return self.display_menu

            def backlight():
                self.config['backlight'] = not self.config['backlight']
                self.write_config()
                return self.display_menu

            def flip():
                self.config['flip'] = not self.config['flip']
                self.write_config()
                return self.display_menu

            def display():
                self.menu = LCDMenu(self, _('Display'),
                                    [config_edit(_('contrast'), '', 'contrast', 30, 90, 1),
                                     (_('invert'), invert),
                                     config_edit(_('backlight'), '', 'backlight', 0, 200, 1),
                                     (_('flip'), flip)],
                                    self.menu)
                return self.display_menu

            def language():
                def set_language(name):
                    def thunk():
                        self.set_language(name)
                        self.write_config()
                        self.create_mainmenu()
                        return language()
                    return thunk

                languages = [(_('English'), 'en'),
                             (_('French'), 'fr'),
                             (_('Spanish'), 'es')]
                index, selection = 0, 0
                for lang in languages:
                    if lang[1] == self.config['language']:
                        selection = index
                    index += 1
                self.menu = LCDMenu(self, _('Language'), list(map(lambda lang : (lang[0], set_language(lang[1])), languages)), self.menu)
                self.menu.selection = selection
            
                return self.display_menu
            
            self.menu = LCDMenu(self, _('Settings'),
                                [(_('mode'), mode),
                                 (_('motor'), motor),
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
        #print('fittext', text, wordwrap, fill)
        if fill != 'none':
            self.surface.box(*(self.convrect(rect) + [fill]))
        metric_size = 16
        if wordwrap:
            words = text.split(' ')
            spacewidth = font.draw(self.surface, False, ' ', metric_size, self.bw)[0]
            if len(words) < 2: # need at least 2 words to wrap
                return self.fittext(rect, text, False, fill)
            metrics = list(map(lambda word : (word, font.draw(self.surface, False, word, metric_size, self.bw)), words))

            widths = list(map(lambda metric : metric[1][0], metrics))
            maxwordwidth = max(*widths)
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
        w, h = self.surface.width - 1, self.surface.height - 1
        self.surface.line(int(x1*w), int(y1*h), int(x2*w+.5), int(y2*h+.5), white)

    def convbox(self, x1, y1, x2, y2):
        w, h = self.surface.width - 1, self.surface.height - 1
        return [int(x1*w), int(y1*h), int(x2*w), int(y2*h)]

    def invertrectangle(self, rect):
        self.surface.invert(*self.convbox(rect.x, rect.y, rect.x+rect.width, rect.y+rect.height))
    def convrect(self, rect):
        return self.convbox(rect.x, rect.y, rect.x+rect.width, rect.y+rect.height)

    def rectangle(self, rect, width = False):
        if not width:
            self.surface.box(*(self.convrect(rect) + [white]))
        else:
            box = self.convrect(rect)
            self.surface.invert(*box)
            if width:
                w, h = self.surface.width - 1, self.surface.height - 1
                px_width = int(max(1, min(w*width, h*width)))
                self.surface.invert(box[0]+px_width, box[1]+px_width, box[2]-px_width, box[3]-px_width)
        
    def last_val(self, name, period=-1):
        if period is -1:
            period = self.frameperiod
        self.watches[name] = period
        if name in self.last_msg:
            return self.last_msg[name]
        return 'N/A'

    def round_last_val(self, name, places):
        n = 10**places
        try:
            v = self.hat.last_msg[name]
        except Exception as e:
            v = self.last_val(name)

        try:
            return str(round(v*n)/n)
        except:
            return v
            
    def have_compass(self):
        return True

    def have_gps(self):
        return self.last_val('gps.source') != 'none'

    def have_wind(self):
        return self.last_val('wind.source') != 'none'

    def have_true_wind(self):
        return self.have_gps() and self.have_wind()

    def display_mode(self, control):
        def modes():
            return [self.have_compass(), self.have_gps(), self.have_wind(), self.have_true_wind()]

        mode = self.last_val('ap.mode')
        if control:
            if control['mode'] == mode and control['modes'] == modes():
                return # no need to refresh
            control['mode'] = mode
            control['modes'] = modes()

        #print('mode', self.last_val('ap.mode'))
        modes = {'compass': ('C', self.have_compass, rectangle(0, .74, .25, .16)),
                 'gps':     ('G', self.have_gps,     rectangle(.25, .74, .25, .16)),
                 'wind':    ('W', self.have_wind,    rectangle(.5, .74, .25, .16)),
                 'true wind': ('T', self.have_true_wind, rectangle(.75, .74, .25, .16))}

        self.surface.box(*(self.convrect(rectangle(0, .74, 1, .18)) + [black]))
        for mode in modes:
            if modes[mode][1]():
                self.fittext(modes[mode][2], modes[mode][0])
            if self.last_val('ap.mode') == mode:
                r = modes[mode][2]
                marg = .02
                self.rectangle(rectangle(r.x-marg, r.y+marg, r.width-marg, r.height), .015)
    
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
                text = 'WIFI'
                if self.hat.config['remote']:
                    text += ' R'
                self.fittext(wifirect, text)
            else:
                self.surface.box(*(self.convrect(wifirect) + [black]))
            self.wifi = wifi
    
    def display_control(self):
        if not self.client.connection:
            self.display_connecting()
            self.control = False
            return

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

        if self.last_val('imu.loopfreq', 1) == 0:
            r = rectangle(0, 0, 1, .92)
            self.fittext(r, _('ERROR\ncompass or gyro failure!'), True, black)
            self.control['heading'] = 'no imu'
            self.control['heading_command'] = 'no imu'
            return

        draw_big_number((0,0), self.last_val('ap.heading'), self.control['heading'])
        self.control['heading'] = self.last_val('ap.heading')

        mode = self.last_val('ap.mode')

        # display warning about any servo faults
        flags = self.last_val('servo.flags').split()
        warning = ''
        for flag in flags:
            if flag.endswith('_FAULT'):
                warning += flag[:-6] + ' '

        if warning:
            self.hat.buzzer.alarm()
            warning = warning.lower()
            warning += 'fault'
            if self.control['heading_command'] != warning:
                self.fittext(rectangle(0, .4, 1, .35), _(warning), True, black)
                self.control['heading_command'] = warning
        elif mode == 'gps' and not self.have_gps():
            if self.control['heading_command'] != 'no gps':
                self.fittext(rectangle(0, .4, 1, .4), _('GPS not detected'), True, black)
                self.control['heading_command'] = 'no gps'
        elif (mode == 'wind' or mode == 'true wind') and not self.have_wind():
            if self.control['heading_command'] != 'no wind':
                self.fittext(rectangle(0, .4, 1, .4), _('WIND not detected'), True, black)
                self.control['heading_command'] = 'no wind'
        elif self.last_val('servo.controller') == 'none':
            if self.control['heading_command'] != 'no controller':
                self.fittext(rectangle(0, .4, 1, .35), _('WARNING no motor controller'), True, black)
                self.control['heading_command'] = 'no controller'
        else:
            # no warning, display the desired course or 'standby'
            if self.last_val('ap.enabled') != True:
                if self.control['heading_command'] != 'standby':
                    r = rectangle(0, .4, 1, .34)
                    self.fittext(r, _('standby'), False, black)
                    self.control['heading_command'] = 'standby'
            else:
                if self.control['heading_command'] != self.last_val('ap.heading_command'):
                    draw_big_number((0,.4), self.last_val('ap.heading_command'), self.control['heading_command'])
                    self.control['heading_command'] = self.last_val('ap.heading_command')

                    self.control['mode'] = False # refresh mode

        warning = False
        if mode == 'compass':
            warning = False
            cal = self.last_val('imu.compass.calibration')
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

        if not warning:
            self.display_mode(self.control)
        self.display_wifi()

    def display_menu(self):
        self.surface.fill(black)
        self.menu.display()

    def display_calibrate(self):
        counter = self.last_val('imu.alignmentCounter')
        if counter:
            r = rectangle(0, 0, 1, .15)
            r.height = .2
            self.fittext(r, ' %d%%' % (100-counter), False, black)
            r.width = 1-float(counter)/100
            r.height = .25
            self.invertrectangle(r)
            
        self.fittext(rectangle(0, .86, .5, .14), self.round_last_val('imu.pitch', 1))
        self.fittext(rectangle(.5, .86, .5, .14), self.round_last_val('imu.heel', 1))

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

        if self.info_page > 3:
            self.info_page = 0
        elif self.info_page < 0:
            self.info_page = 3

        y = .2
        if self.info_page == 0:
            spacing = .11
            v = self.round_last_val('servo.watts', 3)
            runtime = self.last_val('ap.runtime')[:7]
            ah = self.round_last_val('servo.amp_hours', 3)
            items = [_('Watts'), v, _('Amp Hours'), ah, _('runtime'), runtime]
        elif self.info_page == 1:
            spacing = .11
            v = self.round_last_val('servo.voltage', 3)
            rate = self.round_last_val('imu.loopfreq', 2)
            uptime = self.last_val('imu.uptime')[:7]
            items = [_('voltage'), v, _('rate'), rate, _('uptime'), uptime]
        elif self.info_page == 2:
            spacing = .11
            ct = self.round_last_val('servo.controller_temp', 2)
            mt = self.round_last_val('servo.motor_temp', 2)
            faults = self.round_last_val('servo.faults', 0)
            items = [_('cont temp'), ct, _('motor temp'), mt, _('faults'), faults]
        else:
            spacing = .18
            ver = self.last_val('ap.version')

            items = [_('version'), ver, _('author'), "Sean D'Epagnier"]

        even, odd = 0, .05
        for item in items:
            self.fittext(rectangle(0, y, 1, spacing+even), item, True)
            y += spacing + even
            even, odd = odd, even

    def display_calibrate_info(self):
        if self.info_page > 2:
            self.info_page = 0
        elif self.info_page < 0:
            self.info_page = 2
        
        self.surface.fill(black)
        self.fittext(rectangle(0, 0, 1, .24), _('Calibrate Info'), True)

        if self.info_page == 0:
            deviation = [_('N/A'), _('N/A')]
            deviationstr = _('N/A')
            dim = '?'
            try:
                cal = self.last_val('imu.compass.calibration')
                deviation = ['%.2f' % cal[1][0], '%.2f' % cal[1][1]]
                dim = str(int(cal[2]))
                #print(ndeviation)
                names = [(0, _('incomplete')), (.01, _('excellent')), (.02, _('good')),
                         (.04, _('fair')), (.06, _('poor')), (1000, _('bad'))]
                for n in names:
                    if cal[1][0] <= n[0]:
                        deviationstr = n[1]
                        break
            except:
                pass
            
            self.fittext(rectangle(0, .3, 1, .15), _('compass'))
            self.fittext(rectangle(0, .42, 1, .23), deviationstr)
            self.fittext(rectangle(0, .66, 1, .14), deviation[0] + ' ' + dim + 'd')
            self.fittext(rectangle(0, .8, 1, .2), self.last_val('imu.compass.calibration.age')[:7])

        elif self.info_page == 1:
            try:
                cal = self.last_val('imu.compass.calibration')
                raw = ''
                for c in cal[0]:
                    raw += '%.1f\n' % c
            except:
                raw = 'N/A'                    

            self.fittext(rectangle(0, .3, 1, .7), raw)
        else:
            mod = int(time.monotonic()%11)/3
            self.fittext(rectangle(0, .24, 1, .15), 'sigma plot')
            cal = self.last_val('imu.compass.calibration')[0]
            m = cal[3]
            dip = math.radians(cal[4])
            if mod == 1:
                m *= math.cos(dip)
            try:
                p = self.last_val('imu.compass.calibration.sigmapoints')
                q = self.last_val('imu.alignmentQ')
                p = map(lambda p0 : map(lambda x0, c : (x0 - c) / m, p0[:3], cal[:3]), p)
                x, y, r = 24, 56, 20
                if mod > 1:
                    if mod == 3:
                        x1, y1 = int(r*math.cos(dip)), int(r*math.sin(dip))
                        self.surface.line(x, y, x + x1, y + y1, white)
                        self.surface.line(x, y, x - x1, y + y1, white)
                    q = quaternion.multiply(q, quaternion.angvec2quat(math.radians(90), [0, 1, 0]))
                p = map(lambda p0 : quaternion.rotvecquat(p0, q), p)
                for p0 in p:
                    self.surface.putpixel(int(r*p0[0]+x), int(r*p0[1]+y), white)
            except:
                self.fittext(rectangle(0, .3, 1, .7), 'N/A')

    def display(self):
        self.display_page()

        if self.display_page != self.display_control:
            self.control = False
            self.wifi = False

        # status cursor
        t0 = time.monotonic()
        try:
            if t0-self.blinktime > .5:
                self.blink = self.blink[1], self.blink[0]
                self.blinktime = t0
        except:
            self.blinktime = 0
        w, h = self.surface.width, self.surface.height
        size = h // 40
        self.surface.box(w-size-1, h-size-1, w-1, h-1, self.blink[0])

    def set(self, name, value):
        self.client.set(name, value)

    def menu_back(self):
        if self.menu.prev:
            self.menu = self.menu.prev
            return self.display_menu
        return self.display_control
            
    def process_keys(self):
        def testkeydown(key):
            if self.keypad[key]==1:
                self.keypad[key]=2
                return True
            return False

        if self.keypadup[AUTO]: # AUTO
            if self.last_val('ap.enabled') == False and self.display_page == self.display_control:
                self.set('ap.heading_command', self.last_val('ap.heading'))
                self.set('ap.enabled', True)
            else:
                self.set('servo.command', 0) # stop
                self.set('ap.enabled', False)
        
            self.display_page = self.display_control
            
        if testkeydown(SELECT):
            if self.display_page == self.display_control and self.surface:
                # change mode
                for t in range(len(self.modes_list)):
                    #self.modes_list = [self.modes_list[-1]] + self.modes_list[:-1]
                    self.modes_list = self.modes_list[1:] + [self.modes_list[0]]
                    next_mode = self.modes_list[0]
                    if next_mode != self.last_val('ap.mode') and \
                       self.modes[next_mode]():
                        self.set('ap.mode', next_mode)
                        break
            else:
                self.menu = self.menu.adam() # reset to main menu
                self.display_page = self.display_control

        # for up and down keys providing acceration
        down = self.keypad[DOWN] or self.keypadup[DOWN]
        right = self.keypad[RIGHT] or self.keypadup[RIGHT]
        updownup = self.keypadup[LEFT] or self.keypadup[RIGHT]
        updownheld = self.keypad[LEFT] > 10 or self.keypad[RIGHT] > 10
        speed = float(1 if updownup else min(10, .004*max(self.keypad[LEFT], self.keypad[RIGHT])**2.5))
        updown = updownheld or updownup
        if self.keypadup[UP] or self.keypadup[DOWN]:
            updown = True
            speed = 10

        if self.display_page == self.display_control:                
            if testkeydown(MENU) and self.surface: # MENU
                self.display_page = self.display_menu
            elif updown: # LEFT/RIGHT/UP/DOWN
                sign = -1 if down or right else 1                
                if self.last_val('ap.enabled'):
                    if self.keypadup[UP] or self.keypadup[DOWN]:
                        speed = self.config['bigstep']
                    else:
                        speed = self.config['smallstep']                        
                    cmd = self.last_val('ap.heading_command') + sign*speed
                    self.set('ap.heading_command', cmd)
                else:
                    self.set('servo.command', sign*(speed+8.0)/20)

        elif self.display_page == self.display_menu:
            if testkeydown(UP) or testkeydown(LEFT):
                self.menu.selection -= 1
                if self.menu.selection < 0:
                    self.menu.selection = len(self.menu.items)-1
            elif testkeydown(DOWN) or testkeydown(RIGHT):
                self.menu.selection += 1
                if self.menu.selection == len(self.menu.items):
                    self.menu.selection = 0
            elif testkeydown(MENU):
                self.display_page = self.menu.items[self.menu.selection][1]()

        elif self.display_page == self.display_info or \
             self.display_page == self.display_calibrate_info:
            if testkeydown(MENU):
                self.display_page = self.display_menu
            if self.keypadup[UP] or self.keypadup[RIGHT]:
                self.info_page += 1
            if self.keypadup[DOWN] or self.keypadup[LEFT]:
                self.info_page -= 1

        elif self.range_edit and self.display_page == self.range_edit.display:
            if testkeydown(MENU):
                self.display_page = self.display_menu
                if not self.range_edit.pypilot:
                    self.write_config()
            elif updown:
                sign = -1 if down or not right else 1
                self.range_edit.move(sign*speed*.1)

        elif self.display_page == self.display_connecting:
            pass # no keys handled for this page
        else:
            print('unknown display page', self.display_page)

        for key in range(len(self.keypad)):
            if self.keypadup[key]:
                self.keypad[key] = self.keypadup[key] = False
                self.hat.buzzer.beep()

    def key(self, k, down):
        if k >= 0 and k < len(self.keypad):
            if down:
                self.keypad[k] = True
            else:
                self.keypadup[k] = True

    def glutkeydown(self, k, x, y):
        self.glutkey(k);

    def glutkeyup(self, k, x, y):
        self.glutkey(k, False)

    def glutkey(self, k, down=True):
        k = k.decode()
        if k == 'q' or ord(k) == 27:
            exit(0)
        if k == ' ':
            key = AUTO
        elif k == '\n':
            key = MENU
        elif k == '\t':
            key = SELECT
        else:
            key = ord(k) - ord('1')
        self.key(key, down)

    def glutspecialdown(self, k, x, y):
        self.glutspecial(k);

    def glutspecialup(self, k, x, y):
        self.glutspecial(k, False)

    def glutspecial(self, k, down=True):
        from OpenGL import GLUT as glut
        if k == glut.GLUT_KEY_UP:
            self.key(UP, down)
        elif k == glut.GLUT_KEY_DOWN:
            self.key(DOWN, down)
        elif k == glut.GLUT_KEY_LEFT:
            self.key(LEFT, down)
        elif k == glut.GLUT_KEY_RIGHT:
            self.key(RIGHT, down)

    def draw(self):
        self.display()

        surface = self.surface
        if self.config['invert']:
            self.invsurface.blit(surface, 0, 0)
            surface = self.invsurface
            surface.invert(0, 0, surface.width, surface.height)

        if self.mag != 1:
            self.magsurface.magnify(surface, self.mag)
            surface = magsurface

        self.screen.blit(surface, 0, 0, self.config['flip'])
        self.screen.refresh()

        if 'contrast' in self.config:
            self.screen.contrast = int(self.config['contrast'])

        if 'backlight' in self.config:
            self.hat.arduino.set_backlight(int(self.config['backlight']))

    def poll(self):
        if self.screen:
            t = time.monotonic()
            dt = t - self.lastframetime
            if dt > self.frameperiod:
                self.draw()
                self.lastframetime = max(self.lastframetime+self.frameperiod,
                                         t-self.frameperiod)

                for name in list(self.client.watches):
                    if name != 'values' and not name in list(self.watches):
                        self.client.watch(name, False)
                for name, period in self.watches.items():
                    self.client.watch(name, period)

        msgs = self.client.receive()
        for name, value in msgs.items():
            self.last_msg[name] = value
                
        self.watches = self.watchlist
        self.process_keys()
