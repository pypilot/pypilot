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

try:
    import RPi.GPIO as GPIO
except ImportError:
    print 'No gpio available'
    GPIO = False

from signalk.client import SignalKClient

import ugfx
import font

def nr(x):
    try:
        s = str(int(x))
        while len(s) < 3:
            s = ' ' + s
        return s
    except:
        return x

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
    def __init__(self, name, desc, value, signalk_id, lcd, minval, maxval, step):
        self.name = name
        if type(desc) == type(''):
            self.desc = lambda : desc
        else:
            self.desc = desc
        self.value = value
        self.signalk_id = signalk_id
        self.range = minval, maxval, step
        self.lcd = lcd
        self.lastmovetime = 0
     
    def move(self, delta):
        v = self.value + delta*self.range[2]
        v = min(v, self.range[1])
        v = max(v, self.range[0])
        self.value = v
        self.lcd.client.set(self.signalk_id, v)
        self.lastmovetime = time.time()

    def display(self):
        self.lcd.surface.fill(black)
        self.lcd.fittext(rectangle(0, 0, 1, .3), self.name, True)
        self.lcd.fittext(rectangle(0, .3, 1, .3), self.desc(), True)

        # update name
        if time.time()-self.lastmovetime > 1:
            self.value = self.lcd.last_msg[self.signalk_id]
        
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
        self.lcd.client.get(self.signalk_id)

white = ugfx.color(255, 255, 255)
black = ugfx.color(0, 0, 0)

class rectangle():
    def __init__(self, x, y, width, height):
        self.x, self.y, self.width, self.height = x, y, width, height

AUTO, MENU, UP, DOWN, SELECT = range(5)

class LCDClient():
    def __init__(self, screen):
#        w, h, self.bw = 44, 84, 1
#        w, h, self.bw = 64, 128, 1
#        w, h, self.bw = 320, 480, 0

        self.config = {}
        self.configfilename = os.getenv('HOME') + '/.pypilot/lcdclient.conf' 
        try:
            file = open(self.configfilename)
            self.config = json.loads(file.readline())
        except:
            print 'failed to load config file:', self.configfilename
            self.config['invert'] = False
            self.config['language'] = 'en'

        if screen:
            w, h = screen.width, screen.height
            mul = int(math.ceil(w / 48.0))

            self.bw = 1 if w < 256 else False

            width = min(w, 48*mul)
            self.surface = ugfx.surface(width, width*h/w, screen.bypp, None)
            self.frameperiod = .2 # 5 frames a second possible

        else:
            self.surface = None
            self.frameperiod = 1

        self.set_language(self.config['language'])
        self.range_edit = False

        self.modes = {'compass': self.have_compass,
                      'gps':     self.have_gps,
                      'wind':    self.have_wind};        

        self.initial_gets = ['ap/P', 'ap/I', 'ap/D', 'servo/Max Current', 'servo/Min Speed', 'servo/Max Speed', 'imu/alignmentCounter']

        self.have_select = False
        self.create_mainmenu()

        self.last_gps_time = self.last_wind_time = 0

        self.longsleep = 30
        self.display_page = self.display_connecting
        self.connecting_dots = 0

        self.client = False

        self.keystate = {}
        self.keypad = [False, False, False, False, False, False]
        self.keypadup = list(self.keypad)

        self.blink = black, white
        self.control = False
        self.wifi = False

        self.pins = [26, 19, 13, 6, 5]
        if GPIO:
            GPIO.setmode(GPIO.BCM)
            for pin in self.pins:
                try:
                    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                except RuntimeError:
                    os.system("sudo chown tc /dev/gpiomem")
                    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                    
                GPIO.input(pin)

                def cbr(channel):
                    self.longsleep = 0

                GPIO.add_event_detect(pin, GPIO.BOTH, callback=cbr, bouncetime=20)

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
        def value_edit(name, desc, signalk_name, step, value=False):
            min = self.value_list[signalk_name]['min']
            max = self.value_list[signalk_name]['max']

            def thunk():
                self.range_edit=RangeEdit(name, desc, self.last_msg[signalk_name],
                                          signalk_name, self, min, max, step)
                return self.range_edit.display

            if value:
                return name, thunk, lambda : (self.last_msg[signalk_name]-min) / (max - min), signalk_name
            return name, thunk

        def gain():
            self.menu = LCDMenu(self, _('Gain'),
                                [value_edit('P', _('position gain'), 'ap/P', .0002, True),
                                 value_edit('I', _('integral gain'), 'ap/I', .0002, True),
                                 value_edit('D', _('rate gain'),     'ap/D', .0007, True)],
                                self.menu) 
            return self.display_menu

        def level():
            self.client.set('imu/alignmentCounter', 100)
            return self.display_page

        def calibrate():
            def getheading():
                self.client.get('imu/heading')
                try:
                    return '%.1f' % self.last_msg['imu/heading']
                except:
                    return str(self.last_msg['imu/heading'])

            self.menu = LCDMenu(self, _('Calibrate'),
                                [(_('level'), level),
                                 value_edit(_('heading'), getheading, 'imu/heading_offset', 1),
                                 (_('info'), lambda : self.display_calibrate_info)],
                                self.menu)
            self.menu.display_hook = self.display_calibrate
            return self.display_menu

        def settings():
            def mode():
                def set_mode(name):
                    def thunk():
                        self.client.set('ap/mode', name)
                        self.menu = self.menu.adam()
                        return self.display_control
                    return thunk
                modes = []
                index = 0
                selection = 0
                for mode in self.modes:
                    if self.modes[mode]():
                        modes.append((mode, set_mode(mode)))
                        if mode == self.last_msg['ap/mode']:
                            selection = index
                        index+=1
                self.menu = LCDMenu(self, _('Mode'), modes, self.menu)
                self.menu.selection = selection
            
                return self.display_menu

            def motor():
                self.menu = LCDMenu(self, _('Motor'),
                                    [value_edit(_('max current'), _('amps'), 'servo/Max Current', .25),
                                     value_edit(_('min speed'), _('relative'), 'servo/Min Speed', .05),
                                     value_edit(_('max speed'), _('relative'), 'servo/Max Speed', .05)],
                                    self.menu)
                return self.display_menu

            def language():
                def set_language(name):
                    def thunk():
                        self.set_language(name)
                        self.save_config()
                        self.create_mainmenu()
                        return self.display_menu
                    return thunk

                languages = [(_('English'), 'en'), (_('French'), 'fr'),
                             (_('Spanish'), 'es')]
                index, selection = 0, 0
                for language in languages:
                    if language[1] == self.config['language']:
                        selection = index
                    index+=1
                self.menu = LCDMenu(self, _('Language'), map(lambda language : (language[0], set_language(language[1])), languages), self.menu)
                self.menu.selection = selection
            
                return self.display_menu

            def invert():
                self.config['invert'] = not self.config['invert']
                self.save_config()
                return self.display_menu

            self.menu = LCDMenu(self, _('Settings'),
                                [(_('mode'), mode),
                                 (_('motor'), motor),
                                 (_('language'), language),
                                 (_('invert'), invert)],
                                self.menu)
            return self.display_menu

        self.menu = LCDMenu(self, _('Menu'),
                            [(_('gain'), gain),
                             (_('calibrate'), calibrate),
                             (_('settings'), settings),
                             (_('info'), lambda : self.display_info)])

    def text(self, pos, text, size, crop=False):
        pos = int(pos[0]*self.surface.width), int(pos[1]*self.surface.height)
        size = int(size*self.surface.width/48)
        font.draw(self.surface, pos, text, size, self.bw, crop)

    def fittext(self, rect, text, wordwrap=False, crop=False):
        metric_size = 16
        if wordwrap:
            words = text.split(' ')
            spacewidth = font.draw(self.surface, False, ' ', metric_size, self.bw, crop)[0]
            if len(words) < 2: # need at least 2 words to wrap
                return self.fittext(rect, text, False, crop)
            metrics = map(lambda word : (word, font.draw(self.surface, False, word, metric_size, self.bw, crop)), words)

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
                sw = self.surface.width * float(rect.width) / s[0]
                sh = self.surface.height * float(rect.height) / s[1]
                cursize = int(min(sw*metric_size, sh*metric_size))
                if cursize > size:
                    size = cursize
                    text = curtext
        else:
            s = font.draw(self.surface, False, text, metric_size, self.bw, crop)
            sw = self.surface.width * float(rect.width) / s[0]
            sh = self.surface.height * float(rect.height) / s[1]
            size = int(min(sw*metric_size, sh*metric_size))
#        size = max(size, 8)
        pos = int(rect.x*self.surface.width), int(rect.y*self.surface.height)
        size = font.draw(self.surface, pos, text, size, self.bw, crop)
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
        watchlist = ['ap/enabled', 'ap/mode', 'ap/heading_command',
                     'gps/track', 'wind/direction',
                     'servo/controller']
        nalist = watchlist + ['imu/pitch', 'imu/heel', 'imu/runtime',
                              'ap/P', 'ap/I', 'ap/D',
                              'imu/heading',
                              'imu/heading_offset',
                              'imu/alignmentCounter',
                              'imu/compass_calibration',
                              'imu/compass_calibration_age',
                              'servo/Amp Hours', 'servo/Max Current',
                              'servo/Min Speed', 'servo/Max Speed']
        self.last_msg = {}
        for name in nalist:
            self.last_msg[name] = _('N/A')
        self.last_msg['ap/heading_command'] = 0
        
        host = ""
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
                self.client.get(request)
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
        return time.time() - self.last_gps_time < 5

    def have_wind(self):
        return time.time() - self.last_wind_time < 5

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
            self.control = {'heading': False, 'heading_command': True, 'mode': False}
        
        def draw_big_number(pos, num, lastnum):
            num = nr(num)
            if len(num) < 3:
                print 'i', '"'+num+'"'

            if lastnum:
                lastnum = nr(lastnum)

            if self.surface.width < 256:
                size = 34
            else:
                size = 30

            def drawnum(ind, c):
                x = pos[0]+float(ind)/3
                self.surface.box(*(self.convrect(rectangle(x, pos[1], .34, .4)) + [black]))
                self.text((x, pos[1]), c, size, True)

            for i in range(3):
                try:
                    if num[i] == lastnum[i]:
                        continue
                except:
                    pass

                drawnum(i, num[i])

        warning = True
        mode = self.last_msg['ap/mode']
        if self.last_msg['servo/controller'] == 'none' and self.control['mode'] != 'no controller':
            self.fittext(rectangle(0, .5, 1, .4), _('WARNING no motor controller'), True)
        elif mode == 'gps' and not self.have_gps() and self.control['mode'] != 'no gps':
            self.fittext(rectangle(0, .55, 1, .3), _('WARNING GPS not detected'), True)
            self.control['mode'] = 'no gps'
        elif mode == 'wind' and not self.have_wind() and self.control['mode'] != 'no wind':
            self.fittext(rectangle(0, .55, 1, .3), _('WARNING WIND not detected'), True)
            self.control['mode'] = 'no wind'
        else:
            warning = False

        if type(self.last_msg['ap/heading']) == type(False):
            r = rectangle(0, 0, 1, .8)
            self.surface.box(*(self.convrect(r) + [black]))
            self.fittext(r, _('ERROR\ncompass or gyro failure!'), True)
            self.control['heading'] = False
            self.control['heading_command'] = False
        else:
            draw_big_number((0,0), self.last_msg['ap/heading'], self.control['heading'])
            self.control['heading'] = self.last_msg['ap/heading']

            if warning: #if self.last_msg['ap/mode'] != 'N/A':
                self.control['heading_command'] = False
            elif self.last_msg['ap/enabled'] != True:
                if self.control['heading_command'] != 'standby':
                    r = rectangle(0, .4, 1, .4)
                    self.surface.box(*(self.convrect(r) + [black]))
                    self.fittext(r, _('standby'))
                    self.control['heading_command'] = 'standby'
                    self.control['mode'] = False
            else:
                if self.control['heading_command'] != self.last_msg['ap/heading_command']:
                    draw_big_number((0,.4), self.last_msg['ap/heading_command'], self.control['heading_command'])
                    self.control['mode'] = False
                    self.control['heading_command'] = self.last_msg['ap/heading_command']

        def modes():
            return [self.have_compass(), self.have_gps(), self.have_wind()]
                    
        if warning:
            self.control['mode'] = False
        elif self.control['mode'] != mode or \
            self.control['modes'] != modes(): # mode ok
            self.control['mode'] = mode
            self.control['mode'] = modes()
            #print 'mode', self.last_msg['ap/mode']
            modes = {'compass': ('C', self.have_compass, rectangle(.03, .74, .30, .16)),
                     'gps':     ('G', self.have_gps,     rectangle(.34, .74, .30, .16)),
                     'wind':    ('W', self.have_wind,    rectangle(.65, .74, .30, .16))}

            self.surface.box(*(self.convrect(rectangle(0, .74, 1, .18)) + [black]))
            for mode in modes:
                if modes[mode][1]():
                    self.fittext(modes[mode][2], modes[mode][0])
                if self.last_msg['ap/mode'] == mode:
                    r = modes[mode][2]
                    marg = .02
                    self.rectangle(rectangle(r.x-marg, r.y+marg, r.width-marg, r.height), .015)

        self.display_wifi()

    def display_menu(self):
        self.surface.fill(black)
        self.menu.display()

    def display_calibrate(self):
        counter = self.last_msg['imu/alignmentCounter']
        if counter:
            r = rectangle(0, 0, 1, .25)
            self.surface.box(*(self.convrect(r) + [black]))
            r.height = .2
            self.fittext(r, ' %d%%' % (100-counter))
            r.width = 1-float(counter)/100
            r.height = .25
            self.invertrectangle(r)
        self.client.get('imu/alignmentCounter')
            
        self.fittext(rectangle(0, .72, .5, .14), _('pitch'))
        self.fittext(rectangle(.5, .72, .5, .14), self.round_last_msg('imu/pitch', 1))
        self.fittext(rectangle(0, .86, .5, .14), _('heel'))
        self.fittext(rectangle(.5, .86, .5, .14), self.round_last_msg('imu/heel', 1))
        self.client.get('imu/pitch')
        self.client.get('imu/heel')

    def display_connecting(self):
        self.surface.fill(black)
        self.fittext(rectangle(0, 0, 1, .4), _('connect to server...'), True)
        dots = ''
        for i in range(self.connecting_dots):
            dots += '.'
        self.text((0, .4), dots, 12)
        self.connecting_dots += 1
        if self.connecting_dots > 16:
            self.connecting_dots = 0

        self.display_wifi()
            
    def display_info(self):
        self.surface.fill(black)
        self.fittext(rectangle(0, 0, 1, .2), _('Info'))

        v = self.round_last_msg('servo/Amp Hours', 3)

        y = .2
        spacing = .11
        runtime = self.last_msg['imu/runtime'][:7]
        items = [_('Amp Hours'), v, _('runtime'), runtime, _('version'), '0.1']
        even, odd = 0, .05
        for item in items:
            self.fittext(rectangle(0, y, 1, spacing+even), item)
            y += spacing + even
            even, odd = odd, even

        self.client.get('servo/Amp Hours')
        self.client.get('imu/runtime')

    def display_calibrate_info(self):
        self.surface.fill(black)
        self.fittext(rectangle(0, 0, 1, .3), _('Calibrate Info'), True)
        
        deviation = _('N/A')
        try:
            ndeviation = self.last_msg['imu/compass_calibration'][1]
            names = [(100, _('excellent')), (200, _('good')), (300, _('poor')), (1000, _('bad'))]
            for n in names:
                if ndeviation < n[0]:
                    deviation = n[1]
                    break
        except:
            pass
        
        self.fittext(rectangle(0, .3, 1, .15), _('compass'))
        self.fittext(rectangle(.2, .45, .8, .2), deviation)
        self.fittext(rectangle(0, .65, .5, .15), _('age'))
        self.fittext(rectangle(.5, .65, .5, .15), self.last_msg['imu/compass_calibration_age'][:7])
            
        self.client.get('imu/compass_calibration')
        self.client.get('imu/compass_calibration_age')

    def display(self):
        self.display_page()

        if self.display_page != self.display_control:
            self.control = False
            self.wifi = False

        # status cursor
        w, h = self.surface.width, self.surface.height
        self.blink = self.blink[1], self.blink[0]
        size = h / 20
        self.surface.box(w-size-1, h-size-1, w-1, h-1, self.blink[0])

        if use_glut:
            from OpenGL.GLUT import glutPostRedisplay
            glutPostRedisplay()

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
            if self.last_msg['ap/enabled'] == False and self.display_page == self.display_control:
                self.set('ap/heading_command', self.last_msg['ap/heading'])
                self.set('ap/enabled', True)
            else:
                self.set('servo/command', 0) #stop
                self.set('ap/enabled', False)
        
            self.display_page = self.display_control

        if self.keypadup[SELECT]:
            if self.display_page == self.display_control:
                index = 0
                for mode in list(self.modes):
                    if mode == self.last_msg['ap/mode']:
                        break
                    index += 1

                tries = 0
                while tries < len(self.modes):
                    index += 1
                    if index == len(self.modes):
                        index = 0

                    if self.modes[list(self.modes)[index]]():
                        self.client.set('ap/mode', list(self.modes)[index])
                        break
                    tries += 1
            else:
                self.display_page = self.display_control

        # for up and down keys providing acceration
        updownheld = self.keypad[UP] > 10 or self.keypad[DOWN] > 10
        updownup = self.keypadup[UP] or self.keypadup[DOWN]
        sign = 1 if self.keypad[UP] or self.keypadup[UP] else -1
        speed = float(1 if updownup else min(20, .001*max(self.keypad[UP], self.keypad[DOWN])**3))
        updown = updownheld or updownup

        if self.display_page == self.display_control:
            if self.keypadup[MENU]: # MENU
                self.display_page = self.display_menu
            elif updown: # LEFT/RIGHT
                if self.last_msg['ap/enabled']:
                    cmd = self.last_msg['ap/heading_command'] + sign*speed
                    self.set('ap/heading_command', cmd)
                else:
                    self.set('servo/command', sign*(speed+8)/40)

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

        elif self.range_edit and self.display_page == self.range_edit.display:
            if self.keypadup[MENU]:
                self.display_page = self.display_menu
            elif updown:
                self.range_edit.move(sign*speed*.3)
        else:
            # self.display_page = self.display_control
            pass

        for key in range(6):
            if self.keypadup[key]:
                self.keypad[key] = self.keypadup[key] = False

    def glutkeydown(self, k, x, y):
        if k == 'q' or k == 27:
            exit(0)
        self.glutkey(k);
        
    def glutkeyup(self, k, x, y):
        self.glutkey(k, False)

    def glutkey(self, k, down=True):
        key = ord(k) - ord('1')
        if key >= 0 and key < len(self.pins):
            if down:
                self.keypad[key] = True
            else:
                self.keypadup[key] = True
            from OpenGL.GLUT import glutPostRedisplay
            glutPostRedisplay()

    def idle(self):
        if self.client:
            self.client.get('ap/heading')

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
            if not self.surface and (pini == MENU or pini == SELECT):
                continue
        
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

        self.process_keys()

        while True:
            result = False
            if self.client:
                result = self.client.receive_single()
            else:
                self.connect()

            if not result:
                break

            name, data = result
            #print name, ' = ', data

            if name in self.last_msg and self.last_msg[name] != 'N/A': # ignore initial message
                if name == 'gps/track' and 'value' in data:
                    self.last_gps_time = time.time()
                    print 'last msg', self.last_msg[name]
                if name == 'wind/direction' and 'value' in data:
                    self.last_wind_time = time.time()

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
    if use_glut:
        #screen = glut.screen((480, 640))
        screen = glut.screen((64, 128))
    else:
        screen = ugfx.screen("/dev/fb0")
        if screen.width == 416 and screen.height == 656:
            # no actual device or display
            print 'no actual screen, running headless'
            screen = None

    lcdclient = LCDClient(screen)
    print 'complete'

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

            screen.blit(surface, 0, 0)

        lcdclient.idle()

    if use_glut:
        from OpenGL.GLUT import glutMainLoop
        from OpenGL.GLUT import glutIdleFunc
        from OpenGL.GLUT import glutKeyboardFunc
        from OpenGL.GLUT import glutKeyboardUpFunc

        glutKeyboardFunc(lcdclient.glutkeydown)
        glutKeyboardUpFunc(lcdclient.glutkeyup)
        glutIdleFunc(idle)
#        glutIgnoreKeyRepeat(True)
        glutMainLoop()
    else:
        while True:
            idle()

if __name__ == '__main__':
    main()
