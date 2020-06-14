#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

white = 0xffffff
black = 0x00
#white = ugfx.color(255, 255, 255)
#black = ugfx.color(0, 0, 0)

AUTO, MENU, SMALL_PORT, SMALL_STARBOARD, SELECT, BIG_PORT, BIG_STARBOARD, TACK, NUM_KEYS = range(9)

class rectangle():
    def __init__(self, x, y, width, height):
        self.x, self.y, self.width, self.height = x, y, width, height

import time
import json
import font

translate = lambda x : x # initially no translation

def _(x):
    return translate(x)    

def set_language(lang):
    try:
        import gettext, os
        language = gettext.translation('pypilot_hat',
                                       os.path.abspath(os.path.dirname(__file__)) + '/locale',
                                       languages=[lang], fallback=True)
        global translate
        translate = language.gettext
    except Exception as e:
        print('no language', lang, e)

class page(object):
    def __init__(self, name=None, frameperiod=.25):
        self.name = name
        self.frameperiod = frameperiod
        self.watches = {}
        self.fittext_cache = {}

    def fill(self, color):
        self.lcd.surface.fill(color)
        
    def text(self, pos, text, size, crop=False):
        surface = self.lcd.surface
        pos = int(pos[0]*surface.width), int(pos[1]*surface.height)
        size = int(size*surface.width/48)
        size = font.draw(surface, pos, text, size, self.lcd.bw, crop)
        return float(size[0])/surface.width, float(size[1])/surface.height


    def fittextsizewordwrap(self, rect, text, metric_size, bw, surface):
        t0 = time.time()
        words = text.split(' ')
        if not words or not words[0]:
            return 0, ''

        spacewidth = font.draw(surface, False, ' ', metric_size, self.lcd.bw)[0]

        metrics = []
        for word in words:
            metrics.append((word, font.draw(surface, False, word, metric_size, bw)))
            self.lcd.client.receive()

        t1 = time.time()

        widths = list(map(lambda metric : metric[1][0], metrics))
        maxwordwidth = max(*widths+[0])
        totalwidth = sum(widths) + spacewidth * (len(words) - 1)
        t2 = time.time()

        size = 0

        # calculate where to wrap words to maximize font size
        wrappos = maxwordwidth
        while True:
            posx, posy = 0, 0
            curtext = ''
            lineheight = 0
            maxw = 0
            minfirstwidth = maxwordwidth
            for metric in metrics:
                word, (width, height) = metric
                if posx > 0:
                    width += spacewidth
                if posx + width > wrappos:
                    curtext += '\n'
                    posx = 0
                    posy += lineheight
                    if width < minfirstwidth:
                        minfirstwidth = width;
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
            sw = surface.width * float(rect.width) / s[0]
            sh = surface.height * float(rect.height) / s[1]
            cursize = int(min(sw*metric_size, sh*metric_size))
            if cursize < size:
                break
            
            size = cursize
            text = curtext
            if posy == 0:
                break

            wrappos += minfirstwidth

        t3 = time.time()

        return size, text
        
    
    def fittext(self, rect, text, wordwrap=False, fill='none'):
        surface = self.lcd.surface
        bw = self.lcd.bw
        #print('fittext', text, wordwrap, fill)
        if fill != 'none':
            surface.box(*(self.convrect(rect) + [fill]))
        metric_size = 16
        ptext = text
        if text in self.fittext_cache:
            size, r, ptext = self.fittext_cache[text]
            if r.width != rect.width or r.height != rect.height:
                del self.fittext_cache[text]

        if not text in self.fittext_cache:
            if wordwrap:
                size, ptext = self.fittextsizewordwrap(rect, text, metric_size, bw, surface)
            else:
                s = font.draw(surface, False, text, metric_size, bw)
                if s[0] == 0 or s[1] == 0:
                    return 0, 0
                sw = surface.width * float(rect.width) / s[0]
                sh = surface.height * float(rect.height) / s[1]
                size = int(min(sw*metric_size, sh*metric_size))
            try:
                self.lcd.client.reset_timeout()
            except:
                pass
            #time.sleep(.02)  # this line is required!  needed to process wifi packets durning long sleep
            if wordwrap: # only cache wordwrap fit!!
                self.fittext_cache[text] = size, rect, ptext

        pos = int(rect.x*surface.width), int(rect.y*surface.height)

        size = font.draw(surface, pos, ptext, size, bw)
        return float(size[0])/surface.width, float(size[1])/surface.height

    def line(self, x1, y1, x2, y2):
        surface = self.lcd.surface
        w, h = surface.width - 1, surface.height - 1
        surface.line(int(x1*w), int(y1*h), int(x2*w+.5), int(y2*h+.5), white)

    def convbox(self, x1, y1, x2, y2):
        if min(x1, y1, x2, y2) < 0 or max(x1, y2, x2, y2) > 1:
            print('invalid box!', x1, y1, x2, y2)
            raise 1
            return [0, 0, 0, 0]

        surface = self.lcd.surface
        w, h = surface.width - 1, surface.height - 1
        return [int(x1*w), int(y1*h), int(x2*w), int(y2*h)]

    def invertrectangle(self, rect):
        self.lcd.surface.invert(*self.convbox(rect.x, rect.y, rect.x+rect.width, rect.y+rect.height))

    def convrect(self, rect):
        return self.convbox(rect.x, rect.y, rect.x+rect.width, rect.y+rect.height)

    def rectangle(self, rect, width = False):
        surface = self.lcd.surface
        if not width:
            surface.box(*(self.convrect(rect) + [white]))
        else:
            box = self.convrect(rect)
            surface.invert(*box)
            if width:
                w, h = surface.width - 1, surface.height - 1
                px_width = int(max(1, min(w*width, h*width)))
                surface.invert(box[0]+px_width, box[1]+px_width, box[2]-px_width, box[3]-px_width)

    def box(self, rect, color):
        surface = self.lcd.surface
        surface.box(*(self.convrect(rect) + [color]))
        
                
    def last_val(self, name, period=-1, default='N/A'):
        if period is -1:
            period = self.frameperiod
        self.watches[name] = period
        if name in self.lcd.last_msg:
            return self.lcd.last_msg[name]
        return default

    def round_last_val(self, name, places):
        v = self.last_val(name)
        try:
            n = 10**places
            return str(round(v*n)/n)
        except:
            return v

    def testkeydown(self, key):
        if self.lcd.keypad[key]==1:
            self.lcd.keypad[key]=2
            return True
        return False

    def testkeyup(self, key):
        return self.lcd.keypadup[key]

    def speed_of_keys(self):
        # for up and down keys providing acceration
        keypad, keypadup = self.lcd.keypad, self.lcd.keypadup
        down = keypad[SMALL_STARBOARD] or keypadup[SMALL_STARBOARD]
        up = keypad[SMALL_PORT] or keypadup[SMALL_PORT]
        left = keypad[BIG_PORT] or keypadup[BIG_PORT]
        right = keypad[BIG_STARBOARD] or keypadup[BIG_STARBOARD]
        updownup = keypadup[BIG_PORT] or keypadup[BIG_STARBOARD]
        updownheld = keypad[BIG_PORT] > 10 or keypad[BIG_STARBOARD] > 10
        speed = float(1 if updownup else min(10, .004*max(keypad[BIG_PORT], keypad[BIG_STARBOARD])**2.5))
        updown = updownheld or updownup
        if keypadup[SMALL_PORT] or keypadup[SMALL_STARBOARD]:
            updown = True
            speed = 10
        if down or left:
            sign = -1
        elif up or right:
            sign = 1
        else:
            sign = 0
        return sign * speed

    def set(self, name, value):
        self.lcd.client.set(name, value)

    def display(self, refresh):
        pass # some pages only perform an action
        
    def process(self):
        if self.testkeydown(AUTO):
            return control(self.lcd)
        if self.testkeydown(MENU):
            return self.lcd.getmenu()
        if self.testkeydown(SELECT):
            if self.prev:
                return self.prev
            return control(self.lcd)

class info(page):
    def __init__(self, num_pages=4):
        super(info, self).__init__('info')
        self.num_pages = num_pages
        self.page = 0

    def bound_page(self):
        if self.page >= self.num_pages:
            self.page = 0
        elif self.page < 0:
            self.page = self.num_pages-1
        
    def display(self, refresh):
        self.bound_page()
        self.watches = {} # clear watches so they can be page specific
        self.fill(black)
        self.fittext(rectangle(0, 0, 1, .2), _('Info'))

        y = .2
        if self.page == 0:
            spacing = .11
            v = self.round_last_val('servo.watts', 3)
            runtime = self.last_val('ap.runtime')[:7]
            ah = self.round_last_val('servo.amp_hours', 3)
            items = [_('Watts'), v, _('Amp Hours'), ah, _('runtime'), runtime]
        elif self.page == 1:
            spacing = .11
            v = self.round_last_val('servo.voltage', 3)
            rate = self.round_last_val('imu.loopfreq', 2)
            uptime = self.last_val('imu.uptime')[:7]
            items = [_('voltage'), v, _('rate'), rate, _('uptime'), uptime]
        elif self.page == 2:
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

    def process(self):
        if self.testkeyup(SMALL_PORT) or self.testkeyup(BIG_STARBOARD):
            self.page += 1
        if self.testkeyup(SMALL_STARBOARD) or self.testkeyup(BIG_PORT):
            self.page -= 1
        return super(info, self).process()
        
class calibrate_info(info):
    def __init__(self):
        super(calibrate_info, self).__init__(3)

    def display(self, refresh):
        self.bound_page()
        self.fill(black)
        self.fittext(rectangle(0, 0, 1, .24), _('Calibrate Info'), True)

        if self.page == 0:
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

        elif self.page == 1:
            try:
                cal = self.last_val('imu.compass.calibration')
                raw = ''
                for c in cal[0]:
                    raw += '%.1f\n' % c
            except:
                raw = 'N/A'                    

            self.fittext(rectangle(0, .3, 1, .7), raw)
        else:
            import math
            mod = int(time.time()%11)/3
            self.fittext(rectangle(0, .24, 1, .15), 'sigma plot')
            cal = self.last_val('imu.compass.calibration')[0]
            if len(cal) >= 5:
                m = cal[3]
                dip = math.radians(cal[4])
            else:
                m, dip = 0, 0 # not connected

            if mod == 1:
                m *= math.cos(dip)
            try:
                from pypilot import quaternion
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

def test_wifi():
    try:
        import micropython
        import wifi_esp32
        return wifi_esp32.connected
    except:
        pass
        
    try:
        wlan0 = open('/sys/class/net/wlan0/operstate')
        line = wlan0.readline().rstrip()
        wlan0.close()
        if line == 'up':
            return True
    except:
        pass
    return False

class controlbase(page):
    def __init__(self, lcd):
        super(controlbase, self).__init__()
        self.lcd = lcd
        self.batt = False
        self.wifi = False

    def display(self, refresh):
        if refresh:
            self.box(rectangle(0, .9, 1, .1), black)
            self.wifi = False
            
        battrect = rectangle(0.03, .93, .25, .06)
        
        if self.lcd.battery_voltage:
            batt = min(max((self.lcd.battery_voltage-3)/.7, 0), 1)
            if batt != self.batt or refresh:
                self.batt = batt
                self.lcd.surface.box(*(self.convrect(battrect) + [black]))
                self.rectangle(battrect, width=0.015)
                self.rectangle(rectangle(0.28, .95, .03, .02))
                if batt:
                    battrect = rectangle(.06, .95, .19*float(batt), .02)
                    self.box(battrect, white)
        
        wifi = test_wifi()
        if self.wifi == wifi and not refresh:
            return # done displaying
        self.wifi = wifi
        wifirect = rectangle(.35, .9, .6, .1)
        if wifi:
            text = 'WIFI'
            if self.lcd.hat and self.lcd.hat.config['remote']:
                text += ' R'
            self.fittext(wifirect, text)
        else:
            self.lcd.surface.box(*(self.convrect(wifirect) + [black]))

class control(controlbase):
    def __init__(self, lcd):
        super(control, self).__init__(lcd)
        self.modes_list = ['compass', 'gps', 'wind', 'true wind'] # in order
        self.control = {} # used to keep track of what is drawn on screen to avoid redrawing it

    def have_compass(self):
        return True

    def have_gps(self):
        return self.last_val('gps.source') != 'none'

    def have_wind(self):
        return self.last_val('wind.source') != 'none'

    def have_true_wind(self):
        return self.have_gps() and self.have_wind()
    
    def display_mode(self):
        mode = self.last_val('ap.mode')
        modes = [self.have_compass(), self.have_gps(), self.have_wind(), self.have_true_wind()]
        if self.control['mode'] == mode and self.control['modes'] == modes:
            return # no need to refresh
        self.control['mode'] = mode
        self.control['modes'] = modes

        #print('mode', self.last_val('ap.mode'))
        modes = {'compass': ('C', self.have_compass, rectangle(0, .74, .25, .16)),
                 'gps':     ('G', self.have_gps,     rectangle(.25, .74, .25, .16)),
                 'wind':    ('W', self.have_wind,    rectangle(.5, .74, .25, .16)),
                 'true wind': ('T', self.have_true_wind, rectangle(.75, .74, .25, .16))}

        self.lcd.surface.box(*(self.convrect(rectangle(0, .74, 1, .18)) + [black]))
        for mode in modes:
            if modes[mode][1]():
                self.fittext(modes[mode][2], modes[mode][0])
            if self.last_val('ap.mode') == mode:
                r = modes[mode][2]
                marg = .02
                self.rectangle(rectangle(r.x, r.y+marg, r.width-marg, r.height), .015)

    def display(self, refresh):        
        if not self.control:
            self.fill(black)
            self.control = {'heading': '   ', 'heading_command': '   ', 'mode': False, 'modes': []}
        
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

            if self.lcd.surface.width < 120:
                size = 34
            else:
                size = 32

            for i in range(3):
                try:
                    if num[i] == lastnum[i]:
                        continue
                except:
                    pass
                x = pos[0]+float(i)*.33
                self.box(rectangle(x, pos[1], .33, .4), black)
                self.text((x, pos[1]), num[i], size, True)

        if self.last_val('imu.loopfreq', 1) is False:
            r = rectangle(0, 0, 1, .92)
            self.fittext(r, _('ERROR\ncompass or gyro failure!'), True, black)
            self.control['heading'] = 'no imu'
            self.control['heading_command'] = 'no imu'
            return
        
        draw_big_number((0,0), self.last_val('ap.heading'), self.control['heading'])
        self.control['heading'] = self.last_val('ap.heading')
        #print('heading', self.last_val('ap.heading'))

        mode = self.last_val('ap.mode')

        # display warning about any servo faults
        flags = self.last_val('servo.flags').split()
        warning = ''
        for flag in flags:
            if flag.endswith('_FAULT'):
                warning += flag[:-6] + ' '

        if warning:
            if self.hat:
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
            self.display_mode()
        super(control, self).display(refresh)

    def process(self):
        if not self.lcd.client.connection:
            return connecting(self.lcd)

        if self.testkeyup(AUTO): # AUTO
            if self.last_val('ap.enabled') == False:
                self.set('ap.heading_command', self.last_val('ap.heading'))
                self.set('ap.enabled', True)
            else:
                self.set('servo.command', 0) # stop
                self.set('ap.enabled', False)
        if self.testkeydown(SELECT):
            have_mode = {'compass': self.have_compass, 'gps': self.have_gps,
                          'wind': self.have_wind, 'true wind': self.have_true_wind}
            # change mode
            for t in range(len(self.modes_list)):
                self.modes_list = self.modes_list[1:] + [self.modes_list[0]]
                next_mode = self.modes_list[0]
                if next_mode != self.last_val('ap.mode') and have_mode[next_mode]():
                    self.set('ap.mode', next_mode)
                    return
                    
        speed = self.speed_of_keys()
        if not speed:
            return super(control, self).process()
        
        if self.last_val('ap.enabled'):
            if self.lcd.keypadup[SMALL_PORT] or self.lcd.keypadup[SMALL_STARBOARD]:
                speed = self.config['bigstep']
            else:
                speed = self.config['smallstep']                        
                cmd = self.last_val('ap.heading_command') + sign*speed
            self.set('ap.heading_command', cmd)
        else: # manual control
            sign = -1 if speed < 0 else 1
            self.set('servo.command', speed*sign*8.0/20)

class connecting(controlbase):
    def __init__(self, lcd):
        super(connecting, self).__init__(lcd)
        self.connecting_dots = 0
        
    def display(self, refresh):
        if refresh:
            self.box(rectangle(0, 0, 1, .4), black)
            self.fittext(rectangle(0, 0, 1, .4), _('connect to server'), True)
            self.drawn_text = True
        self.box(rectangle(0, .4, 1, .5), black)
        dots = ''
        for i in range(self.connecting_dots):
            dots += '.'
        size = self.text((0, .4), dots, 12)
        self.connecting_dots += 1
        if size[0] >= 1 or self.connecting_dots > 20:
            self.connecting_dots = 0
        super(connecting, self).display(refresh)

    def process(self):
        if self.lcd.client.connection:
            return control(self.lcd)
        if self.testkeydown(MENU):
            return self.lcd.getmenu()
