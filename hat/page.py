#!/usr/bin/env python3
#
#   Copyright (C) 2021 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time
import font

white = 0xffffff
black = 0x00
#white = ugfx.color(255, 255, 255)
#black = ugfx.color(0, 0, 0)

BIG_PORT, SMALL_PORT, SMALL_STARBOARD, BIG_STARBOARD, AUTO, MENU, MODE, NUM_KEYS = range(8)

class rectangle():
    def __init__(self, x, y, width, height):
        self.x, self.y, self.width, self.height = x, y, width, height
        
translate = lambda x : x # initially no translation
no_translation = translate

def _(x):
    return translate(x)

locale_d = ''

try:
    import micropython
    import wifi_esp32
    def test_wifi():
        return wifi_esp32.connected[0]
    def gettime():
        return time.ticks_ms()/1e3

    try:
        import gettext_esp32 as gettext
    except:
        print('failed to import gettext')
except:
    def gettime():
        return time.monotonic()
    def test_wifi():
        try:
            wlan0 = open('/sys/class/net/wlan0/operstate')
            line = wlan0.readline().rstrip()
            wlan0.close()
            if line == 'up':
                return True
        except:
            pass
        return False

    try:
        import os, gettext
        locale_d= os.path.abspath(os.path.dirname(__file__)) + '/'
    except Exception as e:
        print('failed to import gettext', e)
        
def set_language(lang):
    print('set language', lang)
    try:
        language = gettext.translation('pypilot_hat',
                                       locale_d + 'locale',
                                       languages=[lang], fallback=True)
        global translate
        translate = language.gettext
    except Exception as e:
        print('no language', lang, e)

class page(object):
    def __init__(self, name=None, frameperiod=.4):
        self.name = name
        self.frameperiod = frameperiod
        self.watches = {}
        self.fittext_cache = []

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
            # needed in micropython because this routine can be so slow the recieve buffer can overflow
            self.lcd.receive()

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
                    
            if maxw == 0 or maxh == 0: # failed to render anything
                return 0, ''
            sw = surface.width * float(rect.width) / maxw
            sh = surface.height * float(rect.height) / maxh
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
        spaces = ' ' in text
        # numbers have same width
        if spaces:
            ntext = text
        else:
            ntext = ''
            for c in text:
                if c.isdigit():
                    ntext += '0'
                else:
                    ntext += c

        for t in self.fittext_cache:
            if t[0] == ntext:
                t0, size, r, ptext = t
                self.fittext_cache.remove(t)
                if r.width == rect.width and r.height == rect.height:
                    self.fittext_cache.append(t)
                    break
        else:
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
                #print('fittext cache len', ntext, len(self.fittext_cache), list(map(lambda x : x[0], self.fittext_cache)))
                self.fittext_cache.append((ntext, size, rect, ptext if spaces else None))
                if len(self.fittext_cache) > 8:
                    self.fittext_cache = self.fittext_cache[1:]

        pos = int(rect.x*surface.width), int(rect.y*surface.height)
        if not ptext:
            ptext = text
        size = font.draw(surface, pos, ptext, size, bw)
        return float(size[0])/surface.width, float(size[1])/surface.height

    def line(self, x1, y1, x2, y2):
        surface = self.lcd.surface
        w, h = surface.width - 1, surface.height - 1
        surface.line(int(x1*w), int(y1*h), int(x2*w+.5), int(y2*h+.5), white)

    def convbox(self, x1, y1, x2, y2):
        def bound(x):
            return min(max(x, 0), 1)
        x1 = bound(x1)
        y1 = bound(y1)
        x2 = bound(x2)
        y2 = bound(y2)

        surface = self.lcd.surface
        w, h = surface.width - 1, surface.height - 1
        return [int(round(x1*w)), int(round(y1*h)), int(round(x2*w)), int(round(y2*h))]

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
        if period == -1:
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
        k = self.lcd.keypad[key]
        if k.down:
            self.lcd.buzz_key()
            k.down -= 1
            return True
        return False

    def testkeyup(self, key):
        k = self.lcd.keypad[key]
        if k.up:
            k.up = False
            return True
        return False

    def speed_of_keys(self):
        # for keys providing acceration
        keypad = self.lcd.keypad
        ss = keypad[SMALL_STARBOARD].dt()*10
        sp = keypad[SMALL_PORT].dt()*10
        bp = keypad[BIG_PORT].dt()*10
        bs = keypad[BIG_STARBOARD].dt()*10

        speed = 0;
        sign = 0;
        if sp or ss:
            speed = max(.4, min(.8, .003*max(sp, ss)**1.8))
        if bp or bs:
            speed = max(.4, min(1, .006*max(bp, bs)**2.5))

        if ss or bs:
            sign = -1
        elif sp or bp:
            sign = 1

        return sign * speed

    def set(self, name, value):
        #print('set', name, value, time.monotonic())
        self.lcd.client.set(name, value)
        self.lcd.client.poll() # reduce lag

    def display(self, refresh):
        pass # some pages only perform an action
        
    def process(self):
        if self.testkeydown(AUTO):
            return control(self.lcd)
        if self.testkeydown(MENU):
            return self.lcd.getmenu()
        if self.testkeydown(MODE):
            if self.prev:
                return self.prev
            return control(self.lcd)

class info(page):
    def __init__(self, num_pages=4):
        super(info, self).__init__(_('info'))
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
            rate = self.round_last_val('imu.frequency', 2)
            uptime = self.last_val('imu.uptime')[:7]
            items = [_('voltage'), v, _('rate'), rate, _('uptime'), uptime]
        elif self.page == 2:
            spacing = .11
            ct = self.round_last_val('servo.controller_temp', 2)
            mt = self.round_last_val('servo.motor_temp', 2)
            items = [_('cont temp'), ct, _('motor temp'), mt]
            if self.lcd.battery_voltage:
                items += [_('battery'), '%.3f' % self.lcd.battery_voltage]
            else:
                items += [_('faults'), self.round_last_val('servo.faults', 0)]
        else:
            spacing = .18
            ver = self.last_val('ap.version')

            items = [_('version'), ver, _('author'), "Sean D'Epagnier"]

        even, odd = 0, .05
        for item in items:
            self.fittext(rectangle(0, y, 1, spacing+even), item, False)
            y += spacing + even
            even, odd = odd, even

    def process(self):
        if self.testkeydown(SMALL_PORT) or self.testkeydown(BIG_STARBOARD):
            self.page += 1
        if self.testkeydown(SMALL_STARBOARD) or self.testkeydown(BIG_PORT):
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
            deviation = ['N/A', 'N/A']
            deviationstr = 'N/A'
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

class controlbase(page):
    def __init__(self, lcd, frameperiod = .4):
        super(controlbase, self).__init__(frameperiod = frameperiod)
        self.lcd = lcd
        self.batt = False
        self.wifi = False
        self.pilot = False
        self.profile = False

        self.charging_blink = False
        self.charging_blink_time = 0

    def display(self, refresh):
        if refresh:
            self.box(rectangle(0, .92, 1, .1), black)
            self.profile = ''
            self.pilot = False
            self.wifi = False
            
        if self.lcd.battery_voltage:
            battrect = rectangle(0.03, .93, .25, .06)
            batt = min(max(self.lcd.battery_voltage - 3.2, 0), 1)
            if batt != self.batt or refresh:
                self.batt = batt
                self.lcd.surface.box(*(self.convrect(battrect) + [black]))
                self.rectangle(battrect, width=0.015)
                self.rectangle(rectangle(0.28, .95, .03, .02))
                if batt and not self.charging_blink:
                    battrect = rectangle(.06, .95, .19*float(batt), .02)
                    self.box(battrect, white)
            if self.lcd.battery_voltage > 4.5:
                t = gettime()
                if t - self.charging_blink_time > 1:
                    self.charging_blink_time = t
                    self.charging_blink = not self.charging_blink
                    self.batt = 0
            else:
                self.charging_blink = False

        profile = str(self.last_val('profile'))
        if self.profile != profile:
            self.profile = profile
            profilerect = rectangle(0, .92, .32, .09)
            self.lcd.surface.box(*(self.convrect(profilerect) + [black]))
            self.fittext(profilerect, 'P'+profile[:2])

        '''
        pilot = self.last_val('ap.pilot')
        if self.pilot != pilot:
            self.pilot = pilot
            pilotrect = rectangle(.2, .92, .5, .09)
            self.fittext(pilotrect, pilot[:6])
        '''
                
        wifi = test_wifi()
        if self.wifi == wifi and not refresh:
            return # done displaying
        self.wifi = wifi
        wifirect = rectangle(.4, .92, .35, .1)
        if wifi:
            text = 'WIFI'
            if self.lcd.host != 'localhost':
                text += 'R'
            self.fittext(wifirect, text)
        else:
            self.lcd.surface.box(*(self.convrect(wifirect) + [black]))

class control(controlbase):
    def __init__(self, lcd):
        super(control, self).__init__(lcd, .25)
        self.control = {} # used to keep track of what is drawn on screen to avoid redrawing it
        self.lastspeed = 0
        self.lasttime = 0
        self.ap_heading_command_time = gettime()-5
        self.ap_heading_command = 0
        self.resetmanualkeystate()
        self.tack_hint = 0, ''

    def get_ap_heading_command(self):
        if gettime() - self.ap_heading_command_time < 5:
            return self.ap_heading_command
        return self.last_val('ap.heading_command', default='   ')

    def set_ap_heading_command(self, command):
        if 'wind' in self.control['mode']:
            d = -180
        else:
            d = 0
        while command < d:
            command += 360
        while command >= d+360:
            command -= 360
        self.set('ap.heading_command', command)
        self.ap_heading_command = command
        self.ap_heading_command_time = gettime()
        
    def resetmanualkeystate(self, k=0):
        self.manualkeystate = {'key': k,
                               'command': self.get_ap_heading_command(),
                               'change': 0}

    def display_mode(self):
        mode = self.last_val('ap.mode')
        modes = self.last_val('ap.modes', default=[])
        if not mode in modes:
            return
        index = modes.index(mode)
        nmodes = len(modes)

        # flash the compass C if there are imu warnings
        if self.last_val('imu.warning', default=False):
            if int(time.monotonic()) % 2:
                modes = list(modes)
                for i in range(nmodes):
                    if modes[i] == 'compass':
                        modes[i] = ' '

        if self.control['mode'] == mode and self.control['modes'] == modes:
            return # no need to refresh
        self.control['mode'] = mode
        self.control['modes'] = modes
        if index < 2:
            rindex = index
            mindex = 0
        elif nmodes > 3 and index == nmodes-1:
            rindex = 3
            mindex = nmodes-4
        else:
            rindex = 2
            mindex = index - 2
        
        #print('mode', self.last_val('ap.mode'))
        modes_r = [rectangle(  0, .74, .22, .16), rectangle(.22, .74, .25, .16),
                   rectangle(.47, .74, .3,  .16), rectangle(.77, .74, .23, .16)]

        marg = 0.02
        self.lcd.surface.box(*(self.convrect(rectangle(0, .74, 1, .16+marg)) + [black]))

        marg = 0.02
        for i in range(4):
            ind = mindex+i
            if ind < nmodes:
                ret=self.fittext(modes_r[i], modes[ind][0].upper())

        # draw rectangle around mode
        r = modes_r[rindex]
        self.rectangle(rectangle(r.x, r.y+marg, r.width, r.height), .015)

    def display(self, refresh):
        if not self.control:
            self.control = {'heading': False,
                            'heading_command': False,
                            'mode': False, 'modes': []}
        def nr(x):
            try:
                s = str(int(round(abs(x))))
                while len(s) < 3:
                    s = ' ' + s
                return s
            except:
                return x

        def draw_big_number(pos, num, lastnum):
            if num == 'N/A' and lastnum != num:
                r = rectangle(0, pos, 1, .4)
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
                x = float(i)*.33
                self.box(rectangle(x, pos, .34, .4), black)
                self.text((x, pos), num[i], size, True)
                
        def draw_heading(pos, value, lastvalue):
            heading, mode, num = value
            try:
                lastheading, lastmode, lastnum = lastvalue
            except:
                lastmode = False #refresh

            windmode = 'wind' in mode

            if mode != lastmode:
                lastnum = 'XXX' # redraw if mode changes
            elif windmode and lastheading != 'N/A' and \
               heading*lastheading <= 0:
                lastnum = 'XXX' # redraw if sign changes
                
            draw_big_number(pos, num, lastnum)
            # in wind mode draw indicator showing sign
            if windmode:
                if heading > 0:
                    self.box(rectangle(.7, pos+.3, .3, .025), white)
                elif heading < 0:
                    self.box(rectangle(0, pos+.3, .3, .025), white)

        frequency = self.last_val('imu.frequency', 1, None)
        if frequency is False:
            r = rectangle(0, 0, 1, .8)
            self.fittext(r, _('ERROR') + '\n' + _('compass or gyro failure!'), True, black)
            self.control['heading'] = 'no imu'
            self.control['heading_command'] = 'no imu'
            super(control, self).display(refresh)
            return

        t0 = gettime()
        mode = self.last_val('ap.mode')
        modes = self.last_val('ap.modes')
        ap_heading = self.last_val('ap.heading', default='   ')
        ap_heading_command = self.get_ap_heading_command()
        heading = ap_heading, mode, nr(ap_heading)
        if self.control['heading'] and heading == self.control['heading'] and \
           self.control['heading_command'] == ap_heading_command:
            if t0 - self.lasttime < .8 and not refresh:
                return True # optimization to not redraw frame if heading hasn't changed
        self.lasttime = t0
        draw_heading(0, heading, self.control['heading'])
        self.control['heading'] = heading

        # display warning about any servo faults
        flags = self.last_val('servo.flags').split()
        warning = ''
        buzz = False
        for flag in flags:
            if flag.endswith('_FAULT'):
                warning += flag[:-6].replace('_', ' ') + ' '
                if 'OVER' in flag or 'BADVOLTAGE' in flag: # beep overtemp/overcurrent
                    buzz = True
                pulse = 2

        if warning:
            if buzz:
                self.lcd.buzz(pulse, .1)
            warning = warning.lower()
            warning += 'fault'
            if self.control['heading_command'] != warning:
                self.fittext(rectangle(0, .4, 1, .5), _(warning), True, black)
                self.control['heading_command'] = warning
                self.control['mode'] = warning
        elif mode not in modes:
            no_mode = 'no ' + mode
            if self.control['heading_command'] != no_mode:
                self.fittext(rectangle(0, .4, 1, .35), mode.upper() + ' ' + _('not detected'), True, black)
                self.control['heading_command'] = no_mode
        elif self.last_val('imu.error', default=None):
            if self.control['heading_command'] != 'imu error':
                self.fittext(rectangle(0, .4, 1, .35), self.last_val('imu.error'), True, black)
                self.control['heading_command'] = 'imu error'
        elif self.last_val('servo.controller') == 'none':
            if self.control['heading_command'] != 'no controller':
                self.fittext(rectangle(0, .4, 1, .35), _('WARNING no motor controller'), True, black)
                self.control['heading_command'] = 'no controller'
        elif self.lcd.check_voltage():
            msg = self.lcd.check_voltage()
            if self.control['heading_command'] != msg:
                self.fittext(rectangle(0, .4, 1, .35), msg, True, black)
                self.control['heading_command'] = msg
        elif self.last_val('ap.enabled') != True and \
             self.last_val('servo.controller', default=None):
            # no warning, display the desired course or 'standby'
            if self.control['heading_command'] != 'standby':
                r = rectangle(0, .4, 1, .35)
                self.fittext(r, _('standby'), False, black)
                self.control['heading_command'] = 'standby'
        elif self.last_val('ap.tack.state', default='none') != 'none':
            r = rectangle(0, .4, 1, .35)
            d = self.last_val('ap.tack.direction')
            if self.last_val('ap.tack.state') == 'waiting':
                msg = _('tack') + ' ' + str(self.last_val('ap.tack.timeout'))
            else:
                msg = _('tacking') + ' ' + d[0].upper()
            if self.control['heading_command'] != msg:
                self.fittext(r, msg, False, black)
                self.control['heading_command'] = msg
        elif self.control['heading_command'] != ap_heading_command:
            heading_command = ap_heading_command, mode, nr(ap_heading_command)
            draw_heading(.4, heading_command, self.control['heading_command'])
            self.control['heading_command'] = ap_heading_command
            self.control['mode'] = False # refresh mode

        #warning = False
        if mode == 'compass':
            cal = self.last_val('imu.compass.calibration')
            if cal == 'N/A':
                ndeviation = 0
            else:
                ndeviation = cal[1][0]
            def warncal(s):
                r = rectangle(0, .75, 1, .15)
                self.fittext(r, s, True, white)
                self.invertrectangle(r)
                self.control['mode'] = 'warning' # redraw mode when warning is gone
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

        if self.testkeydown(AUTO): # AUTO
            self.lcd.reset_keys()

            if self.last_val('ap.enabled') == False:
                self.set_ap_heading_command(self.last_val('ap.heading'))
                self.set('ap.enabled', True)
            else:
                self.set('servo.command', 0) # stop
                self.set('ap.enabled', False)


        if self.testkeydown(MODE):
            # change mode
            modes = self.last_val('ap.modes')
            mode = self.last_val('ap.mode')
            if mode in modes:
                ind = modes.index(mode) + 1
                if ind == len(modes):
                    ind = 0
                next_mode = modes[ind]
            else:
                next_mode = modes[0]

            self.set('ap.mode', next_mode)
            return

        if self.last_val('ap.enabled'):
            keys = {SMALL_STARBOARD : (0, 1),
                    SMALL_PORT      : (0, -1),
                    BIG_PORT        : (1, -1),
                    BIG_STARBOARD   : (1, 1)}
            if self.last_val('ap.tack.state') != 'none':
                
                return
            key = None
            dt = 0
            for k in keys:
                if self.testkeydown(k):
                    self.resetmanualkeystate(k)
                    key = k
                    dt = .1
                    break
            else: # determine how long key was pressed if released
                key = self.manualkeystate['key']
                if key:
                    dt = self.lcd.keypad[key].dt()

            if not dt:
                self.resetmanualkeystate(0)
            else:
                speed = keys[key][0] # determine if big or small step
                if speed:
                    change = self.lcd.config['bigstep']
                else:
                    change = self.lcd.config['smallstep']
                if not speed: # if holding down small step buttons do big step per second
                    if dt > 1:
                        change = self.lcd.config['bigstep']*int(dt)

                # update exact heading change if it is now different
                if self.manualkeystate['change'] != change:
                    self.manualkeystate['change'] = change
                    sign = keys[key][1]
                    if 'wind' in self.control['mode']:
                        sign = -sign

                    change = float(change)
                    cmd = int(self.manualkeystate['command']) + sign*change
                    self.tack_hint = time.monotonic(), sign
                    self.set_ap_heading_command(cmd)

        else: # manual control
            speed = self.speed_of_keys()
            if speed:
                self.set('servo.command', speed)
            elif self.lastspeed:
                self.set('servo.command', 0)
            self.lastspeed = speed

        return super(control, self).process()
            
class connecting(controlbase):
    def __init__(self, lcd):
        super(connecting, self).__init__(lcd)
        self.connecting_dots = 0
        
    def display(self, refresh):
        if refresh:
            self.box(rectangle(0, 0, 1, .4), black)
            self.fittext(rectangle(0, 0, 1, .4), _('connect to server'), True)
            self.drawn_text = True
        self.box(rectangle(0, .4, 1, .52), black)
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
