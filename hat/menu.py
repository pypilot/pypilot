#!/usr/bin/env python
#
#   Copyright (C) 2021 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time, os

from page import *
from page import _

try:
    import micropython
except:
    micropython = False

class menu(page):
    def __init__(self, name, items):
        super(menu, self).__init__(name)
        self.selection = 0
        self.items = items
        self.prev = False
        self.last_selection = -1        

    def find_parents(self):
        for p in self.items:
            p.lcd = self.lcd
            p.prev = self
            if isinstance(p, menu):
                p.find_parents()

    def mainmenu(self):
        if self.prev:
            return self.prev.mainmenu()
        return self

    def display(self, refresh):
        self.lcd.menu = self
        if not refresh:
            if self.selection == self.last_selection:
                return
        self.last_selection = self.selection
            
        self.fill(black)
        fit = self.fittext(rectangle(0, 0, 1, .25), self.name)

        sy = y = fit[1] + .03
        items = min(int((1 - y)/.15), len(self.items))
        scroll = max(self.selection - int(items/2), 0)
        scroll = min(scroll, len(self.items) - items)
        maxsizeslider = 0
        sliders = []
        for item in self.items[scroll:]:
            size = self.fittext(rectangle(0, y, 1, .15), item.name)
            if isinstance(item, ValueCheck):
                val = self.last_val(item.pypilot_path)
                if val: # draw if value is true
                    self.invertrectangle(rectangle(.8, y+.07, .1, .07))
            elif isinstance(item, RangeEdit) and size[0] < .8:
                maxsizeslider = max(size[0] + .02, maxsizeslider)
                sliders.append((item,y))
            y += .15
            if y >= .9:
                break

        for item, y in sliders:
            # slider, draw box showing value
            sliderarea = rectangle(maxsizeslider, y+.05, (1-maxsizeslider), .07)
            self.rectangle(sliderarea, .015)
            try:
                values = self.lcd.client.get_values()
                name = item.pypilot_path
                minv = values[name]['min']
                maxv = values[name]['max']
                val = (self.last_val(name, 0, 0)-minv) / (maxv - minv)
                sliderarea.width *= val
                self.rectangle(sliderarea)
            except Exception as e:
                pass
            
        # invert selected menu item
        if self.selection >= 0:
            y = .15*(self.selection-scroll) + sy
            if y < .85:
                self.invertrectangle(rectangle(0, y+.01, 1, .14))

    def process(self):
        if self.testkeydown(AUTO):
            self.lcd.menu = self.lcd.menu.mainmenu()
            return control(self.lcd)
        if self.testkeydown(SMALL_PORT) or self.testkeydown(BIG_PORT):
            self.selection -= 1
            if self.selection < 0:
                self.selection = len(self.items)-1
        elif self.testkeydown(SMALL_STARBOARD) or self.testkeydown(BIG_STARBOARD):
            self.selection += 1
            if self.selection == len(self.items):
                self.selection = 0
        elif self.testkeydown(MENU):
            if self.selection >= 0 and self.selection < len(self.items):
                return self.items[self.selection]
            return
        # in case server changes the number of items
        if self.selection >= len(self.items):
            self.selection = len(self.items)-1

        return super(menu, self).process()

class RangeEdit(page):
    def __init__(self, name, desc, id, pypilot_path, minval, maxval, step):
        self.name = name
        if type(desc) == type('') or type(desc) == type(u''):
            self.desc = lambda : desc
        else:
            self.desc = desc
        self.id = id
        self.pypilot_path = pypilot_path
        self.range = minval, maxval
        self.step = step
        self.lastmovetime = 0
        self.value = None
        super(RangeEdit, self).__init__(name)
     
    def display(self, refresh):
        if refresh:
            self.fill(black)
            self.fittext(rectangle(0, 0, 1, .3), self.name, True)
            self.fittext(rectangle(0, .3, 1, .3), self.desc(), True)
        else:
            self.box(rectangle(0, .6, 1, .4), black)

        # update name
        if gettime()-self.lastmovetime > 1:
            if self.pypilot_path:
                self.value = self.last_val(self.pypilot_path)

        if not self.pypilot_path and not self.value:
            self.value = self.lcd.config[self.id]

        if self.value is False:
            return
        v = self.value
        try:
            v = str(round(10000*v)/10000)
            while len(v) < 6:
                v+='0'
        except:
            pass
        
        self.fittext(rectangle(0, .6, 1, .18), v)

        sliderarea = rectangle(0, .8, 1, .1)
        try:
            self.rectangle(sliderarea, .015)
            xp = (float(v) - self.range[0]) / (self.range[1] - self.range[0])
            sliderarea.width *= xp
            self.rectangle(sliderarea)
        except:
            pass

    def move(self, delta):
        if self.value is False:
            return
        if self.pypilot_path:
            v = self.value
            try:
                v += delta*self.step
            except:
                pass # not connected to server and value is N/A
        else: #config items rounded to integer
            if delta > 0:
                delta = max(1, delta)
            else:
                delta = min(-1, delta)
            v = int(self.value) + delta*self.step
            v = round(v)
            
        v = min(v, self.range[1])
        v = max(v, self.range[0])
        self.value = v
        if self.pypilot_path:
            self.lcd.client.set(self.pypilot_path, v)
        else:
            self.lcd.config[self.id] = v
        self.lastmovetime = gettime()

    def process(self):
        if self.testkeydown(MENU):
            if not self.pypilot_path:
                self.lcd.write_config()
            return self.prev

        keypad = self.lcd.keypad
        def spd(k):
            dt = keypad[k].dt()*2
            if dt or self.testkeydown(k):
                return dt + 1
            return 0
        
        ss = spd(SMALL_STARBOARD)
        sp = spd(SMALL_PORT)
        bp = spd(BIG_PORT)
        bs = spd(BIG_STARBOARD)


        speed = 0;
        sign = 0;
        if sp or ss:
            speed = max(sp, ss)
        if bp or bs:
            speed = max(bp, bs)*3

        if ss or bs:
            sign = 1
        elif sp or bp:
            sign = -1

        speed = sign * speed
        if speed:
            self.move(speed)
        else:
            return super(RangeEdit, self).process()

def ConfigEdit(name, desc, config_name, min, max, step):
    return RangeEdit(name, desc, config_name, False, min, max, step)
        
class ValueEdit(RangeEdit):
    def __init__(self, name, desc, pypilot_path, value=False):
        super(ValueEdit, self).__init__(name, desc, False, pypilot_path, 0, 1, .1)
        self.range = False

    def display(self, refresh):
        if not self.range:
            values = self.lcd.get_values()
            if self.pypilot_path in values:
                info = values[self.pypilot_path]
            else:
                info = {'min': 0, 'max': 0}
            self.range = info['min'], info['max']
            self.step = (self.range[1]-self.range[0])/100.0
        super(ValueEdit, self).display(refresh)

class ValueCheck(page):
    def __init__(self, name, pypilot_path=False):
        super(ValueCheck, self).__init__(name)
        self.pypilot_path = pypilot_path

    def process(self):
        self.set(self.pypilot_path, not self.last_val(self.pypilot_path))
        return self.lcd.menu

class ValueEnumSelect(page):
    def __init__(self, lcd, name, pypilot_path):
        super(ValueEnumSelect, self).__init__(name)
        self.lcd = lcd
        self.pypilot_path = pypilot_path

    def process(self):
        self.set(self.pypilot_path, self.name)
        return control(self.lcd)

class ValueEnum(menu):
    def __init__(self, name, pypilot_path, hide_choices=[]):
        super(ValueEnum, self).__init__(name, [])
        self.pypilot_path = pypilot_path
        self.hide_choices = hide_choices
        self.selection = -1
        
    def process(self):
        if not self.items:
            try:
                values = self.lcd.client.get_values()
                if values:
                    info = values[self.pypilot_path]
                    choices = info['choices']
                else:
                    choices = []
                for choice in self.hide_choices:
                    if choice in choices:
                        choices.remove(choice)
                self.items = list(map(lambda choice : ValueEnumSelect(self.lcd, choice, self.pypilot_path), choices))
            except Exception as e:
                print('failed choices', e)

        if self.selection < 0:
            val = self.last_val(self.pypilot_path)
            for i in range(len(self.items)):
                if self.items[i].name == val:
                    self.selection = i
                
        return super(ValueEnum, self).process()

def GainEdit(gain):
    n = gain[gain.rfind('.')+1:]
    return ValueEdit(n, n, gain, True)
            
class gain(menu):
    def __init__(self):
        self.last_pilot = False
        super(gain, self).__init__(_('gain'), [])

    def curgains(self):
        ret = []
        for name, value in self.lcd.get_values().items():
            if 'AutopilotGain' in value:
                if 'ap.pilot.' in name:
                    s = name.split('.')
                    if self.last_pilot == s[2]:
                        ret.append(name)

        ret.sort() # sort of get PID in order (reverse alphabet)
        ret.reverse()
        return ret

    def process(self):
        pilot = self.last_val('ap.pilot')
        if pilot != self.last_pilot:
            self.last_pilot = pilot
            self.items = list(map(GainEdit, self.curgains()))
            self.find_parents()
            self.lcd.need_refresh = True
            if self.selection < 0:
                self.selection = 0
        return super(gain, self).process()

class level(page):
    def process(self):
        self.set('imu.alignmentCounter', 100)
        return self.lcd.menu

class calibrate_rudder_feedback(ValueEnum):
    def __init__(self):
        super(calibrate_rudder_feedback, self).__init__(_('rudder'), 'rudder.calibration_state', ['idle'])

    def process(self):
        if not self.last_val('rudder.angle'):
            if self.testkeydown(MENU):
                return self.prev
            
        return super(calibrate_rudder_feedback, self).process()
        
    def display(self, refresh):
        if not self.last_val('rudder.angle'):
            self.box(rectangle(0, .18, 1, .82), black)
            self.fittext(rectangle(0, .4, 1, .4), "No Rudder Feedback Detected", True)
            return
        super(calibrate_rudder_feedback, self).display(True)
            
        fit = self.fittext(rectangle(0, .9, 1, .1), str(self.last_val('rudder.angle')))

class calibrate(menu):
    def __init__(self):
        super(calibrate, self).__init__(_('calibrate'),
                                        [level(_('level')),
                                         ValueEdit(_('heading'), self.getheading, 'imu.heading_offset'),
                                         ValueCheck(_('lock'), 'imu.compass.calibration.locked'),
                                         calibrate_rudder_feedback(),
                                         calibrate_info()])
        self.lastcounter = 0

    def getheading(self):
        try:
            return '%.1f' % self.last_val('imu.heading')
        except:
            return str(self.last_val('imu.heading'))
        
    def display(self, refresh):
        counter = self.last_val('imu.alignmentCounter', default=0)
        super(calibrate, self).display(refresh or counter != self.lastcounter)
        self.lastcounter = counter
        if counter:
            r = rectangle(0, 0, 1, .15)
            r.height = .2
            self.fittext(r, ' %d%%' % (100-counter), False, black)
            r.width = 1-float(counter)/100
            r.height = .25
            self.invertrectangle(r)
            
        self.fittext(rectangle(0, .9, .5, .11), self.round_last_val('imu.pitch', 1))
        self.fittext(rectangle(.5, .9, .5, .11), self.round_last_val('imu.heel', 1))

    
class motor(menu):
    def __init__(self):
        super(motor, self).__init__(_('motor'),
                                    [ValueEdit(_('min speed'), _('relative'), 'servo.speed.min'),
                                     ValueEdit(_('max speed'), _('relative'), 'servo.speed.max'),
                                     ValueEdit(_('max current'), _('amps'), 'servo.max_current'),
                                     ValueEdit(_('period'), _('seconds'), 'servo.period'),
                                     ValueEdit(_('clutch pwm'), _('percent'), 'servo.clutch_pwm')])

networking = '/home/tc/.pypilot/networking.txt'
default_network = {'mode': 'Master', 'ssid': 'pypilot', 'key':'', 'client_ssid': 'openplotter', 'client_key': '12345678'}

class select_wifi(page):
    def __init__(self, name, wifi_settings):
        self.wifi_settings = wifi_settings
        super(select_wifi, self).__init__(name)

    def setup_network(self):
        try:
            f = open(networking, 'w')
            for setting in self.wifi_settings:
                f.write(setting+'='+self.wifi_settings[setting]+'\n')
            f.close()
        except Exception as e:
            print('exception writing', networking, ':', e)
        os.system('/opt/networking.sh')
        
class select_wifi_ap_toggle(select_wifi):
    def process(self):
        ap = self.wifi_settings['mode'] == 'Master'
        self.wifi_settings['mode'] = 'Managed' if ap else 'Master'
        self.setup_network()
        return self.lcd.menu

class select_wifi_defaults(select_wifi):
    def process(self):
        for n, v in default_network.items():
            self.wifi_settings[n] = v
        self.setup_network()
        return self.lcd.menu
            
class wifi(menu):
    def __init__(self):
        self.wifi_settings = False
        self.have_wifi = False
        self.mtime = False
        self.wifi_updatetime = gettime()
        if self.read_networking():
            items = [select_wifi_ap_toggle('AP/Client', self.wifi_settings),
                     select_wifi_defaults(_('defaults'), self.wifi_settings)]
        else:
            items = []
        super(wifi, self).__init__('WIFI', items)

    def read_networking(self):
        try:
            # if another process updated the network file
            mtime = os.path.getmtime(networking)
            if mtime == self.mtime:
                return False
            self.wifi_settings = default_network.copy()
            f = open(networking, 'r')
            while True:
                l = f.readline()
                if not l:
                    break
                parsed = l.rstrip().split('=', 1)
                if len(parsed) == 2:
                    setting, value = parsed
                    self.wifi_settings[setting] = value
            f.close()
            return True
        except Exception as e:
            print('failed to read', networking, e)
            return False

    def display(self, refresh):
        if not self.wifi_settings:
            self.fill(black)
            self.fittext(rectangle(0, 0, 1, 1), _('Wifi not managed'), True)
            return

        super(wifi, self).display(refresh)
        self.have_wifi = test_wifi()
        if not self.have_wifi:
            info = _('No Connetion')
            self.fittext(rectangle(0, 0, 1, .20), info)
            
        if self.wifi_settings['mode'] == 'Master':
            info = 'mode: AP\n'
            ssid = 'ssid'
            key = 'key'
        else:
            info = 'mode: Client\n'
            ssid = 'client_ssid'
            key = 'client_key'
        self.fittext(rectangle(0, .6, 1, .13), info)

        info = self.wifi_settings[ssid]
        self.fittext(rectangle(0, .73, 1, .14), info)

        if self.wifi_settings[key]:
            info = self.wifi_settings[key]
            self.fittext(rectangle(0, .87, 1, .13), info)

    def process(self):
        have_wifi = test_wifi()
        if have_wifi != self.have_wifi:
            self.lcd.need_refresh = True

        t = gettime()
        if t - self.wifi_updatetime > 1:
            self.wifi_updatetime = t
            if self.read_networking():
                self.lcd.need_refresh = True
            
        return super(wifi, self).process()
        
class control_menu(menu):
    def __init__(self):
        super(control_menu, self).__init__(_('control'),
                                      [wifi(),
                                       ConfigEdit(_('small step'), _('degrees'), 'smallstep', 1, 5, 1),
                                       ConfigEdit(_('big step'), _('degrees'), 'bigstep', 5, 20, 5)])
class invert(page):
    def process(self):
        self.lcd.config['invert'] = not self.lcd.config['invert']
        self.lcd.write_config()
        return self.lcd.menu

class flip(page):
    def process(self):
        self.lcd.config['flip'] = not self.lcd.config['flip']
        self.lcd.write_config()
        return self.lcd.menu

class display(menu):
    def __init__(self):
        if micropython:
            bl = ConfigEdit(_('hue'), '', 'hue', 0, 255, 1)
        else:
            bl = ConfigEdit(_('backlight'), '', 'backlight', 0, 40, 1)
        super(display, self).__init__(_('display'),
                                      [ConfigEdit(_('contrast'), '', 'contrast', 0, 120, 1),
                                       invert(_('invert')), flip(_('flip')), bl])
class select_language(page):
    def __init__(self, lang):
        super(select_language, self).__init__(lang[0])
        self.lang = lang[1]

    def process(self):
        self.lcd.set_language(self.lang)
        self.lcd.menu = mainmenu(self.lcd) # recreate main menu for language change
        return self.lcd.menu

class language(menu):
    languages = [('català', 'ca'),
                 ('dansk', 'da'),
                 ('deutsch', 'de'),
                 ('Eλληνικά', 'el'),
                 ('english', 'en'),
                 ('español', 'es'),
                 ('suomalainen', 'fi'),
                 ('français', 'fr'),
                 ('italiano', 'it'),
                 ('nederlands', 'nl'),
                 ('norsk', 'no'),
                 ('polskie', 'pl'),
                 ('pycкий', 'ru'),
                 ('svenska', 'sv')
    ]
    def __init__(self):
        super(language, self).__init__(_('language'), list(map(select_language, self.languages)))
        self.selection = -1

    def process(self):
        if self.selection < 0:
            index, self.selection = 0, 0
            for lang in self.languages:
                if lang[1] == self.lcd.config['language']:
                    self.selection = index
                index += 1
        return super(language, self).process()

        
class settings(menu):
    def __init__(self):
        if no_translation == translate:
            lang = []
        else:
            lang = [language()]
        super(settings, self).__init__(_('settings'),
                            [ValueEnum(_('mode'), 'ap.mode'),
                             ValueEnum(_('pilot'), 'ap.pilot'),
                             motor(), control_menu(), display()] + lang)
        
class mainmenu(menu):
    def __init__(self, lcd):
        super(mainmenu, self).__init__(_('Menu'), [gain(), calibrate(), settings(), info()])
        self.lcd = lcd
        self.find_parents()
        self.loadtime = 0

    def display(self, refresh):
        values = self.lcd.get_values()
        if not values:
            if not self.loadtime:
                if self.lcd.client.connection:
                    self.loadtime = gettime()
                    self.lcd.client.list_values()
                else:
                    self.loadtime = 0
            else:
                dt = gettime() - self.loadtime
                self.lcd.surface.fill(black)
                if dt > .2:
                    self.fittext(rectangle(0, 0, 1, .4), _('Loading'))
        else:
            if self.loadtime:
                refresh = True
            self.loadtime = 0

        if self.loadtime:
            dt = gettime() - self.loadtime
            if dt > 11:
                self.fittext(rectangle(0, .4, 1, .2), _('timeout'))
            elif dt > 10:
                self.loadtime = 0
            elif dt > .6:
                self.fittext(rectangle(0, .4, 1, .2), '.'*int(dt*2+.5))
        else:
            super(mainmenu, self).display(refresh)
        
