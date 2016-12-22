#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import sys, os, time

import ugfx
import font
from signalk.client import SignalKClient


def nr(x):
    try:
        return int(x)
    except:
        return x

class lcd_menu():
    def __init__(self, name, items, prev=False):
        self.selection = 0
        self.name = name
        self.items = items
        self.prev = prev

    def display(self, lcd):
        lcd.text((0, 0), self.name, 16)
        y = 20
        for item in self.items:
            lcd.text((2, y), item[0], 12)
            y += 12

        y = 12*self.selection + 20
        lcd.rectangle(0,y,43,y+12)

class RangeEdit():
    def __init__(self, name, value, signalk_id, client, minval, maxval, step):
        self.name = name
        self.value = value
        self.signalk_id = signalk_id
        self.range = minval, maxval, step
        self.client = client

    def display(self, lcd):
        lcd.text((0, 0), self.name, 12)
        lcd.text((0, 10), str(self.value()), 12)
        self.lcd.get(self.signalk_id)

white = ugfx.color(255, 255, 255)
black = ugfx.color(0, 0, 0)
class LCDClient():
    def __init__(self, screen):

        fontpath = os.path.abspath(os.path.dirname(__file__)) + '/' + 'ugfxfonts'
        t0 = time.time()
        self.fonts = font.load(fontpath, screen.bypp)
        t1 = time.time();
        print "loaded fonts in ", t1 - t0

        w, h = screen.width, screen.height
        #w, h = 44, 84
        width = min(w, 48)
        self.surface = ugfx.surface(width, width*h/w, screen.bypp, None)
        
        self.display_page = self.display_control
        self.range_edit = False

        def settings():
            self.menu = self.settingsmenu
            return self.display_menu

        def info():
            self.info_offset = 0
            return self.display_info

        self.menu = lcd_menu('Menu', [('gain', lambda : self.display_gain),
                                      ('level', lambda : self.display_level),
                                      ('mode', lambda : self.display_mode),
                                      ('settings', settings),
                                      ('info', info)])
        def max_current():
            self.range_edit=RangeEdit('Max Current', lambda : self.last_msg['servo/Max Current'], 'servo/Max Current', self.client, 1, 10, 1)
            return self.range_edit.display
    
        self.settingsmenu = lcd_menu('Settings', [('Max Current', max_current)], self.menu)

        self.gain_selection = 0

        self.mode = 'compass'
        self.last_gps_time = 0

        self.display_page = self.display_connecting
        self.connecting_dots = 0

        self.client = False

        self.keypad = [False, False, False, False, False, False]
        self.keypadup = list(self.keypad)

        self.blink = black, white

    def text(self, pos, text, size):
        font.draw(self.surface, pos, text, self.fonts[size])

    def line(self, x1, y1, x2, y2):
        self.surface.line(x1, y1, x2, y2, black)

    def rectangle(self, x1, y1, x2, y2):
        self.surface.rectangle(x1, y1, x2, y2, black)

    def connect(self):
        watchlist = ['ap/mode', 'ap/heading_command',
                     'imu/heading_lowpass', 'gps/track']
        nalist = watchlist + ['imu/pitch', 'imu/heel', 'servo/Amp Hours',
                              'imu/runtime', 'ap/P', 'ap/I', 'ap/D', 'servo/Max Current']
        self.last_msg = {}
        for name in nalist:
            self.last_msg[name] = 'N/A'
        self.last_msg['ap/heading_command'] = 0
        
        host = ""
        if len(sys.argv) > 1:
            host = sys.argv[1]
        
        def on_con(client):
            for name in watchlist:
                client.watch(name)

        try:
            self.client = SignalKClient(on_con, host)
            self.display_page = self.display_control
            print "connected"
        except:
            self.client = False

    def display_control(self):
        def draw_big_number(pos, num):
            num = str(nr(num))
            while len(num) < 3:
                num = ' ' + num

            s = 0 if self.surface.width < 48 else 1
            self.text((pos[0]+1*s+ 0, pos[1]), num[0], 28)
            self.text((pos[0]+2*s+15, pos[1]), num[1], 28)
            self.text((pos[0]+3*s+30, pos[1]), num[2], 28)

        draw_big_number((-1,0), self.last_msg['imu/heading_lowpass'])

        if self.last_msg['ap/mode'] == 'disabled' or self.last_msg['ap/mode'] == 'N/A':
            self.text((0,24), 'standby', 12)
        else: #if self.last_msg['ap/mode'] != 'N/A':
            draw_big_number((-1,24), self.last_msg['ap/heading_command'])

        modepos = 0, 68
        if self.mode == 'compass':
            self.text(modepos, 'COMPAS', 10)
        elif self.mode == 'gps':
            self.text(modepos, 'GPS', 16)
            if not self.have_gps():
                self.line(modepos[0], modepos[1], modepos[0]+34, modepos[1]+16)
                self.line(modepos[0], modepos[1]+16, modepos[0]+34, modepos[1])
        elif self.mode == 'wind':
            self.text(modepos, 'WIND', 16)

    def display_menu(self):
        self.menu.display(draw)

    gains = [('P', 'ap/P'), ('I', 'ap/I'), ('D', 'ap/D')]
    def display_gain(self):
        self.text((0, 0), 'Gain', 16)

        y = 20
        for gain in self.gains:
              self.text((0, y), gain[0] + str(self.last_msg[gain[1]]), 10)
              self.client.get(gain[1])
              y += 10

        y = 20 + self.gain_selection*10
        self.rectangle(0,y,43,y+10)

    def display_level(self):
        self.text((0, 0), 'press\nselect\nto level', 12)
        self.text((0, 60), 'pitch' + str(self.last_msg['imu/pitch']), 10)
        self.text((0, 70), 'heel' + str(self.last_msg['imu/heel']), 10)
        self.client.get('imu/pitch')
        self.client.get('imu/heel')

    def have_gps(self):
        return time.time() - self.last_gps_time < 5

    def display_mode(self):
        self.text((0, 0), 'MODE', 12)

        y = 16
        modes = {'compass': y, 'gps' : 12+y, 'wind' : 24+y}
        self.text((0, y), 'COMPASS', 10)
        self.text((0, 12+y), '   GPS', 12)
        self.text((0, 24+y), '  WIND', 12)

        y = modes[self.mode]
        rectangle(0,y,43,y+12)

        if self.mode == 'gps' and not self.have_gps():
            self.text((0, 52), 'WARNING', 10)
            self.text((0, 64), 'GPS not', 10)
            self.text((0, 74), 'detected', 10)
        if self.mode == 'wind':
            self.text((0, 52), 'WARNING', 10)
            self.text((0, 64), 'WIND not', 10)
            self.text((0, 74), 'detected', 10)

    def display_connecting(self):
        self.text((0, 0), 'connect', 12)
        self.text((0, 12), 'to server', 10)
        dots = ''
        for i in range(self.connecting_dots):
            dots += '.'
        self.text((0, 22), dots, 12)
        self.connecting_dots += 1
        if self.connecting_dots > 16:
            self.connecting_dots = 0
            
    def display_info(self):
        self.text((0, 0), 'Info', 16)
        self.text((0, 20), 'Amp Hours', 10)

        v =self.last_msg['servo/Amp Hours']
        try:
            v = str(round(v*1000)/1000)
        except:
            pass
            
        self.text((0, 30), v, 10)
        self.text((0, 40), 'runtime', 10)
        self.text((0, 50), self.last_msg['imu/runtime'], 10)
        self.text((0, 60), 'pypilot', 10)
        self.text((0, 70), 'ver 0.1', 10)

        self.client.get('servo/Amp Hours')
        self.client.get('imu/runtime')
        

    def display(self):
        self.surface.fill(black)
        self.display_page()

        # status
        h = self.surface.height

        self.blink = self.blink[1], self.blink[0]
        self.surface.box(0, h-2, 4, h-1, self.blink[0])

        try:
            wlan0 = open('/sys/class/net/wlan0/operstate')
            line = wlan0.readline().rstrip()
            if line == 'up':
                self.text((12, h-10), 'WIFI', 10)
        except:
            pass


    def set(self, name, value):
        if self.client:
            self.client.set(name, value)

    def keydown(self, key):
        self.keypad[key] = True

    def keyup(self, key):
        self.keypadup[key] = True

    def process_keys(self):
        AUTO = 0
        MENU = 1
        UP =   2
        DOWN = 3
        SELECT = 4
                           
        if self.keypadup[AUTO]: # AUTO
            if self.last_msg['ap/mode'] == 'disabled' and self.display_page == self.display_control:
                self.client.set('ap/heading_command', self.last_msg['imu/heading_lowpass'])
                self.set('ap/mode', self.mode)
            else:
                self.client.set('servo/command', 0) #stop
                self.set('ap/mode', 'disabled')
        
            self.display_page = self.display_control
            
        if self.display_page == self.display_control:
            if self.keypadup[MENU]: # MENU
                self.display_page = self.display_menu
            elif self.keypad[UP] or self.keypad[DOWN]: # LEFT/RIGHT
                sign = -1 if self.keypad[UP] else 1
                held = False
                if self.last_msg['ap/mode'] == 'disabled':
                    self.set('servo/command', sign*self.speed)
                    if self.speed < .18:
                        self.speed += .01 # provide acceleration
                else:
                    cmd = self.last_msg['ap/heading_command'] + sign*(9*held + 1)
                    self.set('ap/heading_command', cmd)
            else:
                self.speed = .08
        elif self.display_page == self.display_menu:
            if self.keypadup[MENU]:
                if self.menu.prev:
                    self.menu = self.menu.prev
                else:
                    self.display_page = self.display_control
            elif self.keypadup[UP]:
                self.menu.selection -= 1
                if self.menu.selection < 0:
                    self.menu.selection = len(self.menu.items)-1
            elif self.keypadup[DOWN]:
                self.menu.selection += 1
                if self.menu.selection == len(self.menu.items):
                    self.menu.selection = 0
            elif self.keypadup[SELECT]:
                self.display_page = self.menu.items[self.menu.selection][1]()

        elif self.display_page == self.display_gain:
            if self.keypadup[MENU]:
                self.display_page = self.display_menu
            if self.keypadup[UP]:
                self.gain_selection -= 1
                if self.gain_selection < 0:
                    self.gain_selection = len(self.gains) - 1
            if self.keypadup[DOWN]:
                self.gain_selection += 1
                if self.gain_selection == len(self.gains):
                    self.gain_selection = 0
            elif self.keypadup[SELECT]:
                gain = self.gains[self.gain_selection]
                self.range_edit=RangeEdit(gain[0], lambda : self.last_msg[gain[1]], gain[1], self.client, 0, 1, .0005)
                self.display_page = self.range_edit.display
        elif self.display_page == self.display_level:
            if self.keypadup[MENU]:
                self.display_page = self.display_menu
            elif self.keypadup[SELECT]:
                pass #level
        elif self.display_page == self.display_mode:
            if self.keypadup[MENU]:
                self.display_page = self.display_menu
            elif self.keypadup[UP]:
                self.mode = {'compass': 'wind', 'gps': 'compass', 'wind': 'gps'}[self.mode]
            elif self.keypadup[DOWN]:
                self.mode = {'compass': 'gps', 'gps': 'wind', 'wind': 'compass'}[self.mode]
            elif self.keypadup[SELECT]:
                self.display_page = self.display_control

        elif self.display_page == self.display_info:
            if self.keypadup[MENU]:
                self.display_page = self.display_menu
            elif self.keypadup[UP]:
                self.info_offset -= 10
                if 0 > self.info_offset:
                    self.info_offset = 0
            elif self.keypadup[DOWN]:
                self.info_offset += 10
                if 120 - 44 < self.info_offset:
                    self.info_offset = 120 - 44
            elif self.keypadup[SELECT]:
                self.display_page = self.display_menu

        elif self.range_edit and self.display_page == self.range_edit.display:
            if self.keypadup[MENU]:
                self.display_page = self.display_menu
            elif self.keypadup[UP]:
                v = min(self.range_edit.value()+self.range_edit.range[2], self.range_edit.range[1])
                self.client.set(self.range_edit.signalk_id, v)
            elif self.keypadup[DOWN]:
                v = max(self.range_edit.value()-self.range_edit.range[2], self.range_edit.range[0])
                self.client.set(self.range_edit.signalk_id, v)
            elif self.keypadup[SELECT]:
                self.display_page = self.display_menu
            
        else:
            # self.display_page = self.display_control
            pass

        for key in range(6):
            if self.keypadup[key]:
                self.keypad[key] = self.keypadup[key] = False

    def idle(self):
        self.process_keys()
        while True:
            result = False
            if self.client:
                result = self.client.receive_single()
            else:
                self.connect()

            if not result:
                time.sleep(.4)
                return

            name, data = result
            self.last_msg[name] = data['value']
            if name == 'ap/mode':
                if data['value'] != 'disabled':
                    self.mode = data['value']
            elif name == 'gps/track':
                self.last_gps_time = time.time()


def main():
    print 'init...'

    screen = ugfx.display("/dev/fb0")
    #screen.fill(white)

    lcdclient = LCDClient(screen)
    print 'complete'

    # magnify to fill screen
    mag = min(screen.width / lcdclient.surface.width, screen.height / lcdclient.surface.height)
    
    while True:
        lcdclient.idle()
        lcdclient.display()
        s = ugfx.surface(lcdclient.surface)
        s.magnify(mag)

        screen.blit(s, 0, 0)       
        time.sleep(.1)

if __name__ == '__main__':
    main()
