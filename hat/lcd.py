#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import sys, os, time, math

from menu import mainmenu, connecting
from page import *
from page import _

try:
    from pypilot.client import pypilotClient
    import font
    def gettime():
        return time.monotonic()
except:
    from upy_client import pypilotClient
    from tftscreen import font
    def gettime():
        return time.time()

class LCD():
    def __init__(self, hat):
        self.hat = hat

        if hat:
            self.config = hat.config['lcd']
        else:
            self.config = {}
            
        default = {'contrast': 60, 'invert': False, 'backlight': 30,
                   'flip': False, 'language': 'en', 'bigstep': 10,
                   'smallstep': 1};

        for name in default:
            if not name in self.config:
                self.config[name] = default[name]

        # set the driver to the one from hat eeprom
        driver = 'default'
        use_tft = False
        if self.hat and self.hat.hatconfig:
            driver = self.hat.hatconfig['lcd']['driver']
            self.host = self.hat.client.config['host']
        else:
            #use_tft = True
            self.host = '10.10.10.1'
            
        for pdriver in ['nokia5110', 'jlx12864', 'glut', 'framebuffer', 'tft', 'none']:
            if pdriver in sys.argv:
                sys.argv.remove(pdriver)
                driver = pdriver
                break
            
        print('using lcd driver', driver)
        from ugfx import ugfx

        if not use_tft:
            use_glut = 'DISPLAY' in os.environ
        self.surface = False
        self.use_glut = False
        if driver == 'none':
            page = None
        elif driver == 'tft' or (driver == 'default' and use_tft):
            import tftscreen
            self.surface = screen = tftscreen.screen()
        elif driver == 'nokia5110' or (driver == 'default' and not use_glut):
            screen = ugfx.spiscreen(0)
        elif driver == 'jlx12864':
            print('ugfx', ugfx)
            screen = ugfx.spiscreen(1)
        elif driver == 'glut' or (driver == 'default' and use_glut):
            self.use_glut = True
            print('using glut')
            import glut
            # emulate which screen resolution?
            #screen = glut.screen((240, 320))
            screen = glut.screen((150, 240))
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
            self.bw = 1 if screen.width < 240 else False
            self.mag=1

            if not self.surface:
                w, h = screen.width, screen.height
                from pypilot.hat.ugfx import ugfx
                
                self.surface = ugfx.surface(w, h, screen.bypp, None)

                # magnify to fill screen
                self.mag = min(screen.width / self.surface.width, screen.height / self.surface.height)
                if self.mag != 1:
                    print('magnifying lcd surface to fit screen')
                    self.magsurface = ugfx.surface(screen)

                self.invsurface = ugfx.surface(self.surface)            
        else:
            self.surface = None

        self.lastframetime = 0
        self.screen = screen
        
        set_language(self.config['language'])
        self.client = False
        self.connect()

        self.menu = mainmenu(self)
        self.page = connecting(self)

        self.keypad = [False, False, False, False, False, False, False, False]
        self.keypadup = list(self.keypad)

        self.blink = black, white
        self.wifi = False
        
    def set_language(self, lang):
        set_language(lang)
        self.config['language'] = lang
        self.write_config()
        
    def connect(self):
        self.last_msg = {}
        self.last_msg['gps.source'] = 'none'
        self.last_msg['wind.source'] = 'none'
        self.last_msg['ap.heading_command'] = 0
        
        if self.client:
            self.client.disconnect()

        self.client = pypilotClient(self.host)
        ret = self.client.list_values()

    def write_config(self):
        if self.hat:
            self.hat.write_config()

    def value_list(self):
        v = self.client.values.value
        if v:
            return v
        return {}
            
    def display(self):
        self.page.display()

        # status cursor
        t0 = gettime()
        try:
            if t0-self.blinktime > .5:
                self.blink = self.blink[1], self.blink[0]
                self.blinktime = t0
        except:
            self.blinktime = 0
        w, h = self.surface.width, self.surface.height
        size = h // 40
        self.surface.box(w-size-1, h-size-1, w-1, h-1, self.blink[0])

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

    def display(self):
        self.page.display()
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

        if 'backlight' in self.config and self.hat:
            self.hat.arduino.set_backlight(int(self.config['backlight']))

    def update_watches(self):
        for name in list(self.client.watches):
            if name != 'values' and not name in list(self.page.watches):
                self.client.watch(name, False)
        for name, period in self.page.watches.items():
            self.client.watch(name, period)
            
    def poll(self):
        msgs = self.client.receive()
        for name, value in msgs.items():
            self.last_msg[name] = value

        if not self.page:
            frameperiod = 1;
        else:
            frameperiod = self.page.frameperiod
        
        t = gettime()
        dt = t - self.lastframetime
        if dt > frameperiod:
            self.display()
            self.update_watches()
            self.lastframetime = max(self.lastframetime+frameperiod, t-frameperiod)

        next_page = self.page.process()
        if next_page and next_page != self.page:
            self.page = next_page
            self.update_watches()

        for key in range(len(self.keypad)):
            if self.keypadup[key]:
                self.keypad[key] = self.keypadup[key] = False
                if self.hat:
                    self.hat.buzzer.beep()

def main():
    lcd = LCD(False)
    if lcd.use_glut:
        from OpenGL.GLUT import glutMainLoop, glutIdleFunc
        glutIdleFunc(lcd.poll)
        glutMainLoop()
    else:
        while True:
            lcd.poll()
            time.sleep(.25)
            
if __name__ == '__main__':
    main() 

