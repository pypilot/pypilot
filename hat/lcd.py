#!/usr/bin/env python3
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time, sys, os, math

from page import *
from page import _

try:
    import micropython
    from upy_client import pypilotClient
    def gettime():
        return time.ticks_ms()/1e3
    import ugfx
except:
    from pypilot.client import pypilotClient
    import font
    import select
    def gettime():
        return time.monotonic()
    from pypilot.hat.ugfx import ugfx
    micropython = False

driver = 'default'
for pdriver in ['nokia5110', 'jlx12864', 'glut', 'framebuffer', 'tft', 'none']:
    if pdriver in sys.argv:
        print('overriding driver', driver, 'to command line', pdriver)
        driver = pdriver
        sys.argv.remove(driver)
        break

class Key():
    def __init__(self):
        self.time = 0
        self.down = 0
        self.up = False

    def update(self, down, count=None):
        if down:
            if not self.time:
                self.down += 1
                self.time = gettime()
            if count:
                t0 = gettime() - count*.1
                self.time = min(t0, self.time)
        elif self.time:
            self.up = True
            self.time = 0

    def dt(self):
        t0 = gettime()
        dt = t0 - self.time
        if self.time:
            if dt > 10:
                self.time = t0-10
            return dt
        return 0

class LCD():
    def __init__(self, config):
        if config:
            self.config = config['lcd']
        else:
            self.config = {}
        self.pipe = False
        self.poller = False
        self.voltage = False
            
        default = {'contrast': 60, 'invert': False, 'backlight': 20,
                   'hue': 214, 'flip': False, 'language': 'en', 'bigstep': 10,
                   'smallstep': 1, 'buzzer': 2};

        for name in default:
            if not name in self.config:
                self.config[name] = default[name]

        global driver
        # set the driver to the one read from hat eeprom, or specified in hat.conf
        if config and 'hat' in config:
            if driver == 'default':
                driver = config['hat']['lcd']['driver']
            self.host = config['host']
        else:
            self.host = False

        self.battery_voltage = 0
        use_tft = True if micropython else False
        self.keypress = False

        use_glut = not use_tft and 'DISPLAY' in os.environ
        self.surface = None

        self.use_glut = False
        print('lcd driver', driver, use_tft, use_glut)
        if driver == 'none':
            page = None
            screen = None
            self.bw = None
        elif driver == 'tft' or (driver == 'default' and use_tft):
            import gc
            if gc.mem_free() > 1e6:  # larger ttgo display
                screen = ugfx.surface(240, 320, 1)
            else:
                screen = ugfx.surface(136, 240, 1)
            self.surface = screen
        elif driver == 'nokia5110' or (driver == 'default' and not use_glut):
            screen = ugfx.spiscreen(0)
        elif driver == 'jlx12864':
            screen = ugfx.spiscreen(1)
        elif driver == 'glut' or (driver == 'default' and use_glut):
            self.use_glut = True
            print('using glut')
            import glut
            # emulate which screen resolution?
            #screen = glut.screen((240, 320))
            screen = glut.screen((136, 240))
            #screen = glut.screen((48, 84))
            #screen = glut.screen((96, 168))
            
            from OpenGL.GLUT import glutKeyboardFunc, glutKeyboardUpFunc
            from OpenGL.GLUT import glutSpecialFunc, glutSpecialUpFunc
            
            glutKeyboardFunc(self.glutkeydown)
            glutKeyboardUpFunc(self.glutkeyup)
            glutSpecialFunc(self.glutspecialdown)
            glutSpecialUpFunc(self.glutspecialup)
            self.glutkeytime = False
            #        glutIgnoreKeyRepeat(True)
        elif driver == 'framebuffer':
            print('using framebuffer')
            screen = ugfx.screen("/dev/fb0")
            if screen.width > 480:
                print('warning huge width')
                #screen.width = 480
                #screen.height= min(screen.height, 640)
                
        if screen:
            self.bw = 1 if screen.width < 120 else False
            self.mag=1

            if not self.surface:
                w, h = screen.width, screen.height
                self.surface = ugfx.surface(w, h, screen.bypp, None)

                # magnify to fill screen
                self.mag = min(screen.width / self.surface.width, screen.height / self.surface.height)
                if self.mag != 1:
                    print('magnifying lcd surface to fit screen')
                    self.magsurface = ugfx.surface(screen)

                self.invsurface = ugfx.surface(self.surface)
        else:
            self.surface = None

        self.screen = screen
        
        set_language(self.config['language'])
        self.client = False
        self.connect()

        self.menu = False
        self.page = connecting(self)
        self.need_refresh = True

        self.keypad = []
        for i in range(NUM_KEYS):
            self.keypad.append(Key())

        self.blink = black, white # two cursor states
        self.data_update = False

    def getmenu(self):
        if not self.menu:
            from menu import mainmenu
            self.menu = mainmenu(self)
        return self.menu
        
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

    def send(self, key, code):
        if self.pipe:
            self.pipe.send((key, code))

    def write_config(self):
        if micropython:
            from config_esp32 import write_config
            write_config(self.config)
        else:
            self.send('write_config', self.config)

    def get_values(self):
        return self.client.get_values()
            
    def key(self, k, down):
        if k < 0 or k >= len(self.keypad):
            return

        self.keypad[k].update(down)
            #self.glutkeytime = k, gettime()

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
            self.key(SMALL_PORT, down)
        elif k == glut.GLUT_KEY_DOWN:
            self.key(SMALL_STARBOARD, down)
        elif k == glut.GLUT_KEY_LEFT:
            self.key(BIG_PORT, down)
        elif k == glut.GLUT_KEY_RIGHT:
            self.key(BIG_STARBOARD, down)

    def display(self):
        t0 = gettime()
        if micropython:
            self.page.watches['ap.heading'] = 1 # heartbeat
            #self.page.watches['imu.accel'] = True
        if self.page.display(self.need_refresh):
            return #optimization
        t1=gettime();

        self.need_refresh = False
        surface = self.surface

        # status cursor
        try:
            if t0-self.blinktime > .8:
                if self.data_update:
                    self.blink = self.blink[1], self.blink[0]
                    self.data_update = False
                self.blinktime = t0
        except:
            self.blinktime = 0
        
        w, h = self.surface.width, self.surface.height
        size = h // 40
        self.surface.box(w-size-1, h-size-1, w-1, h-1, self.blink[0])

        if self.screen != surface:
            if self.config['invert']:
                self.invsurface.blit(surface, 0, 0)
                surface = self.invsurface
                surface.invert(0, 0, surface.width, surface.height)

            if self.mag != 1:
                self.magsurface.magnify(surface, self.mag)
                surface = magsurface

            self.screen.blit(surface, 0, 0, self.config['flip'])

        if 'contrast' in self.config:
            self.screen.contrast = int(float(self.config['contrast'])*9/12)+10

        if micropython:
            self.screen.hue = int(float(self.config['hue']))

        t2=gettime();
        self.screen.refresh()
        t3=gettime();
        #print('display times', t1-t0, t2-t1, t3-t2)

    def update_watches(self):
        for name in list(self.client.watches):
            if name != 'values' and not name in list(self.page.watches):
                self.client.watch(name, False)
        for name, period in self.page.watches.items():
            self.client.watch(name, period)

    def receive(self):
        msgs = self.client.receive()
        if msgs:
            self.data_update = True # allow cursor to blink
            for name, value in msgs.items():
                self.last_msg[name] = value

        if self.pipe:
            if not self.poller:
                self.poller = select.poll()
                self.poller.register(self.pipe, select.POLLIN)
                self.registered = True
                
            while True:
                msg = self.pipe.recv()
                if not msg:
                    break
                index, count = msg
                if index == 'voltage':
                    self.voltage = count
                else:
                    self.keypad[index].update(count, count)

    def reset_keys(self):
        for key in self.keypad:
            key.down = 0
            key.up = False

    def check_voltage(self):
        if not self.voltage:
            return False

        vin, vcc = self.voltage['vin'], self.voltage['vcc']
        if vin < 3 or vin > 3.6:
            return '3v3 Voltage Bad' + (': %.2f' % vin)
        if vcc < 4.5 or vcc > 5.5:
            return '5v Voltage Bad' + (': %.2f' % vcc)
        return False
            
    def poll(self):
        if self.screen == None:
            return

        t0 = gettime()
        self.receive()
        t1 = gettime()
        
        if not self.page:
            frameperiod = 1
            
        else:
            frameperiod = self.page.frameperiod

        t = gettime()
        next_page = self.page.process()
        if next_page and next_page != self.page:
            self.page = next_page
            self.update_watches()
            self.reset_keys()
            self.need_refresh = True
        t2 = gettime()
        
        #        if dt >= frameperiod or dt < 0:
        self.display()
        self.update_watches()
        
        t3 = gettime()

        #dt = t3-t0
        if self.poller:
            self.poller.poll(1000*frameperiod)
        else:
            time.sleep(frameperiod)

        #print('lcd times', t1-t0, t2-t1, t3-t2, dt,t)

def main():
    lcd = LCD(False)
    def idle():
        lcd.poll()
        if lcd.glutkeytime:
            k, t = lcd.glutkeytime
            dt = gettime() - t
            if dt > .5:
                lcd.keypad[k].update(False)
                lcd.glutkeytime = False

        time.sleep(.1)
    if lcd.use_glut:
        from OpenGL.GLUT import glutMainLoop, glutIdleFunc
        glutIdleFunc(idle)
        glutMainLoop()
    else:
        while True:
            lcd.poll()
            time.sleep(.1)
            
if __name__ == '__main__':
    main() 
