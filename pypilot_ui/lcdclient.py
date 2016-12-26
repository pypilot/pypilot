#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time
from signalk.client import SignalKClient

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import os
fontpath = os.path.abspath(os.path.dirname(__file__)) + '/' + 'lcdclient_font.ttf'

font8 = ImageFont.truetype(fontpath, 8)
font10 = ImageFont.truetype(fontpath, 10)
font12 = ImageFont.truetype(fontpath, 12)
font16 = ImageFont.truetype(fontpath, 16)
font28 = ImageFont.truetype(fontpath, 28)

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

    def display(self, draw):
        draw.text((0, 0), self.name, font=font16)
        y = 20
        for item in self.items:
            draw.text((2, y), item[0], font=font12)
            y += 12

        y = 12*self.selection + 20
        draw.rectangle((0,y,43,y+12), outline=0)

class RangeEdit():
    def __init__(self, name, value, signalk_id, client, minval, maxval, step):
        self.name = name
        self.value = value
        self.signalk_id = signalk_id
        self.range = minval, maxval, step
        self.client = client

    def display(self, draw):
        draw.text((0, 0), self.name, font=font12)
        draw.text((0, 10), str(self.value()), font=font12)
        self.client.get(self.signalk_id)

class LCDClient():
    # lcd size
    dim = 44, 84

    def __init__(self):        
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
        self.client = False

        self.keypad = [False, False, False, False, False, False]
        self.keypadup = list(self.keypad)

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
            time.sleep(1)

    def display_control(self, draw):
        def draw_big_number(pos, num):
            num = str(nr(num))
            while len(num) < 3:
                num = ' ' + num
            draw.text(pos,                 num[0], font=font28)
            draw.text((pos[0]+15, pos[1]), num[1], font=font28)
            draw.text((pos[0]+30, pos[1]), num[2], font=font28)

        draw_big_number((-1,-4), self.last_msg['imu/heading_lowpass'])

        if self.last_msg['ap/mode'] == 'disabled' or self.last_msg['ap/mode'] == 'N/A':
            draw.text((0,24), 'standby', font=font12)
        else: #if self.last_msg['ap/mode'] != 'N/A':
            draw_big_number((-1,20), self.last_msg['ap/heading_command'])

        modepos = 0, 68
        if self.mode == 'compass':
            draw.text(modepos, 'COMPAS', font=font10)
        elif self.mode == 'gps':
            draw.text(modepos, 'GPS', font=font16)
            if not self.have_gps():
                draw.line((modepos[0], modepos[1], modepos[0]+34, modepos[1]+16), fill=0)
                draw.line((modepos[0], modepos[1]+16, modepos[0]+34, modepos[1]), fill=0)
        elif self.mode == 'wind':
            draw.text(modepos, 'WIND', font=font16)

    def display_menu(self, draw):
        self.menu.display(draw)

    gains = [('P', 'ap/P'), ('I', 'ap/I'), ('D', 'ap/D')]
    def display_gain(self, draw):
        draw.text((0, 0), 'Gain', font=font16)

        y = 20
        for gain in self.gains:
              draw.text((0, y), gain[0] + str(self.last_msg[gain[1]]), font=font10)
              self.client.get(gain[1])
              y += 10

        y = 20 + self.gain_selection*10
        draw.rectangle((0,y,43,y+10), outline=0)

    def display_level(self, draw):
        draw.text((0, 0), 'press\nselect\nto level', font=font12)
        draw.text((0, 60), 'pitch' + str(self.last_msg['imu/pitch']), font=font10)
        draw.text((0, 70), 'heel' + str(self.last_msg['imu/heel']), font=font10)
        self.client.get('imu/pitch')
        self.client.get('imu/heel')

    def have_gps(self):
        return time.time() - self.last_gps_time < 5

    def display_mode(self, draw):
        draw.text((0, 0), 'MODE', font=font12)

        y = 16
        modes = {'compass': y, 'gps' : 12+y, 'wind' : 24+y}
        draw.text((0, y), 'COMPASS', font=font10)
        draw.text((0, 12+y), '   GPS', font=font12)
        draw.text((0, 24+y), '  WIND', font=font12)

        y = modes[self.mode]
        draw.rectangle((0,y,43,y+12), outline=0)

        if self.mode == 'gps' and not self.have_gps():
            draw.text((0, 52), 'WARNING', font=font10)
            draw.text((0, 64), 'GPS not', font=font10)
            draw.text((0, 74), 'detected', font=font10)
        if self.mode == 'wind':
            draw.text((0, 52), 'WARNING', font=font10)
            draw.text((0, 64), 'WIND not', font=font10)
            draw.text((0, 74), 'detected', font=font10)

    def display_connecting(self, draw):
        draw.text((0, 0), 'CONNECTING', font=font12)
        draw.text((0, 18), 'autopilot', font=font10)

        draw.text((2, 34), 'server', font=font12)

    def display_info(self, draw):
        draw.text((0, 0), 'Info', font=font16)
        draw.text((0, 20), 'Amp Hours', font=font10)

        v =self.last_msg['servo/Amp Hours']
        try:
            v = str(round(v*1000)/1000)
        except:
            pass
            
        draw.text((0, 30), v, font=font10)
        draw.text((0, 40), 'runtime', font=font10)
        draw.text((0, 50), self.last_msg['imu/runtime'], font=font10)
        draw.text((0, 60), 'pypilot', font=font10)
        draw.text((0, 70), 'ver 0.1', font=font10)

        self.client.get('servo/Amp Hours')
        self.client.get('imu/runtime')
        

    def display(self):
        self.image = Image.new('1', LCDClient.dim)
        draw = ImageDraw.Draw(self.image)
        draw.rectangle((0,0,LCDClient.dim[0], LCDClient.dim[1]), outline=255, fill=255)
        
        self.display_page(draw)
        return self.image

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
            self.display_page = self.display_control

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
                time.sleep(.1)
                return

            name, data = result
            self.last_msg[name] = data['value']
            if name == 'ap/mode':
                if data['value'] != 'disabled':
                    self.mode = data['value']
            elif name == 'gps/track':
                self.last_gps_time = time.time()


import pygame
import sys
#size = (pygame.display.Info().current_w, pygame.display.Info().current_h)

def main():
    size = 44, 84
    black = 0, 0, 0
    print 'init...'
    lcdclient = LCDClient()
    pygame.init()
    pygame.mouse.set_visible(False)
    print 'complete'

    mode = 0, 0
    if pygame.display.get_driver() == 'x11':
        modes = pygame.display.list_modes()
        if len(modes) >= 1:
            mode = modes[0][0]*2/3, modes[0][1]*2/3

    scale = min(mode[0] / size[0], mode[1] / size[1])
    screen = pygame.display.set_mode((scale*size[0], scale*size[1]))

#    print 'got screen', screen.get_size(), scale
    while True:
        screen.fill(black)
        image = lcdclient.display().convert('RGB')
        mode = image.mode
        size = image.size
        data = image.tobytes()

        #ball = pygame.image.load("test.gif")
        surface = pygame.image.fromstring(data, size, mode)

        surface = pygame.transform.scale(surface, map(lambda x : scale * x, size))
        
        screen.blit(surface, (0, 0))

        pygame.display.flip()

        for i in range(10):
            lcdclient.idle()
            if pygame.event.peek():
                break
            time.sleep(.05)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
            if event.type == pygame.KEYDOWN:
                k = '%c' % (event.key%0xff)
                if k >= '1' and k <= '5':
                    lcdclient.keydown(int(k) - 1)
                else:
                    exit(0)
            elif event.type == pygame.KEYUP:
                k = '%c' % (event.key%0xff)
                if k >= '1' and k <= '5':
                    lcdclient.keyup(int(k) - 1)

if __name__ == '__main__':
    main()

    """
from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *

import numpy

if __name__ == '__main__':
    lcdclient = LCDClient()

    glutInit(sys.argv)
    glutInitWindowPosition(250, 0)
    glutInitWindowSize(3*lcdclient.dim[0], 2*lcdclient.dim[1])
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB)
    glutCreateWindow ("glplot")

    def display():
        image = lcdclient.display()
        
        data = numpy.array(list(image.convert('RGB').getdata()), numpy.int8)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, LCDClient.dim[0], LCDClient.dim[1], 0, GL_RGB,
                     GL_UNSIGNED_BYTE, data)

        glEnable(GL_TEXTURE_2D)

        glBegin(GL_QUADS)
        glTexCoord2f(0, 1), glVertex2f(-1, -1)
        glTexCoord2f(1, 1), glVertex2f(1, -1)
        glTexCoord2f(1, 0), glVertex2f(1, 1)
        glTexCoord2f(0, 0), glVertex2f(-1, 1)
        glEnd()
        
        glDisable(GL_TEXTURE_2D)
        glutSwapBuffers()

    def reshape (w, h):
        glViewport (0, 0, w, h)
        glMatrixMode (GL_PROJECTION)
        glLoadIdentity ()
        glMatrixMode (GL_MODELVIEW)
        glLoadIdentity()
        gluOrtho2D(0, 1, 0, 1)

    def key(k, x, y):
        if k >= '1' and k <= '5':
            lcdclient.keydown(int(k) - 1)
        else:
            exit(0)

    def keyup(k, x, y):
        if k >= '1' and k <= '5':
            lcdclient.keyup(int(k) - 1)

    def special(key, x, y):
        pass

    def idle():
        lcdclient.idle()

 
    glutDisplayFunc(display)
    glutKeyboardFunc(key)
    glutKeyboardUpFunc(keyup)
    glutSpecialFunc(special)
    glutIdleFunc(idle)

    glClearColor (0.0, 0.0, 0.0, 0.0)

    tex = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, tex)

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)

    glPixelStorei(GL_UNPACK_ALIGNMENT,1)

    def timeout(arg):
        glutPostRedisplay()
        glutTimerFunc(arg, timeout, arg)

    fps = 3
    glutTimerFunc(0, timeout, 1000/fps)
    glutMainLoop()
"""
    
