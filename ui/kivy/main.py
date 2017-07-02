#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

__version__ = '0.0.1'

'''
Autopilot Control
============

pypilot kivy control app
'''

from kivy.app import App
from kivy.uix.tabbedpanel import TabbedPanel
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.spinner import Spinner
from kivy.uix.widget import Widget
from kivy.uix.image import Image
from kivy.animation import Animation
#from kivy.lang import Builder
from kivy.clock import Clock

#from signalk.client import SignalKClient

class AutopilotControl(TabbedPanel):
    pass

class AutopilotControlApp(App):
    def build(self):
        self.client = False
        self.connect(3)
        Clock.schedule_interval(self.connect, 3)
        Clock.schedule_interval(self.update, .1)

        self.enabled = False
        self.mode = 'compass'
        self.heading_command = 0
        self.heading = 0

        self._anim = None

        self.texture = Image(source='compass.png').texture
        self.control = AutopilotControl()
        return self.control

    def connect(self, dt):
        if self.client:
            return
        
        watchlist = ['ap/enabled', 'ap/mode', 'ap/heading', 'ap/heading_command']
        def on_con(client):
            for name in watchlist:
                client.watch(name)

        return
        try:
            #self.client = SignalKClient(on_con, 'pypilot', autoreconnect=True)
            pass
        except:
            return

    def update(self, dt):
        if not self.client:
            return
        
        result = self.client.receive()
        for msg in result:
            value = result[msg]['value']

            if msg == 'ap/enabled' or msg == 'ap/mode':
                if msg == 'ap/enabled':
                    self.enabled = value
                else:
                    self.mode = value

                color = 'ff0000'
                if self.enabled:
                    colors = {'compass': '00ff00', 'gps': '0000ff', 'wind': 'ffff00'}
                    color = colors[self.mode]
                    
                self.control.ap.text = '[color=' + color + ']AP'
                
            elif msg == 'ap/heading':
                self.control.heading_label.text = str(value)
                self.control.compass.heading = value

                #self.control.compass.canvas.needs_redraw = 1
                print 'self.heading', self.heading
            elif msg == 'ap/heading_command':
                self.control.heading_command_label.text = str(value)
                self.heading_command = value

    def onAP(self):
        if not self.client:
            return

        self.client.set('servo/raw_command', False)
        if self.enabled:
            self.client.set('ap/heading_command', self.heading)
            self.client.set('ap/enabled', True)
        else:
            self.client.set('servo/command', 0)
            self.client.set('ap/enabled', False)
            
if __name__ == '__main__':
    AutopilotControlApp().run()
