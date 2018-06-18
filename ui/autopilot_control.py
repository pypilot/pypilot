#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import wx, sys, subprocess, socket, os, time
import autopilot_control_ui
from signalk.client import *

class AutopilotControl(autopilot_control_ui.AutopilotControlBase):
    ID_MESSAGES = 1000

    def __init__(self):
        super(AutopilotControl, self).__init__(None)
        self.sliderlabels = [-120, -40, -10, -5, 0, 5, 10, 40, 120]
        self.fgGains = self.swGains.GetSizer()

        self.host = False
        if len(sys.argv) > 1:
            self.host = sys.argv[1]

        self.client = False

        self.enabled = False
        self.mode = 'compass'
        self.heading_command = 0
        self.heading = 0
        self.lastcommand = False
        self.recv = {}

        self.timer = wx.Timer(self, self.ID_MESSAGES)
        self.timer.Start(100)
        self.Bind(wx.EVT_TIMER, self.receive_messages, id=self.ID_MESSAGES)


    def on_con(self, client):
        self.fgGains.Clear(True)
        self.watchlist = ['ap.enabled', 'ap.mode', 'ap.heading_command',
                          'gps.source', 'wind.source',
                          'ap.heading', 'servo.flags',
                          'servo.controller',
                          'servo.mode', 'servo.engaged']
        value_list = client.list_values()
        self.gains = []
        for name in value_list:
            if 'AutopilotGain' in value_list[name]:
                sizer = wx.FlexGridSizer( 0, 1, 0, 0 )
		sizer.AddGrowableRow( 2 )
		sizer.SetFlexibleDirection( wx.VERTICAL )

                self.watchlist.append(name)
                self.watchlist.append(name+'gain')
                stname = wx.StaticText( self.swGains, wx.ID_ANY, name)
                sizer.Add( stname, 0, wx.ALL, 5 )
                stvalue = wx.StaticText( self.swGains, wx.ID_ANY, '   N/A   ')
                sizer.Add( stvalue, 0, wx.ALL, 5 )

                hsizer = wx.FlexGridSizer( 1, 0, 0, 0 )
		hsizer.AddGrowableRow( 0 )
		hsizer.SetFlexibleDirection( wx.VERTICAL )
                
                gauge = wx.Gauge( self.swGains, wx.ID_ANY, 1000, wx.DefaultPosition, wx.Size( -1,-1 ), wx.SL_VERTICAL )
                hsizer.Add( gauge, 0, wx.ALL|wx.EXPAND, 5 )
                slider = wx.Slider( self.swGains, wx.ID_ANY, 0, 0, 1000, wx.DefaultPosition, wx.Size( -1,-1 ), wx.SL_VERTICAL| wx.SL_INVERSE)
                hsizer.Add( slider, 0, wx.ALL|wx.EXPAND, 5 )

                sizer.Add( hsizer, 1, wx.EXPAND, 5 )
                
                min_val, max_val = value_list[name]['min'], value_list[name]['max']
                gain = {'stname': stname, 'stvalue': stvalue, 'gauge': gauge, 'slider': slider, 'min': min_val, 'max': max_val, 'need_update': False, 'last_change': 0, 'sliderval': 0}
                self.gains.append(gain)
                def make_ongain(gain):
                    def do_gain(event):
                        gain['need_update'] = True
                        gain['last_change'] = time.time()
                    return do_gain
                slider.Bind( wx.EVT_SCROLL, make_ongain(gain) )

                self.fgGains.Add( sizer, 1, wx.EXPAND, 5 )

        self.GetSizer().Fit(self)
        self.SetSize(wx.Size(500, 580))

        for name in self.watchlist:
            client.watch(name)

    def servo_command(self, command):
        if self.lastcommand != command or command != 0:
            self.lastcommand = command
            self.client.set('servo.command', command)

    def send_gain(self, gain):
        name = gain['stname'].GetLabel()
        slidervalue = gain['slider'].GetValue() / 1000.0 * (gain['max'] - gain['min']) + gain['min']
        self.client.set(name, slidervalue)

    def set_mode_color(self):
        modecolors = {'compass': wx.GREEN, 'gps': wx.YELLOW,
                      'wind': wx.BLUE, 'true wind': wx.CYAN}
        if self.enabled and self.mode in modecolors:
            color = modecolors[self.mode]
        else:
            color = wx.RED
        self.tbAP.SetForegroundColour(color)

    def receive_messages(self, event):
        if not self.client:
            self.stStatus.SetLabel('No Connection')
            try:
                self.client = SignalKClient(self.on_con, self.host, autoreconnect=False)
                self.timer.Start(100)
                self.lastmsgtime = time.time()
            except socket.error:
                self.timer.Start(5000)
                return
                
        command = self.sCommand.GetValue()
        if command != 0:
            if self.enabled:
                self.heading_command += self.apply_command(command)
                self.client.set('ap.heading_command', self.heading_command)
                self.sCommand.SetValue(0)
            else:
                if command > 0:
                    command -= 1
                elif command < 0:
                    command += 1
                self.servo_command(-command / 100.0)
                self.sCommand.SetValue(command)

        for gain in self.gains:
            if gain['need_update']:
                self.send_gain(gain)
                gain['need_update'] = False
                
            if gain['slider'].GetValue() != gain['sliderval'] and \
               time.time() - gain['last_change'] > 1:
                gain['slider'].SetValue(gain['sliderval'])


        try:
            msgs = self.client.receive()
        except ConnectionLost:
            self.client = False
            return

        if not msgs:
            if time.time() - self.lastmsgtime > 2:
                print 'message timeout'
                self.client = False
            return

        self.lastmsgtime = time.time()

        for name in msgs:
            data = msgs[name]
            if not 'value' in data:
                print 'no value?!?!', data
                continue
            value = data['value']
            self.recv[name] = True

            found = False
            for gain in self.gains:
                if name == gain['stname'].GetLabel():
                    gain['stvalue'].SetLabel('%.5f' % value)
                    gain['sliderval'] = (value-gain['min'])*1000/(gain['max'] - gain['min'])
                    found = True
                elif name == gain['stname'].GetLabel() + 'gain':
                    v = abs(value) * 1000.0
                    if v < gain['gauge'].GetRange():
                        gain['gauge'].SetValue(v)
                        if value > 0:
                            gain['gauge'].SetBackgroundColour(wx.RED)
                        elif value < 0:
                            gain['gauge'].SetBackgroundColour(wx.GREEN)
                        else:
                            gain['gauge'].SetBackgroundColour(wx.LIGHT_GREY)
                    else:
                        gain['gauge'].SetValue(0)
                        gain['gauge'].SetBackgroundColour(wx.BLUE)

                    found = True

            if found:
                pass
            elif name == 'servo.raw_command':
                self.tbAP.SetValue(False)
                self.tbAP.SetForegroundColour(wx.RED)
            elif name == 'ap.enabled':
                self.tbAP.SetValue(value)
                self.enabled = value
                self.set_mode_color()
            elif name == 'ap.mode':
                rb = {'compass': self.rbCompass, 'gps': self.rbGPS, 'wind': self.rbWind, 'true wind': self.rbTrueWind}
                rb[value].SetValue(True)
                self.mode = value
                self.set_mode_color()
            elif name == 'ap.heading_command':
                self.stHeadingCommand.SetLabel('%.1f' % value)
                if command == 0:
                    self.heading_command = value
            elif name == 'gps.source':
                self.rbGPS.Enable(value != 'none')
                self.rbTrueWind.Enable(value != 'none' and self.rbWind.IsEnabled())
            elif name == 'wind.source':
                self.rbWind.Enable(value != 'none')
                self.rbTrueWind.Enable(value != 'none' and self.rbGPS.IsEnabled())
            elif name == 'ap.heading':
                self.stHeading.SetLabel('%.1f' % value)
                self.heading = value
            elif name == 'servo.engaged':
                self.stEngaged.SetLabel('Engaged' if value else 'Disengaged')
            elif name == 'servo.flags':
                self.stStatus.SetLabel(value)
            elif name == 'servo.controller':
                self.stController.SetLabel(value)
            elif name == 'servo.mode':
                self.stMode.SetLabel(value)
            else:
                print 'warning: unhandled message "%s"' % name

    def onAP( self, event ):
        self.client.set('servo.raw_command', 0)
        if self.tbAP.GetValue():
            self.client.set('ap.heading_command', self.heading)
            self.client.set('ap.enabled', True)
        else:
            self.client.set('servo.command', 0)
            self.client.set('ap.enabled', False)

    def onMode( self, event):
        if self.rbGPS.GetValue():
            mode = 'gps'
        elif self.rbWind.GetValue():
            mode = 'wind'
        elif self.rbTrueWind.GetValue():
            mode = 'true wind'
        else:
            mode = 'compass'
        self.client.set('ap.mode', mode)

    def onPaintControlSlider( self, event ):
        dc = wx.PaintDC( self.sCommand )
        
        s = self.sCommand.GetSize()

        dc.SetTextForeground(wx.BLACK);
        dc.SetPen(wx.Pen(wx.BLACK));
        dc.SetBrush(wx.TRANSPARENT_BRUSH);
        y = 10
        x = 0
        for l in self.sliderlabels:
            t = str(abs(l))
            tx = x
            if l > 0:
                tx -= dc.GetTextExtent(t)[0]

            dc.DrawText(t, tx, y)
            dc.DrawLine(x, 0, x, s.y)
            x += s.x / (len(self.sliderlabels) - 1)

    def apply_command(self, command):
        r = self.sCommand.GetMax() - self.sCommand.GetMin() + 1.0
        p = (len(self.sliderlabels) - 1) * (command - self.sCommand.GetMin()) / r
        l0 = self.sliderlabels[int(p)]
        l1 = self.sliderlabels[int(p)+1]
        v = (p - int(p)) * (l1 - l0) + l0
        #print 'a', command, r, p, l0, l1, v
        return v
            
    def onCommand( self, event ):
        if wx.GetMouseState().LeftIsDown():
            x = self.sCommand.ScreenToClient(wx.GetMousePosition()).x
            val = self.sCommand.GetMin() + (self.sCommand.GetMax() - self.sCommand.GetMin()) * x / self.sCommand.GetSize().x
            self.sCommand.SetValue(val)

    def onScope( self, event ):
        subprocess.Popen(['python', os.path.abspath(os.path.dirname(__file__)) + '/../signalk/scope_wx.py'] + sys.argv[1:])
	
    def onClient( self, event ):
        subprocess.Popen(['python', os.path.abspath(os.path.dirname(__file__)) + '/../signalk/client_wx.py'] + sys.argv[1:])
	
    def onCalibration( self, event ):
        subprocess.Popen(['python', os.path.abspath(os.path.dirname(__file__)) + '/autopilot_calibration.py'] + sys.argv[1:])
	
    def onClose( self, event ):
	self.Close()

def main():
    app = wx.App()
    AutopilotControl().Show()
    app.MainLoop()

if __name__ == "__main__":
    main()

