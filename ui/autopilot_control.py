#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import wx, sys, subprocess, socket, os, time
from gettext import gettext as _
from pypilot.ui import autopilot_control_ui
from pypilot.client import *

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

        self.mode = 'compass'
        self.heading_command = 0
        self.heading = 0
        self.lastcommand = False
        self.recv = {}
        self.rudder = False
        self.apenabled = False
        self.tackstate = False
        #self.bCenter.Show(False)
        self.timer = wx.Timer(self, self.ID_MESSAGES)
        self.timer.Start(100)
        self.Bind(wx.EVT_TIMER, self.receive_messages, id=self.ID_MESSAGES)
        self.init()


    def init(self):
        self.stStatus.SetLabel('No Connection')
        self.client = pypilotClient(self.host)
        self.client.connect(True)
        self.gains = {}
        self.enumerated = False

        watchlist = ['ap.enabled', 'ap.mode', 'ap.heading_command',
                          'ap.tack.state', 'ap.tack.timeout', 'ap.tack.direction',
                          'ap.heading', 'ap.pilot',
                          'gps.source', 'wind.source',
                          'servo.controller', 'servo.engaged', 'servo.flags',
                          'rudder.angle']
        for name in watchlist:
            self.client.watch(name)

    def servo_command(self, command):
        if self.lastcommand != command or command != 0:
            self.lastcommand = command
            self.client.set('servo.command', command)

    def send_gain(self, name, gain):
        slidervalue = gain['slider'].GetValue() / 1000.0 * (gain['max'] - gain['min']) + gain['min']
        self.client.set(name, slidervalue)

    def set_mode_color(self):
        modecolors = {'compass': wx.GREEN, 'gps': wx.YELLOW,
                      'wind': wx.BLUE, 'true wind': wx.CYAN}
        if self.tbAP.GetValue() and self.mode in modecolors:
            color = modecolors[self.mode]
        else:
            color = wx.RED
        self.tbAP.SetForegroundColour(color)

    def enumerate_controls(self, value_list):
        self.tbAP.SetValue(False)
        self.set_mode_color()
        
        self.fgGains.Clear(True)
        self.gains = {}
        pilots = {}
        for name in value_list:
            sname = name.split('.')
            if len(sname) > 2 and sname[0] == 'ap' and sname[1] == 'pilot':
                pilots[sname[2]] = True
                
            if 'AutopilotGain' in value_list[name]:
                sizer = wx.FlexGridSizer( 0, 1, 0, 0 )
                sizer.AddGrowableRow( 2 )
                sizer.SetFlexibleDirection( wx.VERTICAL )
        
                self.client.watch(name)
                self.client.watch(name+'gain')

                lname = name
                sname = name.split('.')
                if len(sname) > 3 and sname[0] == 'ap' and sname[1] == 'pilot':
                    lname = sname[3]
                stname = wx.StaticText( self.swGains, wx.ID_ANY, lname)
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
                gain = {'stname': stname, 'stvalue': stvalue, 'gauge': gauge, 'slider': slider, 'min': min_val, 'max': max_val, 'need_update': False, 'last_change': 0, 'sliderval': 0, 'sizer': sizer}
                self.gains[name] = gain
                def make_ongain(gain):
                    def do_gain(event):
                        gain['need_update'] = True
                        gain['last_change'] = time.monotonic()
                    return do_gain
                slider.Bind( wx.EVT_SCROLL, make_ongain(gain) )
                
        self.enumerate_gains()

        self.cPilot.Clear()
        for pilot in pilots:
            self.cPilot.Append(pilot)
                
        self.GetSizer().Fit(self)
        self.SetSize(wx.Size(570, 420))
        
    def receive_messages(self, event):
        if not self.enumerated and self.client.connection:
            value_list = self.client.list_values(10)
            if value_list:
                self.enumerate_controls(value_list)
                self.enumerated = True
            return
        
        command = self.sCommand.GetValue()
        if command != 0:
            if self.tbAP.GetValue():
                self.heading_command += self.apply_command(command)
                self.client.set('ap.heading_command', self.heading_command)
                self.sCommand.SetValue(0)
            else:
                
                if True:
                    if command > 0:
                        command -= 1
                    elif command < 0:
                        command += 1
                else:
                    if abs(command) < 3:
                        command=0
                self.sCommand.SetValue(command)
                self.servo_command(-command / 100.0)

        for gain_name in self.gains:
            gain = self.gains[gain_name]
            if gain['need_update']:
                self.send_gain(gain_name, gain)
                gain['need_update'] = False
                
            if gain['slider'].GetValue() != gain['sliderval'] and \
               time.monotonic() - gain['last_change'] > 1:
                gain['slider'].SetValue(int(gain['sliderval']))

        msgs = self.client.receive()

        for name in msgs:
            value = msgs[name]
            self.recv[name] = True

            found = False

            for gain_name in self.gains:
                gain = self.gains[gain_name]
                if name == gain_name:
                    gain['stvalue'].SetLabel('%.5f' % value)
                    gain['sliderval'] = (value-gain['min'])*1000/(gain['max'] - gain['min'])
                    found = True
                elif name == gain_name + 'gain':
                    v = abs(value) * 1000.0
                    if v < gain['gauge'].GetRange():
                        gain['gauge'].SetValue(int(v))
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
#            elif name == 'servo.raw_command':
#                self.tbAP.SetValue(False)
#                self.tbAP.SetForegroundColour(wx.RED)
            elif name == 'ap.enabled':
                self.tbAP.SetValue(int(value))
                self.set_mode_color()
                self.apenabled = value
                self.bCenter.Show(not self.apenabled and self.rudder)
            elif name == 'rudder.angle':
                try:
                    value = round(value, 1)
                except:
                    pass
                self.rudder = value
                if (not (not self.apenabled and self.rudder)) == self.bCenter.IsShown():
                    self.bCenter.Show(not self.bCenter.IsShown())
                self.stRudder.SetLabel(str(value))
            elif name == 'ap.mode':
                rb = {'compass': self.rbCompass, 'gps': self.rbGPS, 'wind': self.rbWind, 'true wind': self.rbTrueWind}
                rb[value].SetValue(True)
                self.mode = value
                self.set_mode_color()
            elif name == 'ap.heading_command':
                self.stHeadingCommand.SetLabel('%.1f' % value)
                if command == 0:
                    self.heading_command = value
            elif name == 'ap.pilot':
                self.cPilot.SetStringSelection(value)
                self.enumerate_gains()
            elif name == 'gps.source':
                self.rbGPS.Enable(value != 'none')
                self.rbTrueWind.Enable(value != 'none' and self.rbWind.IsEnabled())
            elif name == 'wind.source':
                self.rbWind.Enable(value != 'none')
                self.rbTrueWind.Enable(value != 'none' and self.rbGPS.IsEnabled())
            elif name == 'ap.heading':
                self.stHeading.SetLabel('%.1f' % value)
                self.heading = value
            elif name == 'ap.tack.state':
                self.stTackState.SetLabel(value)
                self.bTack.SetLabel('Tack' if value == 'none' else 'Cancel')
                self.tackstate = value
            elif name == 'ap.tack.timeout':
                if self.tackstate == 'waiting':
                    self.stTackState.SetLabel(str(value))
            elif name == 'ap.tack.direction':
                self.cTackDirection.SetSelection(value == 'starboard')
            elif name == 'servo.engaged':
                self.stEngaged.SetLabel('Engaged' if value else 'Disengaged')
            elif name == 'servo.flags':
                self.stStatus.SetLabel(value)
            elif name == 'servo.controller':
                self.stController.SetLabel(value)
            elif name == 'servo.current':
                pass # timeout value to know we are receiving
            elif 'ap.pilot.' in name:
                pass
            else:
                print(_('warning: unhandled message') + (' "%s"' % name))

    def onAP( self, event ):
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

    def onPilot(self, event):
        self.client.set('ap.pilot', self.cPilot.GetStringSelection())

    def onTack(self, event):
        if self.bTack.GetLabel() == 'Tack':
            self.tackstate = 'begin'
        else:
            self.tackstate = 'none'
        self.client.set('ap.tack.state', self.tackstate)

    def onTackDirection(self, event):
        self.client.set('ap.tack.direction', 'port' if self.cTackDirection.GetSelection() else 'starboard')

    def onPaintControlSlider( self, event ):
        return
        # gtk3 is a bit broken
        if 'gtk3' in wx.version():
            return
        
        dc = wx.PaintDC( self.sCommand )        
        s = self.sCommand.GetSize()

        #dc.SetTextForeground(wx.BLACK)
        dc.SetPen(wx.Pen(wx.BLACK))
        dc.SetBrush(wx.TRANSPARENT_BRUSH)
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

    def enumerate_gains(self):
        while not self.fgGains.IsEmpty():
            self.fgGains.Detach(0)

        pilot = self.cPilot.GetStringSelection()
        for name in self.gains:
            if pilot in name or not 'ap.pilot.' in name:
                self.gains[name]['sizer'].ShowItems(True)
                self.fgGains.Add( self.gains[name]['sizer'], 1, wx.EXPAND, 5 )
            else:
                self.gains[name]['sizer'].ShowItems(False)

        s = self.GetSize()
        self.Fit()
        self.SetSize(s)
                
    def apply_command(self, command):
        r = self.sCommand.GetMax() - self.sCommand.GetMin() + 1.0
        p = (len(self.sliderlabels) - 1) * (command - self.sCommand.GetMin()) / r
        l0 = self.sliderlabels[int(p)]
        l1 = self.sliderlabels[int(p)+1]
        v = (p - int(p)) * (l1 - l0) + l0
        #print('a', command, r, p, l0, l1, v)
        return v        
    
    def onCommand( self, event ):
        if wx.GetMouseState().LeftIsDown():
            x = self.sCommand.ScreenToClient(wx.GetMousePosition()).x
            val = self.sCommand.GetMin() + (self.sCommand.GetMax() - self.sCommand.GetMin()) * x / self.sCommand.GetSize().x
            self.sCommand.SetValue(val)

    def onCenter( self, event ):
        self.client.set('servo.position_command', 0)

    def onScope( self, event ):
        subprocess.Popen(['pypilot_scope'] + sys.argv[1:])
	
    def onClient( self, event ):
        subprocess.Popen(['pypilot_client_wx'] + sys.argv[1:])
	
    def onCalibration( self, event ):
        subprocess.Popen(['pypilot_calibration'] + sys.argv[1:])
	
    def onClose( self, event ):
        self.Close()

def main():
    app = wx.App()
    AutopilotControl().Show()
    app.MainLoop()

if __name__ == "__main__":
    main()
