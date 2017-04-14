#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import wx, sys, subprocess, os
import autopilot_control_ui
from signalk.client import SignalKClient

class AutopilotControl(autopilot_control_ui.AutopilotControlBase):
    ID_MESSAGES = 1000
    ID_GPS_TIMEOUT = 1001
    ID_WIND_TIMEOUT = 1002

    def __init__(self):
        super(AutopilotControl, self).__init__(None)
        fgGains = self.swGains.GetSizer()

        host = False
        if len(sys.argv) > 1:
            host = sys.argv[1]
            
        watchlist = []
        def on_con(client):
            for name in watchlist:
                client.watch(name)

        self.client = SignalKClient(on_con, host, autoreconnect=True)

        self.gps_heading_offset = 0

        watchlist += ['ap/enabled', 'ap/mode', 'ap/heading_command',
                      'gps/track', 'wind/direction',
                      'ap/heading', 'servo/command', 'servo/flags',
                      'servo/mode', 'servo/engauged']

        value_list = self.client.list_values()
        self.gains = []
        for name in value_list:
            if 'AutopilotGain' in value_list[name]:
                sizer = wx.FlexGridSizer( 0, 1, 0, 0 )
		sizer.AddGrowableRow( 2 )
		sizer.SetFlexibleDirection( wx.VERTICAL )

                watchlist.append(name)
                watchlist.append(name+'gain')
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
                self.gains.append((stname, stvalue, gauge, slider, min_val, max_val))

                fgGains.Add( sizer, 1, wx.EXPAND, 5 )


        on_con(self.client)

        self.GetSizer().Fit(self)

        self.enabled = False
        self.mode = 'compass'
        self.heading_command = 0
        self.heading = 0
        self.lastcommand = False
        self.recv = {}

        self.timer = wx.Timer(self, self.ID_MESSAGES)
        self.timer.Start(50)
        self.Bind(wx.EVT_TIMER, self.receive_messages, id=self.ID_MESSAGES)

        self.gps_timer = wx.Timer(self, self.ID_GPS_TIMEOUT)
        self.Bind(wx.EVT_TIMER, self.gps_timeout, id=self.ID_GPS_TIMEOUT)

        self.wind_timer = wx.Timer(self, self.ID_WIND_TIMEOUT)
        self.Bind(wx.EVT_TIMER, self.wind_timeout, id=self.ID_WIND_TIMEOUT)

        self.SetSize(wx.Size(500, 580))
        self.stop()

    def stop(self):
        self.client.set('ap/enabled', False)
        self.client.set('servo/command', 0)

    def servo_command(self, command):
        if self.lastcommand != command:
            self.lastcommand = command

            self.client.set('servo/command', command)

    def send_gain(self, gain):
        stname, stvalue, gauge, slider, min_val, max_val = gain
        name = stname.GetLabel()

        if not name in self.recv:
            return
        try:
            statvalue = float(stvalue.GetLabel())
        except:
            statvalue = -1 # force update
        slidervalue = slider.GetValue() / 1000.0 * (max_val - min_val) + min_val

        if abs(statvalue - slidervalue) >= .001 / 100.0:
            self.client.set(name, slidervalue)

    def receive_messages(self, event):
        command = self.sCommand.GetValue()
        if self.enabled:
            if command != 0:
                self.heading_command += (-1 if command < 0 else 1) / 2.0
                self.client.set('ap/heading_command', self.heading_command)
        else:
            self.servo_command(command / 50.0)

        if command > 0:
            self.sCommand.SetValue(command - 1)
        elif command < 0:
            self.sCommand.SetValue(command + 1)

        for gain in self.gains:
            self.send_gain(gain)

        msgs = self.client.receive()
        for name in msgs:
            data = msgs[name]
            value = data['value']
            self.recv[name] = True

            found = False
            for gain in self.gains:
                stname, stvalue, gauge, slider, min_val, max_val = gain
                if name == stname.GetLabel():
                    stvalue.SetLabel('%.5f' % value)
                    val = (value-min_val)*1000/(max_val - min_val)
                    if slider.GetValue() != val:
                        slider.SetValue(val)
                    found = True
                elif name == stname.GetLabel() + 'gain':
                    v = abs(value) * 1000.0 * 5
                    if v < gauge.GetRange():
                        gauge.SetValue(v)
                        if value > 0:
                            gauge.SetBackgroundColour(wx.RED)
                        else:
                            gauge.SetBackgroundColour(wx.GREEN)
                    else:
                        gauge.SetValue(0)
                        gauge.SetBackgroundColour(wx.BLUE)

                    found = True

            if found:
                pass
            elif name == 'servo/raw_command':
                self.tbAP.SetValue(False)
                self.tbAP.SetForegroundColour(wx.RED)
            elif name == 'ap/enabled':
                self.tbAP.SetValue(value)
                if value:
                    color = {'compass': wx.GREEN, 'gps': wx.BLUE, 'wind': wx.YELLOW}[self.mode]
                else:
                    color = wx.RED
                self.tbAP.SetForegroundColour(color)
                self.enabled = value
            elif name == 'ap/mode':
                rb = {'compass': self.rbCompass, 'gps': self.rbGPS, 'wind': self.rbWind}
                rb[value].SetValue(True)
                self.mode = value
            elif name == 'ap/heading_command' and not self.rbGPS.GetValue():
                self.stHeadingCommand.SetLabel('%.1f' % value)
                if command == 0:
                    self.heading_command = value
            elif name == 'gps/track':
                self.rbGPS.Enable()
                self.gps_timer.Start(5000)
            elif name == 'wind/direction':
                self.rbWind.Enable()
                self.wind_timer.Start(5000)
            elif name == 'ap/heading':
                self.stHeading.SetLabel('%.1f' % value)
                self.heading = value
            elif name == 'servo/command':
                #print 'command', command
                pass
            elif name == 'servo/engauged':
                self.stEngauged.SetLabel('Ready' if value else 'Fault')
            elif name == 'servo/flags':
                self.stStatus.SetLabel(value)
            elif name == 'servo/mode':
                self.stMode.SetLabel(value)
            else:
                print 'warning: unhandled message "%s"' % name

    def gps_timeout(self, event):
        self.rbGPS.Disable()

    def wind_timeout(self, event):
        self.rbWind.Disable()

    def onAP( self, event ):
        self.client.set('servo/raw_command', 0)
        if self.tbAP.GetValue():
            self.client.set('ap/heading_command', self.heading)
            self.client.set('ap/enabled', True)
        else:
            self.client.set('servo/command', 0)
            self.client.set('ap/enabled', False)

    def onMode( self, event):
        if self.rbGPS.GetValue():
            mode = 'gps'
        elif self.rbWind.GetValue():
            mode = 'wind'
        else:
            mode = 'compass'
        self.client.set('ap/mode', mode)

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
        self.stop()
	self.Close()

def main():
    app = wx.App()
    AutopilotControl().Show()
    app.MainLoop()

if __name__ == "__main__":
    main()

