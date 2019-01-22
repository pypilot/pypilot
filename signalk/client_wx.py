#!/usr/bin/env python
#
#   Copyright (C) 2018 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import wx, sys, math, subprocess, os, socket
from client import SignalKClient, SignalKClientFromArgs, ConnectionLost

def round3(value):
    if type(value) == type([]):
        return map(round3, value)
    elif type(value) == type({}):
        ret = {}
        for each in value:
            ret[round3(each)] = round3(value[each])
        return ret
    elif type(value) == type(1.0):
        return round(value, 3)
    return value

class MainFrame(wx.Frame):
    def __init__(self):
	wx.Frame.__init__(self, None, title="signalk client", size=(1000, 600))

        self.value_list = []
        self.client = SignalKClientFromArgs(sys.argv, True, self.on_con)
        self.host_port = self.client.host_port
        self.client.autoreconnect = False

        ssizer = wx.FlexGridSizer(0, 1, 0, 0)
        ssizer.AddGrowableRow( 0 )
        ssizer.AddGrowableCol( 0 )
        ssizer.SetFlexibleDirection( wx.BOTH )
        ssizer.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )

        self.scrolledWindow = wx.ScrolledWindow(self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.HSCROLL|wx.VSCROLL )
        self.scrolledWindow.SetScrollRate(5, 5)
        
        sizer = wx.FlexGridSizer(0, 3, 0, 0)
        sizer.AddGrowableCol( 2 )
        sizer.SetFlexibleDirection( wx.BOTH )
        sizer.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )

        self.values = {}
        self.controls = {}
        self.sliderrange = {}
        self.value_list = self.client.list_values()
        self.on_con(self.client)

        for name in sorted(self.value_list):
            sizer.Add( wx.StaticText(self.scrolledWindow, wx.ID_ANY, name), 0, wx.ALL, 5 )
                    
            self.values[name] = wx.StaticText(self.scrolledWindow, wx.ID_ANY)
            sizer.Add( self.values[name], 0, wx.ALL, 5 )

            t = self.value_list[name]['type']

            if t == 'Property':
                tb = wx.TextCtrl(self.scrolledWindow, wx.ID_ANY)
                sizer.Add( tb )
                self.controls[name] = tb

            elif t == 'BooleanProperty':
                def proc(): # encapsulate to fix scope
                    cb = wx.CheckBox(self.scrolledWindow, wx.ID_ANY, '')
                    sizer.Add( cb, 0, wx.EXPAND)
                    self.controls[name] = cb

                    cbname = name
                    def oncheck(event):
                        self.client.set(cbname, cb.GetValue() )
                    cb.Bind( wx.EVT_CHECKBOX, oncheck )
                proc()

            elif t == 'RangeProperty':
                useSlider = True
                def proc():
                    r = self.value_list[name]['min'], self.value_list[name]['max']
                    if useSlider:
                        s = wx.Slider(self.scrolledWindow)
                        s.SetRange(0, 1000)
                    else:
                        s = wx.SpinCtrlDouble(self.scrolledWindow)
                        s.SetRange(r[0], r[1])
                        s.SetIncrement(min(1, (r[1] - r[0]) / 100.0))
                        s.SetDigits(-math.log(s.GetIncrement()) / math.log(10) + 1)
                    sizer.Add( s, 0, wx.EXPAND)
                    self.controls[name] = s
                    sname = name
                    def onspin(event):
                        if useSlider:
                            v = s.GetValue() / 1000.0 * (r[1] - r[0]) + r[0]
                            self.client.set(sname, v)
                        else:
                            self.client.set(sname, s.GetValue())
                    if useSlider:
                        s.Bind( wx.EVT_SLIDER, onspin )
                        self.sliderrange[name] = r
                    else:
                        s.Bind( wx.EVT_SPINCTRLDOUBLE, onspin )
                proc()

            elif t == 'EnumProperty':
                def proc():
                    c = wx.Choice(self.scrolledWindow, wx.ID_ANY)
                    for choice in self.value_list[name]['choices']:
                        c.Append(str(choice))
                    sizer.Add( c, 0, wx.EXPAND)
                    self.controls[name] = c
                    cname = name
                    def onchoice(event):
                        self.client.set(cname, str(c.GetStringSelection()) )
                    c.Bind( wx.EVT_CHOICE, onchoice )
                proc()

            elif t == 'ResettableValue':
                def proc():
                    b = wx.Button(self.scrolledWindow, wx.ID_ANY, 'Reset')
                    sizer.Add( b, 0, wx.EXPAND)
                    bname = name
                    def onclick(event):
                        self.client.set(bname, 0)
                    b.Bind( wx.EVT_BUTTON, onclick)
                proc()

            else:
                sizer.Add( wx.StaticText(self.scrolledWindow, wx.ID_ANY, ''))

        self.scrolledWindow.SetSizer(sizer)
        self.scrolledWindow.Layout()

        sizer.Fit(self.scrolledWindow)
        ssizer.Add(self.scrolledWindow, 1, wx.EXPAND | wx.ALL, 5)

        bsizer = wx.FlexGridSizer(1, 0, 0, 0)
        self.bRefresh = wx.Button(self, wx.ID_ANY, 'Refresh')
        self.bRefresh.Bind( wx.EVT_BUTTON, self.Refresh )
        bsizer.Add(self.bRefresh)

        self.bScope = wx.Button(self, wx.ID_ANY, 'Scope')
        self.bScope.Bind( wx.EVT_BUTTON,
                          lambda event :
                          subprocess.Popen(['python',
                                            os.path.abspath(os.path.dirname(__file__)) +
                                            '/' + 'scope_wx.py'] + sys.argv[1:]))
        bsizer.Add(self.bScope)

        self.bClose = wx.Button(self, wx.ID_ANY, 'Close')
        self.bClose.Bind( wx.EVT_BUTTON, exit )
        bsizer.Add(self.bClose)

        ssizer.Add(bsizer, 1, wx.EXPAND)
        
        self.SetSizer(ssizer)
        self.Layout()

        self.timer = wx.Timer(self, wx.ID_ANY)
        self.timer.Start(500)
        self.Bind(wx.EVT_TIMER, self.receive_messages, id=wx.ID_ANY)

        self.Refresh()
        
    def Refresh(self):
        for name in self.value_list:
            self.client.get(name)

    def on_con(self, client):
        self.SetTitle("signalk client - Connected")
        for name in sorted(self.value_list):
            t = self.value_list[name]['type']
            if t != 'SensorValue':
                client.watch(name)
            else:
                client.get(name)
        
    def receive_messages(self, event):
        if not self.client:
            try:
                host, port = self.host_port
                self.client = SignalKClient(self.on_con, host, port, autoreconnect=False)
                self.timer.Start(100)
            except socket.error:
                self.timer.Start(1000)
                return

        while True:
            result = False
            try:
                result = self.client.receive()
            except ConnectionLost:
                self.SetTitle("signalk client - Disconnected")
                self.client = False
                return
            except:
                pass
            if not result:
                break

            for name in result:
                if not 'value' in result[name]:
                    print 'no value', result
                    raise 'no value'

                value = round3(result[name]['value'])

                strvalue = str(value)
                if len(strvalue) > 50:
                    strvalue = strvalue[:47] + '...'
                self.values[name].SetLabel(strvalue)

                if name in self.controls:
                    try:
                        if str(type(self.controls[name])) == "<class 'wx._controls.Choice'>":
                            if not self.controls[name].SetStringSelection(value):
                                print 'warning, invalid choice value specified'
                        elif str(type(self.controls[name])) == "<class 'wx._controls.Slider'>":
                            r = self.sliderrange[name]
                            self.controls[name].SetValue(float(value - r[0])/(r[1]-r[0])*1000)
                        else:
                            self.controls[name].SetValue(value)
                    except:
                        self.controls[name].SetValue(str(value))

                size = self.GetSize()
                self.Fit()
                self.SetSize(size)
            
def main():
    app = wx.App()
    MainFrame().Show()
    app.MainLoop()

if __name__ == '__main__':
  main()
