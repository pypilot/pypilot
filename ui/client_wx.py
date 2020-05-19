#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import wx, sys, math, subprocess, os, socket
from pypilot.client import pypilotClient

def round3(value):
    if type(value) == type([]):
        return list(map(round3, value))
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
        wx.Frame.__init__(self, None, title="pypilot client", size=(1000, 600))
        host = ''
        if len(sys.argv) > 1:
            host = sys.argv[1]
        self.client = pypilotClient(host)
        self.connected = False

        ssizer = wx.FlexGridSizer(0, 1, 0, 0)
        ssizer.AddGrowableRow( 0 )
        ssizer.AddGrowableCol( 0 )
        ssizer.SetFlexibleDirection( wx.BOTH )
        ssizer.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )

        self.scrolledWindow = wx.ScrolledWindow(self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize, wx.HSCROLL|wx.VSCROLL )
        self.scrolledWindow.SetScrollRate(5, 5)
        self.Refresh(None)

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
        
    def layout_widgets(self, value_list):
        sizer = self.scrolledWindow.GetSizer()
        if not sizer:
            sizer = wx.FlexGridSizer(0, 3, 0, 0)
            sizer.AddGrowableCol( 2 )
            sizer.SetFlexibleDirection( wx.BOTH )
            sizer.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )

        for name in sorted(value_list):
            t = value_list[name]['type']
            watch = True
            if t == 'SensorValue':
                watch=10 # update only every 10 seconds
            self.client.watch(name, watch)

        for name in sorted(value_list):
            if name in self.values:
                continue

            sizer.Add( wx.StaticText(self.scrolledWindow, wx.ID_ANY, name), 0, wx.ALL, 5 )
                    
            self.values[name] = wx.StaticText(self.scrolledWindow, wx.ID_ANY)
            sizer.Add( self.values[name], 0, wx.ALL, 5 )

            t = value_list[name]['type']

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
                        self.client.set(cbname, cb.GetValue())
                    cb.Bind( wx.EVT_CHECKBOX, oncheck )
                proc()

            elif t == 'RangeProperty' or t == 'RangeSetting':
                useSlider = True
                def proc():
                    r = value_list[name]['min'], value_list[name]['max']
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
                    for choice in value_list[name]['choices']:
                        c.Append(str(choice))
                    sizer.Add( c, 0, wx.EXPAND)
                    self.controls[name] = c
                    cname = name
                    def onchoice(event):
                        self.client.set(cname, str(c.GetStringSelection()))
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
        
    def Refresh(self, event):
        if self.client.connection:
            self.client.disconnect()
        sizer = self.scrolledWindow.GetSizer()
        if sizer:
            sizer.Clear(True)
        self.values = {}
        self.controls = {}
        self.sliderrange = {}
        
    def receive_messages(self, event):
        if self.client.connection != self.connected:
            self.connected = self.client.connection
            if self.connected:
                self.SetTitle("pypilot client - Connected")
            else:
                self.SetTitle("pypilot client - Disconnected")

        value_list = self.client.list_values()
        if value_list:
            self.layout_widgets(value_list)
                
        while True:
            result = self.client.receive()
            if not result:
                break

            for name in result:
                value = round3(result[name])

                strvalue = str(value)
                if len(strvalue) > 50:
                    strvalue = strvalue[:47] + '...'
                self.values[name].SetLabel(strvalue)

                if name in self.controls:
                    try:
                        t = str(type(self.controls[name]))
                        if t == "<class 'wx._controls.Choice'>" or t == "<class 'wx._core.Choice'>":
                            if not self.controls[name].SetStringSelection(value):
                                print('warning, invalid choice value specified')
                        elif t == "<class 'wx._controls.Slider'>" or t == "<class 'wx._core.Slider'>":
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
