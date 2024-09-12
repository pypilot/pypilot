#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import tempfile, time, math, sys, subprocess, json, socket, os
import wx, wx.glcanvas
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import calibration_plot, boatplot, autopilot_control_ui
from pypilot import quaternion
import scope_wx
from pypilot.client import pypilotClient
from client_wx import round3

from OpenGL.GL import *
from OpenGL.GLU import *


class CalibrationDialog(autopilot_control_ui.CalibrationDialogBase):
    ID_MESSAGES = 1000
    ID_CALIBRATE_SERVO = 1001
    ID_HEADING_OFFSET = 1002

    def __init__(self):
        super(CalibrationDialog, self).__init__(None)
        self.host = ''
        if len(sys.argv) > 1:
            self.host = sys.argv[1]

        self.client = False
        self.accel_calibration_plot = calibration_plot.AccelCalibrationPlot()
        self.accel_calibration_glContext =  wx.glcanvas.GLContext(self.AccelCalibration)

        self.compass_calibration_plot = calibration_plot.CompassCalibrationPlot()
        self.compass_calibration_glContext =  wx.glcanvas.GLContext(self.CompassCalibration)

        self.boat_plot = boatplot.BoatPlot()
        self.boat_plot_glContext =  wx.glcanvas.GLContext(self.BoatPlot)
        
        self.lastmouse = False
        self.alignment_count = 0

        self.timer = wx.Timer(self, self.ID_MESSAGES)
        self.timer.Start(50)
        self.Bind(wx.EVT_TIMER, self.receive_messages, id=self.ID_MESSAGES)

        self.heading_offset_timer = wx.Timer(self, self.ID_HEADING_OFFSET)
        self.Bind(wx.EVT_TIMER, lambda e : self.sHeadingOffset.SetValue(round3(self.pypilot_heading_offset)), id = self.ID_HEADING_OFFSET)

        self.have_rudder = False

        self.fusionQPose = [1, 0, 0, 0]
        self.alignmentQ = [1, 0, 0, 0]        
        self.controltimes = {}

        self.client = pypilotClient(self.host)

        # clear out plots
        self.accel_calibration_plot.points = []
        self.compass_calibration_plot.points = []
        self.settings = {}
        self.set_watches()

    def set_watches(self):
        if not self.client:
            return

        def calwatch(name):
            name = 'imu.' + name
            return [name + '.calibration', name + '.calibration.age',
                    (name, .2), name + '.calibration.sigmapoints', name + '.calibration.points',
                    name + '.calibration.locked', name + '.calibration.log']
        
        watchlist = [
            ['imu.fusionQPose', ('imu.alignmentCounter', .2), ('imu.heading', .5),
             ('imu.alignmentQ', 1), ('imu.pitch', .5), ('imu.roll', .5), ('imu.heel', .5), ('imu.heading_offset', 1)],
            calwatch('accel'),
            calwatch('compass') + ['imu.fusionQPose'],
            ['rudder.offset', 'rudder.scale', 'rudder.nonlinearity', ('rudder.angle', 1),
             'rudder.range', 'servo.flags'], list(self.settings)]
            
        pageindex = self.m_notebook.GetSelection()
        watches = {}
        for i in range(len(watchlist)):
            pagelist = watchlist[i]
            for name in watchlist[pageindex]:
                if i == pageindex:
                    if type(name) == type(()):
                        name, watch = name # if a pair, it specifies the period
                    else:
                        watch = True
                    watches[name] = watch
                elif name in self.client.watches:
                    self.client.watch(name, False)

        for name, watch in watches.items():
            self.client.watch(name, watch)

    def enumerate_settings(self, values):
        fgSettings = self.m_pSettings.GetSizer()
        if not fgSettings:
            fgSettings = wx.FlexGridSizer( 0, 3, 0, 0 )
            fgSettings.AddGrowableCol( 1 )
            fgSettings.SetFlexibleDirection( wx.BOTH )
            fgSettings.SetNonFlexibleGrowMode( wx.FLEX_GROWMODE_SPECIFIED )

            self.m_pSettings.SetSizer( fgSettings )

        lvalues = list(values)
        lvalues.sort()
        for name in lvalues:
            if name in self.settings:
                continue
            if 'units' in values[name]:
                v = values[name]
                def proc():
                    s = wx.SpinCtrlDouble(self.m_pSettings, wx.ID_ANY)
                    s.SetRange(v['min'], v['max'])
                    s.SetIncrement(min(1, (v['max'] - v['min']) / 100.0))
                    s.SetDigits(-math.log(s.GetIncrement()) / math.log(10) + 1)
                    self.settings[name] = s
                    fgSettings.Add(wx.StaticText(self.m_pSettings, wx.ID_ANY, name), 0, wx.ALL, 5)
                    fgSettings.Add(s, 0, wx.ALL | wx.EXPAND, 5)
                    fgSettings.Add(wx.StaticText(self.m_pSettings, wx.ID_ANY, v['units']), 0, wx.ALL, 5)
                    
                    sname = name
                    def onspin(event):
                        self.client.set(sname, s.GetValue())
                    s.Bind( wx.EVT_SPINCTRLDOUBLE, onspin )
                proc()

        self.m_pSettings.Layout()
        fgSettings.Fit( self.m_pSettings )

        #fgSettings.Add( ( 0, 0), 1, wx.EXPAND, 5 )
        #fgSettings.Add( ( 0, 0), 1, wx.EXPAND, 5 )
        #b = wx.Button( self.m_pSettings, wx.ID_OK )
        #fgSettings.Add ( b, 1, wx.ALIGN_RIGHT, 5)
            
    def receive_messages(self, event):
        self.client.poll()

        values_list = self.client.list_values()
        if values_list:
            self.enumerate_settings(values_list)

        msg = self.client.receive_single()
        while msg:
            self.receive_message(msg)
            msg = self.client.receive_single()
        self.timer.Start(50)
        return
            
        try:
            msg = self.client.receive_single()
            while msg:
                self.receive_message(msg)
                msg = self.client.receive_single()
            self.timer.Start(50)
        except Exception as e:
            print(_('exception in calibration:'), e)

    def UpdateControl(self, control, update):
        t = time.monotonic()
        if not control in self.controltimes or t - self.controltimes[control] > .5:
            update()
            self.controltimes[control] = t
                
    def UpdateLabel(self, label, value):
        self.UpdateControl(label, lambda : label.SetLabel(str(value)))

    def UpdatedSpin(self, dspin, value):
        self.UpdateControl(dspin, lambda : dspin.SetValue(int(value)))
                
    def receive_message(self, msg):
        name, value = msg

        if 1:
        #if self.m_notebook.GetSelection() == 0:
            if name == 'imu.alignmentQ':
                self.stAlignment.SetLabel(str(value) + ' ' + str(round3(math.degrees(quaternion.angle(value)))))
                self.alignmentQ = value
            elif name == 'imu.fusionQPose':                
                if not value:
                    return # no imu!  show warning?

                #lastaligned = quaternion.normalize(quaternion.multiply(self.fusionQPose, self.alignmentQ))
                aligned = quaternion.normalize(quaternion.multiply(value, self.alignmentQ))
                value = aligned
                    
                if self.cCoords.GetSelection() == 1:
                    #self.boat_plot.Q = quaternion.multiply(self.boat_plot.Q, lastedaligned)
                    self.boat_plot.Q = quaternion.multiply(self.boat_plot.Q, self.fusionQPose)
                    self.boat_plot.Q = quaternion.multiply(self.boat_plot.Q, quaternion.conjugate(aligned))
                elif self.cCoords.GetSelection() == 2:
                    ang = quaternion.toeuler(self.fusionQPose)[2] - quaternion.toeuler(aligned)[2]
                    self.boat_plot.Q = quaternion.multiply(self.boat_plot.Q, quaternion.angvec2quat(ang, [0, 0, 1]))

                self.fusionQPose = value
                self.BoatPlot.Refresh()
            elif name=='imu.alignmentCounter':
                self.gAlignment.SetValue(100 - value)
                enable = value == 0
                self.bLevel.Enable(enable)
            elif name == 'imu.pitch':
                self.stPitch.SetLabel(str(round3(value)))
            elif name == 'imu.roll':
                self.stRoll.SetLabel(str(round3(value)))
            elif name == 'imu.heel':
                self.stHeel.SetLabel(str(round3(value)))
            elif name == 'imu.heading':
                self.stHeading.SetLabel(str(round3(value)))
            elif name == 'imu.heading_offset':
                self.pypilot_heading_offset = value
                self.heading_offset_timer.Start(1000, True)

        #elif self.m_notebook.GetSelection() == 1:
            self.accel_calibration_plot.read_data(msg)
            if name == 'imu.accel':
                self.AccelCalibration.Refresh()
            elif name == 'imu.accel.calibration':
                self.stAccelCal.SetLabel(str(round3(value)))
            elif name == 'imu.accel.calibration.age':
                self.stAccelCalAge.SetLabel(str(value))
            elif name == 'imu.accel.calibration.locked':
                self.cbAccelCalibrationLocked.SetValue(value)
            elif name == 'imu.accel.calibration.log':
                self.tAccelCalibrationLog.WriteText(value+'\n')

        #elif self.m_notebook.GetSelection() == 2:
            self.compass_calibration_plot.read_data(msg)
            if name == 'imu.compass':
                self.CompassCalibration.Refresh()
            elif name == 'imu.compass.calibration':
                self.stCompassCal.SetLabel(str(round3(value)))
            elif name == 'imu.compass.calibration.age':
                self.stCompassCalAge.SetLabel(str(value))
            elif name == 'imu.compass.calibration.locked':
                self.cbCompassCalibrationLocked.SetValue(value)
            elif name == 'imu.compass.calibration.log':
                self.tCompassCalibrationLog.WriteText(value+'\n')
        
        #elif self.m_notebook.GetSelection() == 3:
            if name == 'rudder.angle':
                self.UpdateLabel(self.stRudderAngle, str(round3(value)))
                self.have_rudder = type(value) != type(bool)
            elif name == 'rudder.offset':
                self.UpdateLabel(self.stRudderOffset, str(round3(value)))
            elif name == 'rudder.scale':
                self.UpdateLabel(self.stRudderScale, (str(round3(value))))
            elif name == 'rudder.nonlinearity':
                self.UpdateLabel(self.stRudderNonlinearity, str(round3(value)))
            elif name == 'rudder.range':
                self.UpdatedSpin(self.sRudderRange, value)
            elif name == 'servo.flags':
                self.stServoFlags.SetLabel(value)

        #elif self.m_notebook.GetSelection() == 4:
            if name in self.settings:
                self.UpdatedSpin(self.settings[name], value)

    def servo_console(self, text):
        self.stServoCalibrationConsole.SetLabel(self.stServoCalibrationConsole.GetLabel() + text + '\n')
    
    def PageChanged( self, event ):
        self.set_watches()
            
    def onKeyPressAccel( self, event ):
        self.onKeyPress(event, self.compass_calibration_plot)

    def onKeyPressCompass( self, event ):
        self.onKeyPress(event, self.compass_calibration_plot)
        
    def onKeyPress( self, event, plot ):
        scope_wx.wxglutkeypress(event, plot.special, plot.key)

    def onClearAccel( self, event ):
        self.accel_calibration_plot.points = []

    def onClearCompass( self, event ):
        self.compass_calibration_plot.points = []
        
    def onAccelCalibrationLocked( self, event ):
        self.client.set('imu.accel.calibration.locked', self.cbAccelCalibrationLocked.GetValue())

    def onCompassCalibrationLocked( self, event ):
        self.client.set('imu.compass.calibration.locked', self.cbCompassCalibrationLocked.GetValue())

    def onCalibrationLocked( self, sensor, ctrl ):
        self.client.set('imu.'+sensor+'.calibration.locked', self.ctrl.GetValue())

    def onMouseEventsAccel( self, event ):
        self.AccelCalibration.SetFocus()
        self.onMouseEvents( event, self.AccelCalibration, self.accel_calibration_plot )
        
    def onMouseEventsCompass( self, event ):
        self.CompassCalibration.SetFocus()
        self.onMouseEvents( event, self.CompassCalibration, self.compass_calibration_plot )
        
    def onMouseEvents( self, event, canvas, plot ):
        pos = event.GetPosition()
        if event.LeftDown():
            self.lastmouse = pos

        if event.Dragging():
            if self.lastmouse:
                calibration_plot.rotate_mouse(pos[0] - self.lastmouse[0],  pos[1] - self.lastmouse[1])
                canvas.Refresh()
            self.lastmouse = pos

        rotation = event.GetWheelRotation() / 60
        if rotation:
            canvas.Refresh() # set context
            while rotation > 0:
                plot.userscale /= .9
                rotation -= 1
            while rotation < 0:
                plot.userscale *= .9
                rotation += 1
            plot.userscale = min(max(plot.userscale, .001), 1)
        
    def onPaintGLAccel( self, event ):
        self.onPaintGL( self.AccelCalibration, self.accel_calibration_plot, self.accel_calibration_glContext )

    def onPaintGLCompass( self, event ):
        self.onPaintGL( self.CompassCalibration, self.compass_calibration_plot, self.compass_calibration_glContext )

    def onPaintGL( self, canvas, plot, context ):
        wx.PaintDC( canvas )
        canvas.SetCurrent(context)
        plot.display()
        canvas.SwapBuffers()

    def onSizeGLAccel( self, event ):
        self.accel_calibration_plot.reshape(event.GetSize().x, event.GetSize().y)

    def onSizeGLCompass( self, event ):
        self.compass_calibration_plot.reshape(event.GetSize().x, event.GetSize().y)

    def onResetAlignment(self, event):
        self.client.set('imu.alignmentQ', False)

    def onLevel( self, event ):
        self.client.set('imu.alignmentCounter', 100)
        
    def onIMUHeadingOffset( self, event ):
        self.client.set('imu.heading_offset', self.sHeadingOffset.GetValue())
        self.heading_offset_timer.Stop()
        
    def onKeyPressBoatPlot( self, event ):
        self.BoatPlot.SetFocus()
        k = '%c' % (event.GetKeyCode()&255)
        if not event.GetModifiers() & wx.MOD_SHIFT:
            k = k.lower()
        self.BoatPlot.Refresh()

    def onMouseEventsBoatPlot( self, event ):
        self.BoatPlot.SetFocus()

        pos = event.GetPosition()
        if event.LeftDown():
            self.lastmouse = pos

        if event.Dragging():
            if self.lastmouse:
                self.BoatPlot.Refresh()
                dx, dy = pos[0] - self.lastmouse[0], pos[1] - self.lastmouse[1]
                q = quaternion.angvec2quat((dx**2 + dy**2)**.4/180*math.pi, [dy, dx, 0])
                self.boat_plot.Q = quaternion.multiply(q, self.boat_plot.Q)
            self.lastmouse = pos

        rotation = event.GetWheelRotation() / 60
        if rotation:
            while rotation > 0:
                self.boat_plot.Scale /= .9
                rotation -= 1
            while rotation < 0:
                self.boat_plot.Scale *= .9
                rotation += 1
            self.BoatPlot.Refresh()
            
    def onPaintGLBoatPlot( self, event ):
        wx.PaintDC( self.BoatPlot )
        self.BoatPlot.SetCurrent(self.boat_plot_glContext)

        # stupid hack
        self.boat_plot.reshape(self.BoatPlot.GetSize().x, self.BoatPlot.GetSize().y)

        self.boat_plot.display(self.fusionQPose)
        self.BoatPlot.SwapBuffers()

    def onSizeGLBoatPlot( self, event ):
        self.boat_plot.reshape(event.GetSize().x, event.GetSize().y)
        self.BoatPlot.Refresh()

    def onTextureCompass( self, event ):
        self.boat_plot.texture_compass = event.IsChecked()
        self.BoatPlot.Refresh()

    def onIMUScope( self, event ):
        host = self.client.config['host']
        args = ['pypilot_scope', host,
                'imu.pitch', 'imu.roll', 'imu.heel', 'imu.heading']
        subprocess.Popen(args)

    def onRudderResetCalibration( self, event ):
        self.client.set('rudder.calibration_state', 'reset')

    def onRudderCentered( self, event ):
        self.client.set('rudder.calibration_state', 'centered')

    def onRudderStarboardRange( self, event ):
        self.client.set('rudder.calibration_state', 'starboard range')

    def onRudderPortRange( self, event ):
        self.client.set('rudder.calibration_state', 'port range')

    def onRudderRange( self, event ):
        self.client.set('rudder.range', self.sRudderRange.GetValue())

def main():
    app = wx.App()
    
    CalibrationDialog().ShowModal()

if __name__ == "__main__":
    main()
