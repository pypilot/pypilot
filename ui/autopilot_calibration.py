#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import tempfile, time, math, sys, subprocess, json, socket, os
import wx, wx.glcanvas
import autopilot_control_ui
import compass_calibration_plot, pypilot.quaternion, boatplot
import signalk.scope_wx
from signalk.client import SignalKClient, ConnectionLost
from signalk.client_wx import round3

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *


class CalibrationDialog(autopilot_control_ui.CalibrationDialogBase):
    ID_MESSAGES = 1000
    ID_CALIBRATE_SERVO = 1001

    def __init__(self):
        super(CalibrationDialog, self).__init__(None)
        self.host = ''
        if len(sys.argv) > 1:
            self.host = sys.argv[1]

        self.client = False

        self.compass_calibration_plot = compass_calibration_plot.CompassCalibrationPlot()
        self.compass_calibration_glContext =  wx.glcanvas.GLContext(self.CompassCalibration)

        self.boat_plot = boatplot.BoatPlot()
        self.boat_plot_glContext =  wx.glcanvas.GLContext(self.BoatPlot)

        self.dsServoMaxCurrent.SetIncrement(.1)
        self.dsServoMaxCurrent.SetDigits(1)
        self.dsServoMaxCurrent.Bind( wx.EVT_SPINCTRLDOUBLE, self.onMaxCurrent )

        self.lastmouse = False
        self.alignment_count = 0

        self.timer = wx.Timer(self, self.ID_MESSAGES)
        self.timer.Start(50)
        self.Bind(wx.EVT_TIMER, self.receive_messages, id=self.ID_MESSAGES)

        self.servo_timer = wx.Timer(self, self.ID_CALIBRATE_SERVO)
        self.Bind(wx.EVT_TIMER, self.calibrate_servo_timer, id=self.ID_CALIBRATE_SERVO)
        self.servoprocess = False

        self.alignmentQ = [1, 0, 0, 0]
        self.fusionQPose = [1, 0, 0, 0]

    def on_con(self, client):
        watchlist = ['imu/compass_calibration', 'imu/compass_calibration_age', \
                     'imu/compass', 'imu/compass_calibration_sigmapoints', \
                     'imu/accel', 'imu/fusionQPose', 'imu/alignmentCounter', \
                     'imu/heading', \
                     'imu/alignmentQ', 'imu/pitch', 'imu/roll', 'imu/heel', \
                     'imu/heading_offset', 'servo/calibration', \
                     'servo/Max Current']
        for name in watchlist:
            client.watch(name)

    def receive_messages(self, event):
        if not self.client:
            try:
                self.client = SignalKClient(self.on_con, self.host, autoreconnect=True)
            except socket.error:
                self.timer.Start(5000)
                return
        try:
            msg = self.client.receive_single()
            while msg:
                self.receive_message(msg)
                msg = self.client.receive_single()
            self.timer.Start(50)
        except ConnectionLost:
            self.client = False

    def receive_message(self, msg):
        name, data = msg
        value = data['value']

        self.compass_calibration_plot.read_data(msg)

        if name == 'imu/compass':
            self.CompassCalibration.Refresh()
        
        if name == 'imu/compass_calibration':
            self.stCompassCal.SetLabel(str(round3(value[0])))
            self.stCompassCalDeviation.SetLabel(str(round3(value[1])))
        elif name == 'imu/compass_calibration_age':
            self.stCompassCalAge.SetLabel(str(value))
        elif name == 'imu/alignmentQ':
            self.alignmentQ = value
            self.stAlignment.SetLabel(str(round3(value)) + ' ' + str(math.degrees(pypilot.quaternion.angle(self.alignmentQ))))
        elif name == 'imu/fusionQPose':
            if self.cCoords.GetSelection() == 1:
                self.boat_plot.Q = pypilot.quaternion.multiply(self.boat_plot.Q, self.fusionQPose)
                self.boat_plot.Q = pypilot.quaternion.multiply(self.boat_plot.Q, pypilot.quaternion.conjugate(value))
            elif self.cCoords.GetSelection() == 2:
                ang = pypilot.quaternion.toeuler(self.fusionQPose)[2] - pypilot.quaternion.toeuler(value)[2]
                self.boat_plot.Q = pypilot.quaternion.multiply(self.boat_plot.Q, pypilot.quaternion.angvec2quat(ang, [0, 0, 1]))

            self.fusionQPose = value
            self.BoatPlot.Refresh()
        elif name=='imu/alignmentCounter':
            self.gAlignment.SetValue(100 - value)

            enable = value == 0
            self.bLevel.Enable(enable)

        elif name == 'imu/pitch':
            self.stPitch.SetLabel(str(round3(value)))
        elif name == 'imu/roll':
            self.stRoll.SetLabel(str(round3(value)))
        elif name == 'imu/heel':
            self.stHeel.SetLabel(str(round3(value)))
        elif name == 'imu/heading':
            self.stHeading.SetLabel(str(round3(value)))
        elif name == 'imu/heading_offset':
            self.sHeadingOffset.SetValue(round3(value))
        elif name == 'servo/calibration':
            s = ''
            for cal in sorted(value):
                s += str(round3(float(cal))) + ' = ' + str(round3(value[cal])) + '\n'
            self.stServoCalibration.SetLabel(s)
            self.SetSize(wx.Size(self.GetSize().x+1, self.GetSize().y))
            
            self.servo_calibration = {}
            for cal in value:
                self.servo_calibration[float(cal)] = value[cal]

            self.bServoPlot.Enable()
        elif name == 'servo/calibration console':
            self.stServoCalibrationConsole.SetLabel(self.stServoCalibrationConsole.GetLabel() + value)
        elif name == 'servo/Max Current':
            self.dsServoMaxCurrent.SetValue(round3(value))


    def servo_console(self, text):
        self.stServoCalibrationConsole.SetLabel(self.stServoCalibrationConsole.GetLabel() + text + '\n')
    
    def calibrate_servo_timer(self, event):
        if not self.servoprocess:
            return

        self.servoprocess.poll()

        print 'servotimer', self.servoprocess.returncode, self.servoprocess.returncode==None
        if self.servoprocess.returncode == None:
            self.servoprocess.communicate()
            line = self.servoprocess.stdout.readline()
            print 'line', line
            if line:
                self.servo_console(line)
            self.servo_timer.Start(150, True)
        else:
            self.bCalibrateServo.Enable()
            if self.servoprocess.returncode == 0:
                file = open('servo_calibration')
                calibration = json.loads(file.readline())
                self.client.set('servo/calibration', calibration)
                self.servo_console('calibration sent.')
            else:
                self.servo_console('calibration failed.')

            self.servoprocess = False

    def onKeyPressCompass( self, event ):
        signalk.scope_wx.wxglutkeypress(event, self.compass_calibration_plot.special, \
                                     self.compass_calibration_plot.key)

    def onClearCompass( self, event ):
        self.compass_calibration_plot.points = []

    def onMouseEventsCompass( self, event ):
        self.CompassCalibration.SetFocus()

        pos = event.GetPosition()
        if event.LeftDown():
            self.lastmouse = pos

        if event.Dragging():
            compass_calibration_plot.rotate_mouse(pos[0] - self.lastmouse[0], \
                                                  pos[1] - self.lastmouse[1])
            self.CompassCalibration.Refresh()
            self.lastmouse = pos

        rotation = event.GetWheelRotation() / 60
        if rotation:
            self.CompassCalibration.Refresh()
        while rotation > 0:
            self.compass_calibration_plot.userscale /= .9
            rotation -= 1
        while rotation < 0:
            self.compass_calibration_plot.userscale *= .9
            rotation += 1
	
    def onPaintGLCompass( self, event ):
        wx.PaintDC( self.CompassCalibration )
        self.CompassCalibration.SetCurrent(self.compass_calibration_glContext)
        self.compass_calibration_plot.display()
        self.CompassCalibration.SwapBuffers()

    def onSizeGLCompass( self, event ):
        self.compass_calibration_plot.reshape(event.GetSize().x, event.GetSize().y)

    def StartAlignment(self):
        self.client.set('imu/alignmentCounter', 100)

    def onResetAlignment(self, event):
        self.client.set('imu/alignmentQ', [1, 0, 0, 0])

    def onLevel( self, event ):
        self.StartAlignment()
	
    def onIMUHeadingOffset( self, event ):
        self.client.set('imu/heading_offset', self.sHeadingOffset.GetValue())

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
            dx, dy = pos[0] - self.lastmouse[0], pos[1] - self.lastmouse[1]
            q = pypilot.quaternion.angvec2quat((dx**2 + dy**2)**.4/180*math.pi, [dy, dx, 0])
            
            self.boat_plot.Q = pypilot.quaternion.multiply(q, self.boat_plot.Q)
            self.BoatPlot.Refresh()
            self.lastmouse = pos

        rotation = event.GetWheelRotation() / 60
        if rotation:
            self.BoatPlot.Refresh()
        while rotation > 0:
            self.boat_plot.Scale /= .9
            rotation -= 1
        while rotation < 0:
            self.boat_plot.Scale *= .9
            rotation += 1
            
    def onPaintGLBoatPlot( self, event ):
        wx.PaintDC( self.CompassCalibration )
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
        host, port = self.client.host_port
        args = ['python', os.path.abspath(os.path.dirname(__file__)) + '/../signalk/scope_wx.py', host + ':' + str(port),
                'imu/pitch', 'imu/roll', 'imu/heel', 'imu/heading']
        subprocess.Popen(args)
	
    def onServoPlot( self, event ):
        plots = [tempfile.NamedTemporaryFile(), tempfile.NamedTemporaryFile(), \
                 tempfile.NamedTemporaryFile()]
        ind = 0
        for plot in plots:
            file = open(plot.name, 'w')
            for cal in sorted(self.servo_calibration):
                file.write('%f %f\n' % (cal, self.servo_calibration[cal][ind]))
            file.close()
            xlabel = 'speed'
            ylabel = ['command', 'idle_current', 'stall_current', 'cal_voltage', 'dt'][ind]
            
            ind += 1
            subprocess.Popen(['gnuplot', '-p', \
                              '-e', 'set xlabel "' + xlabel + '"',
                              '-e', 'set ylabel "' + ylabel + '"',
                              '-e', 'plot \'' + plot.name + '\' with lines, \'' + plot.name + '\' with points'])

            # all data in one plot
#        cmd = 'plot '
#        for plot in plots:
#            cmd += "'" + plot.name + "' with lines, '" + plot.name + "' with points, "

            #subprocess.Popen(['gnuplot', '-p', '-e', cmd])

        time.sleep(1)
	
    def onCalibrateServo( self, event ):
#        self.stServoCalibrationConsole.SetLabel('')
        try:
	    self.servoprocess = subprocess.Popen(['python', 'servo_calibration.py', sys.argv[1]], stdout=subprocess.PIPE)
            self.servo_console('executed servo_calibration.py...')
            self.servo_timer.Start(150, True)
            self.bCalibrateServo.Disable()
        except OSError:
            self.servo_console('Failed to execute servo_calibration.py.\n')

    def onMaxCurrent( self, event ):
        self.client.set('servo/Max Current', event.GetValue())

if __name__ == "__main__":
    glutInit(sys.argv)
    app = wx.App()
    CalibrationDialog().ShowModal()
