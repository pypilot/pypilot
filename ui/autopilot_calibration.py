#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import tempfile, time, math, sys, subprocess, json, socket, os
import wx, wx.glcanvas
import autopilot_control_ui
import calibration_plot, pypilot.quaternion, boatplot
import signalk.scope_wx
from signalk.client import SignalKClient, ConnectionLost
from signalk.client_wx import round3

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *


class CalibrationDialog(autopilot_control_ui.CalibrationDialogBase):
    ID_MESSAGES = 1000
    ID_CALIBRATE_SERVO = 1001
    ID_HEADING_OFFSET = 1002
    ID_REQUEST_MSG = 1003

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

        self.dsServoMaxCurrent.SetIncrement(.1)
        self.dsServoMaxCurrent.SetDigits(1)
        self.dsServoMaxCurrent.Bind( wx.EVT_SPINCTRLDOUBLE, self.onMaxCurrent )

        self.dsServoMaxControllerTemp.SetRange(45, 100)
        self.dsServoMaxControllerTemp.Bind( wx.EVT_SPINCTRL, self.onMaxControllerTemp )

        self.dsServoMaxMotorTemp.SetRange(30, 100)
        self.dsServoMaxMotorTemp.Bind( wx.EVT_SPINCTRL, self.onMaxMotorTemp )

        self.dsServoCurrentFactor.SetRange(.8, 1.2)
        self.dsServoCurrentFactor.SetIncrement(.0016)
        self.dsServoCurrentFactor.SetDigits(4)
        self.dsServoCurrentFactor.Bind( wx.EVT_SPINCTRLDOUBLE, self.onCurrentFactor )

        self.dsServoCurrentOffset.SetRange(-1.2, 1.2)
        self.dsServoCurrentOffset.SetIncrement(.01)
        self.dsServoCurrentOffset.SetDigits(2)
        self.dsServoCurrentOffset.Bind( wx.EVT_SPINCTRLDOUBLE, self.onCurrentOffset )

        self.dsServoVoltageFactor.SetRange(.8, 1.2)
        self.dsServoVoltageFactor.SetIncrement(.0016)
        self.dsServoVoltageFactor.SetDigits(4)
        self.dsServoVoltageFactor.Bind( wx.EVT_SPINCTRLDOUBLE, self.onVoltageFactor )

        self.dsServoVoltageOffset.SetRange(-1.2, 1.2)
        self.dsServoVoltageOffset.SetIncrement(.01)
        self.dsServoVoltageOffset.SetDigits(2)
        self.dsServoVoltageOffset.Bind( wx.EVT_SPINCTRLDOUBLE, self.onVoltageOffset )

        self.dsServoMaxSlewSpeed.SetRange(0, 100)
        self.dsServoMaxSlewSpeed.Bind( wx.EVT_SPINCTRL, self.onMaxSlewSpeed )

        self.dsServoMaxSlewSlow.SetRange(0, 100)
        self.dsServoMaxSlewSlow.Bind( wx.EVT_SPINCTRL, self.onMaxSlewSlow )

        self.dsServoGain.SetIncrement(.1)
        self.dsServoGain.SetDigits(1)
        self.dsServoGain.Bind( wx.EVT_SPINCTRLDOUBLE, self.onServoGain )
        
        self.lastmouse = False
        self.alignment_count = 0

        self.timer = wx.Timer(self, self.ID_MESSAGES)
        self.timer.Start(50)
        self.Bind(wx.EVT_TIMER, self.receive_messages, id=self.ID_MESSAGES)

        self.heading_offset_timer = wx.Timer(self, self.ID_HEADING_OFFSET)
        self.Bind(wx.EVT_TIMER, lambda e : self.sHeadingOffset.SetValue(round3(self.signalk_heading_offset)), id = self.ID_HEADING_OFFSET)

        self.servo_timer = wx.Timer(self, self.ID_CALIBRATE_SERVO)
        self.Bind(wx.EVT_TIMER, self.calibrate_servo_timer, id=self.ID_CALIBRATE_SERVO)

        self.have_rudder = False

        self.request_msg_timer = wx.Timer(self, self.ID_REQUEST_MSG)
        self.Bind(wx.EVT_TIMER, self.request_msg, id=self.ID_REQUEST_MSG)
        self.request_msg_timer.Start(250)
        
        self.servoprocess = False

        self.fusionQPose = [1, 0, 0, 0]
        self.controltimes = {}

    def set_watches(self, client):
        watchlist = [
            ['imu.fusionQPose', 'imu.alignmentCounter', 'imu.heading',
             'imu.alignmentQ', 'imu.pitch', 'imu.roll', 'imu.heel', 'imu.heading_offset'],
            ['imu.accel.calibration', 'imu.accel.calibration.age', 'imu.accel',
             'imu.accel.calibration.sigmapoints', 'imu.accel.calibration.locked'],
            ['imu.compass.calibration', 'imu.compass.calibration.age', 'imu.compass',
             'imu.compass.calibration.sigmapoints', 'imu.compass.calibration.locked'],
            ['servo.flags',
             'servo.max_current', 'servo.max_controller_temp', 'servo.max_motor_temp',
             'servo.current.factor', 'servo.current.offset','servo.voltage.factor',
             'servo.voltage.offset', 'servo.max_slew_speed', 'servo.max_slew_slow'],
            ['rudder.offset', 'rudder.scale', 'rudder.nonlinearity',
             'rudder.range', 'rudder.calibration_state']]

        pageindex = 0
        for pagelist in watchlist:
            watch = self.m_notebook.GetSelection() == pageindex            
            for name in pagelist:
                client.watch(name, watch)
            pageindex+=1
        
    def on_con(self, client):
        self.set_watches(client)

    def receive_messages(self, event):
        if not self.client:
            try:
                self.client = SignalKClient(self.on_con, self.host, autoreconnect=False)
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

    def request_msg(self, event):
        if not self.client:
            return

        page_gets = [[], [], [], ['servo.voltage', 'servo.current'], ['rudder.angle']]
        i = self.m_notebook.GetSelection()
        for name in page_gets[i]:
            self.client.get(name)

    def UpdateControl(self, control, update):
        t = time.time()
        if not control in self.controltimes or t - self.controltimes[control] > .5:
            update()
            self.controltimes[control] = t
                
    def UpdateLabel(self, label, value):
        self.UpdateControl(label, lambda : label.SetLabel(str(value)))

    def UpdatedSpin(self, dspin, value):
        self.UpdateControl(dspin, lambda : dspin.SetValue(value))
                
    def receive_message(self, msg):
        name, data = msg
        value = data['value']

        if self.m_notebook.GetSelection() == 1:
            self.accel_calibration_plot.read_data(msg)
            if name == 'imu.accel':
                self.AccelCalibration.Refresh()
            elif name == 'imu.accel.calibration':
                self.stAccelCal.SetLabel(str(round3(value)))
            elif name == 'imu.accel.calibration.age':
                self.stAccelCalAge.SetLabel(str(value))
            elif name == 'imu.accel.calibration.locked':
                self.cbAccelCalibrationLocked.SetValue(value)

        elif self.m_notebook.GetSelection() == 2:
            self.compass_calibration_plot.read_data(msg)
            if name == 'imu.compass':
                self.CompassCalibration.Refresh()
            elif name == 'imu.compass.calibration':
                self.stCompassCal.SetLabel(str(round3(value[0])))
            elif name == 'imu.compass.calibration.age':
                self.stCompassCalAge.SetLabel(str(value))
            elif name == 'imu.compass.calibration.locked':
                self.cbCompassCalibrationLocked.SetValue(value)

        elif self.m_notebook.GetSelection() == 0:
            if name == 'imu.alignmentQ':
                self.stAlignment.SetLabel(str(round3(value)) + ' ' + str(math.degrees(pypilot.quaternion.angle(value))))
            elif name == 'imu.fusionQPose':
                if self.cCoords.GetSelection() == 1:
                    self.boat_plot.Q = pypilot.quaternion.multiply(self.boat_plot.Q, self.fusionQPose)
                    self.boat_plot.Q = pypilot.quaternion.multiply(self.boat_plot.Q, pypilot.quaternion.conjugate(value))
                elif self.cCoords.GetSelection() == 2:
                    ang = pypilot.quaternion.toeuler(self.fusionQPose)[2] - pypilot.quaternion.toeuler(value)[2]
                    self.boat_plot.Q = pypilot.quaternion.multiply(self.boat_plot.Q, pypilot.quaternion.angvec2quat(ang, [0, 0, 1]))

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
                self.signalk_heading_offset = value
                self.heading_offset_timer.Start(1000, True)

        elif self.m_notebook.GetSelection() == 3:
            if name == 'servo.flags':
                self.UpdateLabel(self.stServoFlags, value)
            elif name == 'servo.calibration':
                s = ''
                for name in value:
                    s += name + ' = ' + str(value[name]) + '\n'
                self.stServoCalibration.SetLabel(s)
                self.SetSize(wx.Size(self.GetSize().x+1, self.GetSize().y))
            elif name == 'servo.max_current':
                self.UpdatedSpin(self.dsServoMaxCurrent, round3(value))
            elif name == 'servo.max_controller_temp':
                self.UpdatedSpin(self.dsServoMaxControllerTemp, value)
            elif name == 'servo.max_motor_temp':
                self.UpdatedSpin(self.dsServoMaxMotorTemp, value)
            elif name == 'servo.current.factor':
                self.UpdatedSpin(self.dsServoCurrentFactor, round(value, 4))
            elif name == 'servo.current.offset':
                self.UpdatedSpin(self.dsServoCurrentOffset, value)
            elif name == 'servo.voltage.factor':
                self.UpdatedSpin(self.dsServoVoltageFactor, round(value, 4))
            elif name == 'servo.voltage.offset':
                self.UpdatedSpin(self.dsServoVoltageOffset, value)
            elif name == 'servo.max_slew_speed':
                self.UpdatedSpin(self.dsServoMaxSlewSpeed, value)
            elif name == 'servo.max_slew_slow':
                self.UpdatedSpin(self.dsServoMaxSlewSlow, value)
            elif name == 'servo.voltage':
                self.UpdateLabel(self.m_stServoVoltage, (str(round3(value))))
            elif name == 'servo.current':
                self.UpdateLabel(self.m_stServoCurrent, (str(round3(value))))

        elif self.m_notebook.GetSelection() == 4:
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
                self.client.set('servo.calibration', calibration)
                self.servo_console('calibration sent.')
            else:
                self.servo_console('calibration failed.')

            self.servoprocess = False

    def PageChanged( self, event ):
        self.set_watches(self.client)
            
    def onKeyPressAccel( self, event ):
        self.onKeyPress(event, self.compass_calibration_plot)

    def onKeyPressCompass( self, event ):
        self.onKeyPress(event, self.compass_calibration_plot)
        
    def onKeyPress( self, event, plot ):
        signalk.scope_wx.wxglutkeypress(event, plot.special, plot.key)

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
            calibration_plot.rotate_mouse(pos[0] - self.lastmouse[0], \
                                          pos[1] - self.lastmouse[1])
            canvas.Refresh()
            self.lastmouse = pos

        rotation = event.GetWheelRotation() / 60
        if rotation:
            canvas.Refresh()
        while rotation > 0:
            plot.userscale /= .9
            rotation -= 1
        while rotation < 0:
            plot.userscale *= .9
            rotation += 1
        
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

    def StartAlignment(self):
        self.client.set('imu.alignmentCounter', 100)

    def onResetAlignment(self, event):
        self.client.set('imu.alignmentQ', [1, 0, 0, 0])

    def onLevel( self, event ):
        self.StartAlignment()
        
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
        host, port = self.client.host_port
        args = ['python', os.path.abspath(os.path.dirname(__file__)) + '/../signalk/scope_wx.py', host + ':' + str(port),
                'imu.pitch', 'imu.roll', 'imu.heel', 'imu.heading']
        subprocess.Popen(args)
        
    def onCalibrateServo( self, event ):
        try:
            self.servoprocess = subprocess.Popen(['python', 'servo_calibration.py', sys.argv[1]], stdout=subprocess.PIPE)
            self.servo_console('executed servo_calibration.py...')
            self.servo_timer.Start(150, True)
            self.bCalibrateServo.Disable()
        except OSError:
            self.servo_console('Failed to execute servo_calibration.py.\n')

    def onMaxCurrent( self, event ):
        self.client.set('servo.max_current', event.GetValue())

    def onMaxControllerTemp( self, event ):
        self.client.set('servo.max_controller_temp', event.GetValue())

    def onMaxMotorTemp( self, event ):
        self.client.set('servo.max_motor_temp', event.GetValue())

    def onCurrentFactor( self, event ):
        self.client.set('servo.current.factor', event.GetValue())

    def onCurrentOffset( self, event ):
        self.client.set('servo.current.offset', event.GetValue())

    def onVoltageFactor( self, event ):
        self.client.set('servo.voltage.factor', event.GetValue())

    def onVoltageOffset( self, event ):
        self.client.set('servo.voltage.offset', event.GetValue())

    def onMaxSlewSpeed( self, event ):
        self.client.set('servo.max_slew_speed', event.GetValue())

    def onMaxSlewSlow( self, event ):
        self.client.set('servo.max_slew_slow', event.GetValue())

    def onServoGain( self, event ):
        self.client.set('servo.gain', event.GetValue())

    def onServoAutoGain( self, event ):
        if self.have_rudder:
            self.client.set('rudder.calibration_state', 'auto gain')
        else:
            wx.MessageDialog(self, _('Auto gain calibration requires a rudder sensor'), _('Warning'), wx.OK).ShowModal()
    def onRudderResetCalibration( self, event ):
        self.client.set('rudder.calibration_state', 'reset')

    def onRudderCentered( self, event ):
        self.client.set('rudder.calibration_state', 'centered')

    def onRudderStarboardRange( self, event ):
        self.client.set('rudder.calibration_state', 'starboard range')

    def onRudderPortRange( self, event ):
        self.client.set('rudder.calibration_state', 'port range')

    def onRudderRange( self, event ):
        self.client.set('rudder.range', event.GetValue())

def main():
    import gettext
    gettext.install('pypilot', 'locale', unicode=False)
    glutInit(sys.argv)
    app = wx.App()
    
    CalibrationDialog().ShowModal()

if __name__ == "__main__":
    main()
