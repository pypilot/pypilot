#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time, sys
from pypilot.client import pypilotClient
import json, math, numpy
from pypilot import quaternion
from pypilot import vector

from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *

history_point_count=200
recent_point_count=20

from shape import *

def TranslateAfter(x, y, z):
    m = glGetFloatv(GL_MODELVIEW_MATRIX)
    glLoadIdentity()
    glTranslatef(x, y, z)
    glMultMatrixf(m)

def RotateAfter(ang, x, y, z):
    m = glGetFloatv(GL_MODELVIEW_MATRIX)
    glLoadIdentity()
    glRotatef(ang, x, y, z)
    glMultMatrixf(m)

def rotate_mouse(dx, dy):
    RotateAfter((dx**2 + dy**2)**.1, dy, dx, 0)

class CalibrationPlot(object):
    def __init__(self, name):
        self.name = name
        self.mode = GL_LINE
        self.fusionQPose = [1, 0, 0, 0]
        self.alignmentQ = [1, 0, 0, 0]
        self.recentpoints = []
        self.historypoints = []
        self.sigmapoints = []
        self.points = []
        
    def add_point(self, value):
        if not value:
            return
        self.recentpoints.append(value)
        if len(self.recentpoints) > recent_point_count * 2:
            avg = [0, 0, 0]
            for point in self.recentpoints[:recent_point_count]:
                for i in range(3):
                    avg[i] += point[i]
            for i in range(3):
                avg[i] /= recent_point_count
            self.historypoints.append(avg)
            self.recentpoints = self.recentpoints[recent_point_count:]
            if len(self.historypoints) > history_point_count:
                self.historypoints = self.historypoints[1:]

    def read_data_plot(self, msg):
        name, value = msg
        if name == 'imu.fusionQPose':
            self.fusionQPose = value
        elif name == 'imu.alignmentQ':
            self.alignmentQ = value
        elif name == 'imu.'+self.name:
            self.add_point(value)
        elif name == 'imu.'+self.name+'.calibration.sigmapoints':
            self.sigmapoints = value
        elif name == 'imu.'+self.name+'.calibration.points':
            self.points = value
                
    def display_setup(self):
        width, height = self.dim
        ar = float(width) / float(height)
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        fac = .05
        glFrustum( -fac*ar, fac*ar, -fac, fac, .1, 15 )
        glMatrixMode(GL_MODELVIEW)

        glClearColor(0, 0, 0, 0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glPushMatrix()

        s = self.userscale

        glScalef(s, s, s)
        TranslateAfter( 0, 0, -1 )

        glPolygonMode(GL_FRONT_AND_BACK, self.mode)
        glLineWidth(1)
        glPushMatrix()

        if not self.fusionQPose:
            return [0, 0, 1]

        q = quaternion.multiply(self.fusionQPose, self.alignmentQ)
        down = quaternion.rotvecquat([0, 0, 1], quaternion.conjugate(q))
        try:
            glRotatef(-math.degrees(quaternion.angle(q)), *q[1:])
        except:
            print('couldnt do q angle', q)
            
        return down

    def draw_points(self):
        glPointSize(2)
        glColor3f(1,.3,.3)
        glBegin(GL_POINTS)
        for p in self.recentpoints:
            glVertex3fv(p)
        glEnd()
            
        glPointSize(2)
        glColor3f(0,1,0)
        glBegin(GL_POINTS)
        for p in self.historypoints:
            glVertex3fv(p)
        glEnd()

        glColor3f(1, 1, 0)
        glPointSize(4)
        glBegin(GL_POINTS)
        if self.sigmapoints:
            for p in self.sigmapoints:
                glVertex3fv(p[:3])
        glEnd()

        glColor3f(0, 1, 1)
        glPointSize(4)
        glBegin(GL_POINTS)
        if self.points:
            for p in self.points:
                glVertex3fv(p[:3])
        glEnd()
        glPopMatrix()

    def special(self, key, x, y):
        step = 5
        if key == GLUT_KEY_UP:
            RotateAfter(step, 1, 0, 0)
        elif key == GLUT_KEY_DOWN:
            RotateAfter(step, -1, 0, 0)
        elif key == GLUT_KEY_LEFT:
            RotateAfter(step, 0, 1, 0)
        elif key == GLUT_KEY_RIGHT:
            RotateAfter(step, 0, -1, 0)
        elif key == GLUT_KEY_PAGE_UP:
            self.userscale /= .9
        elif key == GLUT_KEY_PAGE_DOWN:
            self.userscale *= .9
        elif key == GLUT_KEY_INSERT:
            RotateAfter(step, 0, 0, 1)
        else:
            return

    def key(self, k, x, y):
        k = k.decode()
        step = 5
        if k == '\b':
            RotateAfter(step, 0, 0, -1)
        elif k == '+' or k == '=':
            self.userscale /= .9
        elif k == '-' or k == '_':
            self.userscale *= .9
        elif k == 'f':
            glutFullScreen()
        elif k == 'm':
            if self.mode == GL_LINE:
                self.mode = GL_FILL
            else:
                self.mode = GL_LINE
        elif k == 'v':
            self.uncalibrated_view = not self.uncalibrated_view
        elif k == 27 or k=='q':
            exit(0)

    def reshape(self, width, height):
        glEnable(GL_DEPTH_TEST)
        self.dim = width, height
        
class AccelCalibrationPlot(CalibrationPlot):
    default_radius = 1
    def __init__(self):
        super(AccelCalibrationPlot, self).__init__('accel')
        self.userscale = .3
        self.cal_sphere = [0, 0, 0, 1]
        self.fit_sphere = False

    def read_data(self, msg):
        self.read_data_plot(msg)
        name, value = msg
        if name == 'imu.accel.calibration' and value:
            def fsphere(beta, x):
                return beta[3]*x+beta[:3]
            self.cal_sphere = value[0]
            self.fit_sphere = Spherical(self.cal_sphere, fsphere,  32, 16);

    def display(self):
        self.display_setup();
        cal_sphere = self.cal_sphere
        
        if self.fit_sphere:
            glColor3f(0, .3, .8)
            self.fit_sphere.draw()

        glPopMatrix()
        glTranslatef(-cal_sphere[0], -cal_sphere[1], -cal_sphere[2])        
        self.draw_points()

class CompassCalibrationPlot(CalibrationPlot):
    default_radius = 30
    def __init__(self):
        super(CompassCalibrationPlot, self).__init__('compass')
        self.userscale = .005
        self.unit_sphere = Spherical([0, 0, 0, 1], lambda beta, x: x, 32, 16)
        self.mag_fit_new_bias = self.mag_fit_new_sphere = False
        self.mag_fit_sphere = self.mag_fit_cone = False
        self.mag_cal_new_bias = [0, 0, 0, 30, 0]
        self.mag_cal_new_sphere = [0, 0, 0, 30, 0]
        self.mag_cal_sphere = [0, 0, 0, 30]
        
        self.accel = [0, 0, 0]
        self.heading = 0

        self.apoints = []
        self.avg = [0, 0, 0]

    def read_data(self, msg):
        self.read_data_plot(msg)
        name, value = msg
        if name == 'imu.accel':
            self.accel = value
        elif name == 'imu.heading':
            self.heading = value
        elif name == 'imu.compass.calibration' and value:
            def fsphere(beta, x):
                return beta[3]*x+beta[:3]
            try:
                self.mag_cal_sphere = value[0]
                self.mag_fit_sphere = Spherical(self.mag_cal_sphere, fsphere,  32, 16);
                self.mag_fit_cone = Conical(self.mag_cal_sphere, 32, 16);
            except Exception as e:
                print("Exception", e)
                
    def display(self):
        down = self.display_setup()
        cal_new_bias = self.mag_cal_new_bias
        cal_new_sphere = self.mag_cal_new_sphere
        cal_sphere = self.mag_cal_sphere
        
        if self.mag_fit_new_bias:
            glColor3f(1, 0, 0)
            self.mag_fit_new_bias.draw()

        if self.mag_fit_new_sphere:
            glColor3f(1, 0, 1)
            self.mag_fit_new_sphere.draw()
                
        if self.mag_fit_sphere:
            glColor3f(0, 0, 1)
            self.mag_fit_sphere.draw()

        if self.mag_fit_cone:
            glColor3f(1, 0, 0)
            self.mag_fit_cone.draw()

        glPopMatrix()
        glTranslatef(-cal_sphere[0], -cal_sphere[1], -cal_sphere[2])

        glColor3f(1,1,1)
        glLineWidth(3.8)
        glBegin(GL_LINES)

        try:
            glColor3f(.8, .8, .8)
            glVertex3fv(list(map(lambda x, y :-x*cal_sphere[3]+y, down, cal_sphere[:3])))
            glVertex3fv(list(map(lambda x, y : x*cal_sphere[3]+y, down, cal_sphere[:3])))
        except Exception as e:
            print('ERROR!', down, cal_sphere, e)
        glEnd()
        self.draw_points()

if __name__ == '__main__':
    host = False
    if len(sys.argv) > 1:
        host = sys.argv[1]

    watchlist = ['imu.accel', 'imu.compass', 'imu.compass.calibration', 'imu.compass.calibration', 'imu.compass.calibration.sigmapoints', 'imu.fusionQPose']
    client = pypilotClient(host)
    for name in watchlist:
        client.watch(name)
        
    plot = CompassCalibrationPlot()

    def display():
        plot.display()
        glutSwapBuffers()

    last = False
    def mouse(button, state, x, y):
        if button == GLUT_LEFT_BUTTON and state == GLUT_DOWN:
            global last
            last = x, y
                
    def motion(x, y):
        global last
        rotate_mouse(x - last[0], y - last[1])
        glutPostRedisplay()
        last = x, y

    n = 0
    def idle():
        client.poll()
        while True:
            result = client.receive_single()
            if not result:
                time.sleep(.01)
                return

            if plot.read_data(result):
                glutPostRedisplay()


    glutInit(sys.argv)
    glutInitWindowPosition(0, 0)
    glutInitWindowSize(600, 500)
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
    glutCreateWindow(sys.argv[0])

    glutIdleFunc(idle)
    glutReshapeFunc( plot.reshape )
    glutKeyboardFunc( lambda *a: plot.key(*a), glutPostRedisplay() )
    glutSpecialFunc( lambda *a : plot.special(*a), glutPostRedisplay() )
    glutDisplayFunc( display )

    glutMouseFunc( mouse )
    glutMotionFunc( motion )
    
    glutMainLoop()
