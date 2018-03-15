#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time, sys
from signalk.client import SignalKClient
import json, math, numpy
from pypilot import quaternion
from pypilot import vector

from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *

point_count=200
recent_point_count=20

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

def GLArray(points):
    vpoints = (GLfloat * (3*len(points)))()
    i = 0
    for point in points:
        for j in range(3):
            vpoints[i+j] = point[j]
        i += 3
    return vpoints

class Shape(object):
    def __init__(self, vertexes):
        self.array = GLArray(vertexes)

    def draw(self):
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, self.array)
        glDrawArrays(GL_LINE_STRIP, 0, len(self.array)/3)
        glDisableClientState(GL_VERTEX_ARRAY);

class Spherical(Shape):
    def __init__(self, beta, f, lons, lats):
        lastPoints = False
        vertexes = []
        for lat in range(lats):
            flat = -math.pi/2+ math.pi/(lats-1)*lat
            points = []

            for lon in range(lons):
                flon = -math.pi + 2*math.pi/(lons-1)*lon
                x = math.cos(flat)*math.cos(flon)
                y = math.cos(flat)*math.sin(flon)
                z = math.sin(flat)
                #v = f(beta, numpy.array([x, y, z]))
                v = beta[3]*numpy.array([x, y, z])
                points.append(v)

            if lastPoints:
                l_lp = lastPoints[0]
                l_p = points[0]
                for i in range(1, len(points)):
                    lp = lastPoints[i]
                    p = points[i]
                    vertexes += [l_lp, l_p, p, lp]

                    l_lp = lp
                    l_p = p
            
            lastPoints = points

        super(Spherical, self).__init__(vertexes)

class Conical(Shape):
    def __init__(self, beta, lons, rs):
        lastPoints = False
        vertexes = []
        dip = math.radians(beta[4])
        for r in range(rs):
            fr = beta[3]*r/rs
            points = []

            for lon in range(lons):
                flon = -math.pi + 2*math.pi/(lons-1)*lon
                x = fr*math.cos(dip)*math.cos(flon)
                y = fr*math.cos(dip)*math.sin(flon)
                z = fr*math.sin(dip)
                #v = beta[:3] + numpy.array([x, y, z])
                v = numpy.array([x, y, z])
                points.append(v)

            if lastPoints:
                l_lp = lastPoints[0]
                l_p = points[0]
                for i in range(1, len(points)):
                    lp = lastPoints[i]
                    p = points[i]
                    vertexes += [l_lp, l_p, p, lp]

                    l_lp = lp
                    l_p = p
            
            lastPoints = points

        super(Conical, self).__init__(vertexes)

        
class Plane(Shape):
    def __init__(self, plane_fit, gridsize):
        plane = numpy.array(plane_fit)

        origin = -plane / numpy.dot(plane, plane)
        n = numpy.array([plane[1], plane[2], plane[0]])

        u = numpy.cross(plane, n)
        v = numpy.cross(plane, u)

        u /= numpy.linalg.norm(u)
        v /= numpy.linalg.norm(v)

        def project_point(point):
            return origin + point[0]*u + point[1]*v

        vertexes = []

        for x in range(-gridsize+1, gridsize):
            for y in range(-gridsize+1, gridsize):
                vertexes += [project_point((x-1, y-1)),
                             project_point((x, y-1)),
                             project_point((x, y)),
                             project_point((x-1, y))]

        super(self, Plane).__init__(vertexes)


class CompassCalibrationPlot():
    default_radius = 30
    def __init__(self):
        self.unit_sphere = Spherical([0, 0, 0, 1], lambda beta, x: x, 32, 16)
        self.mag_fit_new_bias = self.mag_fit_new_sphere = False
        self.mag_fit_sphere = self.mag_fit_cone = False
        self.mag_cal_new_bias = [0, 0, 0, 30, 0]
        self.mag_cal_new_sphere = [0, 0, 0, 30, 0]
        self.mag_cal_sphere = [0, 0, 0, 30]
        
        self.fusionQPose = False
        self.alignmentQ = False

        self.userscale = .005
        self.accel = [0, 0, 0]
        self.heading = 0
        self.points = []
        self.recent_points = []
        
        self.sigmapoints = False
        self.apoints = []

        self.avg = [0, 0, 0]
        self.mode = GL_LINE
        self.uncalibrated_view = True

        '''
    if len(apoints) > 0:
        avg = [0, 0, 0]
        for j in range(3):
            for i in range(len(apoints)):
                avg[j] += apoints[i][j]
        avg = avg/numpy.linalg.norm(avg)
        alignment = quaternion.vec2vec2quat(avg, [0, 0, 1])
        print "avg accel", avg[0], avg[1], avg[2], "alignment", alignment, "angle", math.degrees(2*math.acos(alignment[0]))

    def fellipsoid(beta, x):
        return numpy.array([beta[3]*x[0] + beta[0], \
                            x[1]*beta[3]/beta[4] + beta[1], \
                            x[2]*beta[3]/beta[5] + beta[2]])
                            
    mag_ellipsoid = Shape(mag_cal_ellipsoid, fellipsoid,  64, 32);
    mag_plane = PlaneShape(mag_cal_plane, 2*int(mag_cal_sphere[3]))
    mag_plane_applied = PlaneShape(mag_cal_plane_applied, 2*int(mag_cal_sphere[3]))
    plane_norm = mag_cal_plane/numpy.linalg.norm(mag_cal_plane)

    print "plane norm", plane_norm
    alignment = quaternion.vec2vec2quat(plane_norm, [0, 0, 1])
    print "alignment", alignment, "angle", math.degrees(2*math.acos(alignment[0]))
    '''

    def read_data(self, msg):
        name, data = msg

        if name == 'imu.accel':
            self.accel = data['value']
        elif name == 'imu.heading':
            self.heading = data['value']
        elif name == 'imu.compass':
            value = data['value']
            if value:
                self.recent_points.append(value)
                if len(self.recent_points) > recent_point_count * 2:
                    avg = [0, 0, 0]
                    for point in self.recent_points[:recent_point_count]:
                        for i in range(3):
                            avg[i] += point[i]
                    for i in range(3):
                        avg[i] /= recent_point_count
                    self.points.append(avg)
                    self.recent_points = self.recent_points[recent_point_count:]
                    if len(self.points) > point_count:
                        self.points = self.points[1:]
                    
        elif name == 'imu.compass_calibration_sigmapoints':
            self.sigmapoints = data['value']
        elif name == 'imu.compass_calibration' and data['value']:
            def fsphere(beta, x):
                return beta[3]*x+beta[:3]

            self.mag_cal_sphere = data['value'][0]
            self.mag_fit_sphere = Spherical(self.mag_cal_sphere, fsphere,  32, 16);

            self.mag_fit_cone = Conical(self.mag_cal_sphere, 32, 16);

            #self.mag_cal_new_bias = data['value'][0]
            #self.mag_fit_new_bias = Spherical(self.mag_cal_new_bias, fsphere,  64, 32);

            #self.mag_cal_new_sphere = data['value'][1]
            #self.mag_fit_new_sphere = Spherical(self.mag_cal_new_sphere, fsphere,  64, 32);

            '''
            self.mag_cal_ellipsoid = data['value'][2]
            def fellipsoid(beta, x):
                return numpy.array([beta[3]*x[0] + beta[0], \
                                    x[1]*beta[3]/beta[4] + beta[1], \
                                    x[2]*beta[3]/beta[5] + beta[2]])
            if abs(1-self.mag_cal_ellipsoid[4]) < .25 and \
               abs(1-self.mag_cal_ellipsoid[5]) < .25:
                self.mag_fit_ellipsoid = Spherical(self.mag_cal_ellipsoid, fellipsoid,  64, 32);
            '''

        elif name == 'imu.fusionQPose':
            self.fusionQPose = data['value']

        elif name == 'imu.alignmentQ':
            self.alignmentQ = data['value']
        
    def display(self):
        width, height = self.dim
        ar = float(width) / float(height)
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        fac = .05
        glFrustum( -fac*ar, fac*ar, -fac, fac, .1, 15 )
        glMatrixMode(GL_MODELVIEW)

        cal_new_bias = self.mag_cal_new_bias
        cal_new_sphere = self.mag_cal_new_sphere
        cal_sphere = self.mag_cal_sphere

        glClearColor(0, 0, 0, 0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glPushMatrix()

        s = self.userscale

        glScalef(s, s, s)
        TranslateAfter( 0, 0, -1 )

        glPolygonMode(GL_FRONT_AND_BACK, self.mode)

        if self.uncalibrated_view:
            glPushMatrix()
            glLineWidth(1)

            down = [0, 0, 1]
            if self.fusionQPose and self.alignmentQ:
                q = quaternion.multiply(self.fusionQPose, quaternion.conjugate(self.alignmentQ))
                down = quaternion.rotvecquat(down, quaternion.conjugate(q))
                #down =             self.accel


            glPushMatrix()

            q = [1, 0, 0, 0]
            if self.fusionQPose and self.alignmentQ:
                q = quaternion.multiply(self.fusionQPose, quaternion.conjugate(self.alignmentQ))
                q = quaternion.normalize(q) # correct possible rounding errors
            glRotatef(-math.degrees(quaternion.angle(q)), *q[1:])

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

        
            glPointSize(4)
            glColor3f(1,.3,.3)
            glBegin(GL_POINTS)
            for i in xrange(max(len(self.recent_points) - recent_point_count, 0), \
                            len(self.recent_points)):
                glVertex3fv(self.recent_points[i])
            glEnd()
            
            glPointSize(4)
            glColor3f(0,1,0)
            glBegin(GL_POINTS)
            for i in xrange(len(self.points)):
                glVertex3fv(self.points[i])
            glEnd()

            '''
            glBegin(GL_LINE_STRIP)
            for i in xrange(len(self.points)):
                glVertex3fv(self.points[i])
            glEnd()
            '''

            glColor3f(1, 1, 0)
            glPointSize(6)
            glBegin(GL_POINTS)
            if self.sigmapoints:
                for p in self.sigmapoints:
                    glVertex3fv(p[:3])
            glEnd()

            glColor3f(1,1,1)
            glLineWidth(3.8)
            glBegin(GL_LINES)
#            glVertex3fv(cal[:3])

            try:
                '''
                glColor3f(.8, 0, 0)
                glVertex3fv(map(lambda x,y :-x*cal_new_bias[3]+y, down, cal_new_bias[:3]))
                glVertex3fv(map(lambda x,y : x*cal_new_bias[3]+y, down, cal_new_bias[:3]))

                glColor3f(.8, 0, .8)
                glVertex3fv(map(lambda x,y :-x*cal_new_sphere[3]+y, down, cal_new_sphere[:3]))
                glVertex3fv(map(lambda x,y : x*cal_new_sphere[3]+y, down, cal_new_sphere[:3]))
                '''                
                glColor3f(.8, .8, .8)
                glVertex3fv(map(lambda x,y :-x*cal_sphere[3]+y, down, cal_sphere[:3]))
                glVertex3fv(map(lambda x,y : x*cal_sphere[3]+y, down, cal_sphere[:3]))
            except:
                print 'ERROR!!!!!!!!!!!!!!', self.accel, cal_sphere
            glEnd()

            glPopMatrix()
        else: # calibrated view

            glColor3f(0, 1, 1)
            unit_sphere.draw()
            
            glColor3f(1,0,1)
            mag_plane_applied.draw()

            def f_apply_sphere(beta, x):
                return (x-beta[:3])/beta[3]
            cpoints = map(lambda p : f_apply_sphere(numpy.array(cal), numpy.array(p)), self.points)

            glBegin(GL_LINE_STRIP)
            for i in range(len(cpoints)-10):
                glVertex3fv(cpoints[i])

            glColor3f(0,1,0)
            for i in range(max(len(cpoints)-10, 0), len(cpoints)):
                glVertex3fv(cpoints[i])
            glEnd()

            glBegin(GL_LINE_STRIP)
            glColor3f(1,1,0)
            for i in range(len(self.apoints)/10):
                glVertex3fv(self.apoints[10*i])
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

    def key(self, k, x, y):
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
        

if __name__ == '__main__':
    host = ''
    if len(sys.argv) > 1:
        host = sys.argv[1]

    def on_con(client):
        watchlist = ['imu.accel', 'imu.compass', 'imu.compass_calibration', 'imu.compass_calibration', 'imu.compass_calibration_sigmapoints', 'imu.fusionQPose']
        for name in watchlist:
            client.watch(name)
        
    client = SignalKClient(on_con, host, autoreconnect=True)
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
        while True:
            result = False
            if client:
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
    glutKeyboardFunc( lambda *a: apply(plot.key, a), glutPostRedisplay() )
    glutSpecialFunc( lambda *a : apply(plot.special, a), glutPostRedisplay() )
    glutDisplayFunc( display )

    glutMouseFunc( mouse )
    glutMotionFunc( motion )
    
    glutMainLoop()
