#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from OpenGL.GLUT import *
from OpenGL.GLU import *
from OpenGL.GL import *
import numpy
import math
import time
import sys

from pypilot.client import pypilotClientFromArgs

class trace(object):
    colors = [[1, 0, 0], [0, 1, 0], [1, 1, 0],
              [1, 0, 1], [0, 1, 1], [0, 0, 1],
              [1, 1, 1], [1, .5, 0], [.5, 1, 0],
              [.5, .5, .5], [0, .5, .5], [.5, 0, 1]]

    def __init__(self, name, group, colorindex, directional):
        self.points = []
        self.offset = 0
        self.visible = True
        self.timeoff = False
        self.name = name
        self.group = group
        self.color = self.colors[colorindex%len(self.colors)]
        self.directional = directional

    def add(self, t, data, mindt):
        # update previous timestamps based on downtime
        if self.points and math.isnan(self.points[0][1]):
            dt = time.monotonic() - t - self.timeoff
            self.timeoff = False
            for i in range(len(self.points)):
                point = self.points[i]
                self.points[i] = point[0]-dt, point[1]

                
        if not self.timeoff or self.timeoff < time.monotonic() - t or self.timeoff > time.monotonic() - t + 1:
            self.timeoff = time.monotonic() - t
            
        elif self.points and t-self.points[0][0]<mindt:
            return False

        self.points.insert(0, (t, data))
        return True
        
    def add_blank(self):
        if self.points:
            self.points.insert(0, (self.points[0][0], float('nan')))

    def center(self):
        if len(self.points) > 0:
            self.offset = self.points[0][1]

    def noise(self):
        try:
            avg = sum(map(lambda x : x[1], self.points)) / len(self.points)
            return math.sqrt(sum(map(lambda x : (avg-x[1])**2, self.points))) / len(self.points)
        except:
            return 0

    def tracevertexes(self, time, plot, gldrawtype):
        # remove datapoints after the first one that is off the screen
        for i in range(len(self.points)):
            if self.points[i][0] < time - plot.disptime:
                self.points = self.points[:i+1]
                break

        glBegin(gldrawtype)
        for point in self.points:
            if math.isnan(point[1]):
                glEnd()
                glBegin(gldrawtype)
            else:
                y = point[1] - self.offset
                if self.directional:
                    if y >= 180:
                        y -= 360
                    elif y < - 180:
                        y += 360
                glVertex2d(point[0]-time, y)
        glEnd()

    def draw(self, plot):
        if not self.visible or not self.timeoff:
            return

        t = time.monotonic() - self.timeoff
        
        glPushMatrix()

        glColor3dv(self.color)
        self.tracevertexes(t, plot, GL_LINE_STRIP)

        if plot.drawpoints:
            glPointSize(8)
            self.tracevertexes(t, plot, GL_POINTS)

        glPopMatrix()

    def draw_fft(self):
        if len(self.points) < 1:
            return
        pts = map(lambda p: p[1] - self.offset, self.points)

        out = numpy.fft.rfft(pts)
        c = len(out)

        norm = 0
        for i in range(c/2):
            norm += numpy.real(out[i])**2 + numpy.imag(out[i])**2

        norm = math.sqrt(norm)
        if norm <= 0:
            return

        for i in range(1, pypilotPlot.NUM_X_DIV):
            x = float(i) / pypilotPlot.NUM_X_DIV
            self.rasterpos([x, .95])
            period = 3/math.exp(x) # incorrect!!
            pypilotPlot.drawputs(str(period))

        glPushMatrix()
        glBegin(GL_LINE_STRIP)
        for i in range(c/2):
            glVertex2d(float(i) * 2 / (c-2), abs(out[i]) / norm)
        glEnd()
        glPopMatrix()


class pypilotPlot():
    NUM_X_DIV = 5
    NUM_Y_DIV = 6

    #FONT = GLUT_BITMAP_9_BY_15
    FONT = GLUT_BITMAP_TIMES_ROMAN_24

    def __init__(self):
        self.value_list = False
        self.freeze = False
        self.drawpoints = False

        self.scale = 1.0
        self.scalestate = 0

        self.delay = 0
        self.disptime = 30
        self.curtrace = False
        self.fft_on = False
        self.starttime = 0

        self.reset()

    def reset(self):
        self.traces = []
        self.timestamp = False

    def add_data(self, name, group, timestamp, value):
        t = False
        for tn in self.traces:
            if tn.name == name:
                t = tn
                break

        if not t:
            for tn in self.traces:
                if name == group and tn.group == group:
                    return

            directional = name in self.value_list and \
                          'directional' in self.value_list[name] and \
                          self.value_list[name]['directional']
            t = trace(name, group, len(self.traces), directional)
            self.traces.append(t)
#            if not self.curtrace:
            self.curtrace = t

        # time must change by 1 pixel to bother to log and display
        mindt = self.disptime / float(self.width)
        return t.add(timestamp, value, mindt) and t.visible
        
    def add_blank(self, group=False):
        for t in self.traces:
            if not group or group == t.group:
                t.add_blank()

    def read_data(self, msg):
        name, value = msg
        if name == 'timestamp':
            self.timestamp = value
            return
        #timestamp = time.monotonic()
        if not self.timestamp:
            return
        timestamp = self.timestamp

        if type(value) == type([]):
            ret = False
            for i in range(len(value)):
                namei = name+str(i)
                ret = self.add_data(namei, name, timestamp, float(value[i])) or ret
            return ret
        else:
            if type(value) == type(True):
                if value:
                    value = 1
                else:
                    value = 0
            return self.add_data(name, name, timestamp, float(value))

    @staticmethod
    def drawputs(str):
        for c in str:
            glutBitmapCharacter(pypilotPlot.FONT, ctypes.c_int(ord(c)))

    # because glutbitmapcharacter doesn't get the glcolor unless the raster position is set
    def synccolor(self):
        vp = glGetDoublev(GL_VIEWPORT)
        glRasterPos2d(*self.lastrasterpos)

    def rasterpos(self, pos):
        #pos = glGetDoublev(GL_CURRENT_RASTER_POSITION)
        glRasterPos2d(*pos)
        self.lastrasterpos = pos

    def drawticks(self):
        glLineWidth(1)
        glEnable(GL_LINE_STIPPLE)
        glColor3d(.6, .6, .6)
        glLineStipple(1, 0x0011)
        glBegin(GL_LINES)

        for i in range(1, pypilotPlot.NUM_X_DIV):
            x = float(i) / pypilotPlot.NUM_X_DIV
            glVertex2d(x, 0)
            glVertex2d(x, 1)
            
        for i in range(1, pypilotPlot.NUM_Y_DIV):
            y = float(i) / pypilotPlot.NUM_Y_DIV
            glVertex2d(0, y)
            glVertex2d(1, y)
       
        glEnd()
        glDisable(GL_LINE_STIPPLE)

    def drawtext(self):
        if not self.curtrace:
            return

        # For each datapoint display its scale
        glColor3d(1, 1, 1)
        self.rasterpos([0, .01])
        i=1
        for t in self.traces:
            glColor3dv(t.color)
            self.synccolor()
            pypilotPlot.drawputs("%d " % i)
            i+=1

        glColor3dv(self.curtrace.color)
        self.synccolor()

        val = float('nan')
        if len(self.curtrace.points):
            val = self.curtrace.points[0][1]

        pypilotPlot.drawputs("name: %s offset: %g  value: %g  visible: %s  " % \
                 (self.curtrace.name, self.curtrace.offset, val, 'T' if self.curtrace.visible else 'F'))
        glColor3d(1, 1, 1)
        #self.synccolor()
        pypilotPlot.drawputs("scale: %g  time: %g  " % (self.scale, self.disptime))
        
        glColor3dv(self.curtrace.color)
        #self.synccolor()
        
        pypilotPlot.drawputs("noise: %g" % self.curtrace.noise())

    def init(self, value_list):
        glClearColor (0.0, 0.0, 0.0, 0.0)
    
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0)

        #glEnable(GL_LINE_SMOOTH)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        self.value_list = value_list

    def display(self):
        if self.freeze:
            return

        glClear (GL_COLOR_BUFFER_BIT)

        self.drawticks()
        if self.fft_on:
            self.curtrace.draw_fft()

        glPushMatrix()

        glScaled(1.0/self.disptime, .5, 1)
        glTranslated(self.disptime, .5, 0)

        glTranslated(0, .5, 0) # center on 0 ??
        glScaled(1, 2/(self.scale * pypilotPlot.NUM_Y_DIV), 1)

        glLineWidth(1)

        for t in self.traces:
            t.draw(self)

        glPopMatrix()
        self.drawtext()

    def reshape (self, w, h):
        glViewport (0, 0, w, h)
        glMatrixMode (GL_PROJECTION)
        glLoadIdentity ()
        glMatrixMode (GL_MODELVIEW)
        glLoadIdentity()
        gluOrtho2D(0, 1, 0, 1)
        self.width = w

    def increasescale(self):
        if self.scalestate%3 == 1:
            self.scale *= 2.5
        else:
            self.scale *= 2
        self.scalestate+=1

    def decreasescale(self):
        if self.scalestate%3 == 2:
            self.scale /= 2.5
        else:
            self.scale /= 2
        self.scalestate-=1

    def adjustoffset(self, offset, y):
        self.curtrace.offset += offset*self.scale * self.NUM_Y_DIV / y

    def key(self, k, x, y):
        if k == 'q' or k == 27:
            exit(0)
        
        if not self.curtrace:
            return

        if k >= '0' and k <= '9':
            ind = int(k) - 1

            if ind < 0:
                ind += 10
            if len(self.traces) <= ind:
                return
            self.curtrace = self.traces[ind]
        elif k == '+' or k == '=':
            self.increasescale()
        elif k == '-' or k == '_':
            self.decreasescale()
        elif k == 'f':
            self.freeze = not self.freeze
        elif k == 'p':
            self.drawpoints = not self.drawpoints
        elif k == 'c':
            self.curtrace.center()
        elif k == 'C':
            for trace in self.traces:
                trace.center()
        elif k == 'v':
            self.curtrace.visible = not self.curtrace.visible
        elif k == 'V':
            v = not self.curtrace.visible
            for trace in self.traces:
                trace.visible = v
        elif k == 'z':
            self.curtrace.offset = 0
        elif k == 'Z':
            for trace in self.traces:
                trace.offset = 0
        elif k == 'w':
            self.fft_on = not self.fft_on
        
    def special(self, key, x, y):
        if not self.curtrace:
            return

        dist = self.scale / 10.0
        if key == GLUT_KEY_DOWN:
            self.curtrace.offset += dist
        elif key == GLUT_KEY_UP:
            self.curtrace.offset -= dist
        elif key == GLUT_KEY_F11:
            glutFullScreen()

    def select(self, name):
        for t in self.traces:
            if name in t.name:
                self.curtrace = t
                break

def main():
    plot = pypilotPlot()
    client = pypilotClientFromArgs(sys.argv)
    
    print('connected')
    def idle():
        while True:
            try:
                result = client.receive_single()
                if result:
                    plot.read_data(result)
                else:
                    time.sleep(.01)
                    break
            except:
                pass

    glutInit(sys.argv)
    glutInitWindowPosition(250, 0)
    glutInitWindowSize(1000, 500)
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB)
    glutCreateWindow ('glplot')

    def display():
        plot.display()
        glutSwapBuffers()

    glutDisplayFunc(display)
    glutReshapeFunc(plot.reshape)
    glutKeyboardFunc(plot.key)
    glutSpecialFunc(plot.special)
    glutIdleFunc(idle)

    plot.init(client.list_values(10))

    fps = 30
    def timeout(arg):
        glutPostRedisplay()
        glutTimerFunc(int(1000/fps), timeout, arg)

    glutTimerFunc(0, timeout, None)
    glutMainLoop()

if __name__ == '__main__':
    main()
