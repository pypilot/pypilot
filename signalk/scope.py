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
import json

from client import SignalKClient

class trace():
    colors = [[1, 0, 0], [0, 1, 0], [1, 1, 0],
              [1, 0, 1], [0, 1, 1], [0, 0, 1],
              [1, 1, 1], [1, .5, 0], [.5, 1, 0],
              [.5, .5, .5], [0, .5, .5], [.5, 0, 1]]

    def __init__(self, name, colorindex):
        self.points = []
        self.offset = 0
        self.visible = True
        self.timeoff = False
        self.name = name
        self.color = self.colors[colorindex%len(self.colors)]

    def add(self, t, data):
        if not self.timeoff:
            self.timeoff = time.time() - t
        self.points.insert(0, (t, data))

    def center(self):
        if len(self.points) > 0:
            self.offset = self.points[0][1]

    def noise(self):
        return 0

    def tracevertexes(self, time, plot):
        # remove datapoints after the first one that is off the screen
        i = 0
        for point in self.points:
            if point[0] < time - plot.disptime:
                self.points = self.points[:i+1]
                break
            i+=1

        for point in self.points:
            glVertex2d(point[0]-time, point[1])

    def draw(self, plot):
        if not self.visible or not self.timeoff:
            return

        t = time.time() - self.timeoff
        
        glPushMatrix()

        glTranslated(0, -self.offset, 0)

        glColor3dv(self.color)
        glBegin(GL_LINE_STRIP)
        self.tracevertexes(t, plot)
        glEnd()
        if plot.drawpoints:
            glPointSize(8)
            glBegin(GL_POINTS)
            self.tracevertexes(t, plot)
            glEnd()

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

        for i in range(1, SignalKPlot.NUM_X_DIV):
            x = float(i) / SignalKPlot.NUM_X_DIV
            glRasterPos2d(x, .95)
            period = 3/math.exp(x) # incorrect!!
            SignalKPlot.drawputs(str(period))

        glPushMatrix()
        glBegin(GL_LINE_STRIP)
        for i in range(c/2):
            glVertex2d(float(i) * 2 / (c-2), abs(out[i]) / norm)
        glEnd()
        glPopMatrix()


class SignalKPlot():
    NUM_X_DIV = 5
    NUM_Y_DIV = 6

    FONT = GLUT_BITMAP_9_BY_15

    def __init__(self):
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

    def add_data(self, name, timestamp, value):
        t = False
        for tn in self.traces:
            if tn.name == name:
                t = tn
                break

        if not t:
            t = trace(name, len(self.traces))
            self.traces.append(t)
#            if not self.curtrace:
            self.curtrace = t
        
        t.add(timestamp, value)


    def read_data(self, msg):
        name, data = msg
        if 'timestamp' in data:
            timestamp = data['timestamp']
        else:
            timestamp = time.time()

        value = data['value']
        if type(value) == type([]):
            for i in range(len(value)):
                namei = name+str(i)
                self.add_data(namei, timestamp, float(value[i]))
        else:
            if type(value) == type(True):
                if value:
                    value = 1
                else:
                    value = 0
            self.add_data(name, timestamp, float(value))

    @staticmethod
    def drawputs(str):
        for c in str:
            glutBitmapCharacter(SignalKPlot.FONT, ctypes.c_int(ord(c)))

    # because glutbitmapcharacter doesn't get the glcolor unless the raster position is set
    @staticmethod
    def synccolor():
        pos = (GLdouble * 4)()
        vp = (GLdouble * 4)()
        glGetDoublev(GL_VIEWPORT, vp)
        glGetDoublev(GL_CURRENT_RASTER_POSITION, pos)
        glRasterPos2d(pos[0]/vp[2], pos[1]/vp[3])


    def drawticks(self):
        glLineWidth(1)
        glEnable(GL_LINE_STIPPLE)
        glColor3d(.6, .6, .6)
        glLineStipple(1, 0x0011)
        glBegin(GL_LINES)

        for i in range(1, SignalKPlot.NUM_X_DIV):
            x = float(i) / SignalKPlot.NUM_X_DIV
            glVertex2d(x, 0)
            glVertex2d(x, 1)
            
        for i in range(1, SignalKPlot.NUM_Y_DIV):
            y = float(i) / SignalKPlot.NUM_Y_DIV
            glVertex2d(0, y)
            glVertex2d(1, y)
       
        glEnd()
        glDisable(GL_LINE_STIPPLE)

    def drawtext(self):
        if not self.curtrace:
            return

        # For each datapoint display its scale
        glColor3d(1, 1, 1)
        glRasterPos2d(0, 0)
        i=1
        for t in self.traces:
            glColor3dv(t.color)
            SignalKPlot.synccolor()
            SignalKPlot.drawputs("%d " % i)
            i+=1

        glColor3dv(self.curtrace.color)
        SignalKPlot.synccolor()

        val = float('nan')
        if len(self.curtrace.points):
            val = self.curtrace.points[0][1]

        SignalKPlot.drawputs("name: %s offset: %g  value: %g  visible: %s  noise: %g  " % \
                 (self.curtrace.name, self.curtrace.offset, val, str(self.curtrace.visible), self.curtrace.noise()))
   
        glColor3d(1, 1, 1)
        SignalKPlot.synccolor()
        SignalKPlot.drawputs("scale: %g  time: %g" % (self.scale, self.disptime))

    def init(self):
        glClearColor (0.0, 0.0, 0.0, 0.0)
    
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0.0, 1.0, 0.0, 1.0, -1.0, 1.0)

        glEnable(GL_LINE_SMOOTH)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    def display(self):
        if self.freeze:
            return

        glClear (GL_COLOR_BUFFER_BIT)

        self.drawticks()
        self.drawtext()

        if self.fft_on:
            self.curtrace.draw_fft()

        glPushMatrix()

        glScaled(1.0/self.disptime, .5, 1)
        glTranslated(self.disptime, .5, 0)

        glTranslated(0, .5, 0) # center on 0 ??
        glScaled(1, 2/(self.scale * SignalKPlot.NUM_Y_DIV), 1)


        glLineWidth(1)

        for t in self.traces:
            t.draw(self)

        glPopMatrix()


    def reshape (self, w, h):
        glViewport (0, 0, w, h)
        glMatrixMode (GL_PROJECTION)
        glLoadIdentity ()
        glMatrixMode (GL_MODELVIEW)
        glLoadIdentity()
        gluOrtho2D(0, 1, 0, 1)

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
    plot = SignalKPlot()

    host = ""
    if len(sys.argv) > 1:
        host = sys.argv[1]

    def on_con(client):
        time.sleep(1)
        plot.reset()
        for arg in sys.argv[2:]:
            client.watch(arg)

    try:
        client = SignalKClient(on_con, host, autoreconnect=True)
        print "connected to", host
    except:
        print "Failed to connect:", host
        client = False


    if False:
        points = []
        for line in sys.stdin.readlines():
            try:
                data = json.loads(line.rstrip())
                for msg in client.flatten_line(line):
                    plot.read_data(msg)
            except:
                print "invalid input line", line

    def idle():
        while True:
            result = False
            if client:
                try:
                    result = client.receive_single()
                except:
                    pass

            if not result:
                time.sleep(.01)
                return

            plot.read_data(result)

    glutInit(sys.argv)
    glutInitWindowPosition(250, 0)
    glutInitWindowSize(1000, 500)
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB)
    glutCreateWindow ("glplot")

    def display():
        plot.display()
        glutSwapBuffers()

    glutDisplayFunc(display)
    glutReshapeFunc(plot.reshape)
    glutKeyboardFunc(plot.key)
    glutSpecialFunc(plot.special)
    glutIdleFunc(idle)

    plot.init()

    def timeout(arg):
        glutPostRedisplay()
        glutTimerFunc(arg, timeout, arg)

    fps = 30
    glutTimerFunc(0, timeout, 1000/fps)
    glutMainLoop()

if __name__ == '__main__':
    main()
