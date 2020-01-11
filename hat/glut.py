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

from ugfx import ugfx

class screen(ugfx.surface):
    def __init__(self, size):
        super(screen, self).__init__(size[0], size[1], 4, None)
        self.fill(0)

        glutInit(sys.argv)
        glutInitWindowPosition(250, 0)
        glutInitWindowSize(int(640*size[0]/size[1]), 640)
        glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB)
        glutCreateWindow ("lcd screen as window")

        def display():
            glEnable(GL_TEXTURE_2D)
            glBindTexture(GL_TEXTURE_2D, 0)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
#            p=range(640*480*4)
#            data = numpy.array(list(p), numpy.int8)

            p = []
            for y in range(size[1]):
                for x in range(size[0]):
                    v = int(self.getpixel(x, y))
                    p.append((v>>16)&0xff)
                    p.append((v>>8)&0xff)
                    p.append(v&0xff)
                    p.append(0)

            data = numpy.array(list(p), numpy.int8)

#            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, size[0], size[1], 0, GL_RGBA, GL_UNSIGNED_BYTE, data)


            OpenGL.raw.GL.VERSION.GL_1_1.glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, size[0], size[1], 0, GL_RGBA, GL_UNSIGNED_BYTE, data)

            glBegin(GL_QUADS)
            glTexCoord2f(0, 1), glVertex2f(0, 0)
            glTexCoord2f(1, 1), glVertex2f(1, 0)
            glTexCoord2f(1, 0), glVertex2f(1, 1)
            glTexCoord2f(0, 0), glVertex2f(0, 1)
            glEnd()
            glDisable(GL_TEXTURE_2D)
            glutSwapBuffers()

        def reshape (w, h):
            glViewport (0, 0, w, h)
            glMatrixMode (GL_PROJECTION)
            glLoadIdentity ()
            glMatrixMode (GL_MODELVIEW)
            glLoadIdentity()
            gluOrtho2D(0, 1, 0, 1)

        def key(k, x, y):
            if k == 'q' or k == 27:
                exit(0)
        
        glutDisplayFunc(display)
        glutReshapeFunc(reshape)
        glutKeyboardFunc(key)
        #        glutSpecialFunc(plot.special)

    def refresh(self):
        OpenGL.GLUT.glutPostRedisplay()

if __name__ == '__main__':
   s = screen((128, 160))
   s.invert(10, 10, 20, 20)
   glutMainLoop()
