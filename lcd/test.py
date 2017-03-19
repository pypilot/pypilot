#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import os
import ugfx
import glut

white = ugfx.color(255, 255, 255)
black = ugfx.color(0, 0, 0)

use_glut = 'DISPLAY' in os.environ

if use_glut:
    screen = glut.screen()
else:
    screen = ugfx.screen("/dev/fb0")

c, d = black, white
x, w = 0, 400
while w >= 100:
    screen.box(x, x, x+w, x+w, c)
    x += 2
    w -= 4
    c, d = d, c

screen.invert(100, 100, 200, 200)

import font
font.draw(screen, (0, 0), "Hello!", 80, False)

print 'type', str(type(screen))
if use_glut:
    from OpenGL.GLUT import glutMainLoop
    glutMainLoop()
