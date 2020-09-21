s#!/usr/bin/env python
#
#   Copyright (C) 2018 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import math, numpy
from OpenGL.GL import *

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
        glDrawArrays(GL_LINE_STRIP, 0, int(len(self.array)/3))
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
