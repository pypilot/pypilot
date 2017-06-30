#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import json
import sys
import math
import time
import vector
from quaternion import *
import multiprocessing
from signalk.pipeserver import NonBlockingPipe

import numpy

def FitLeastSq(beta0, f, zpoints):
    try:
        import scipy.optimize
    except:
        print "failed to load scientific library, cannot perform calibration update!"
        return False

    leastsq = scipy.optimize.leastsq(f, beta0, zpoints)
    return list(leastsq[0])

def FitPoints(points, sphere_fit):
    if len(points) < 4:
        return False

    zpoints = [[], [], [], [], [], []]
    for i in range(6):
        zpoints[i] = map(lambda x : x[i], points)

    # with few sigma points, adjust only bias
#    def f_sphere_bias3(beta, x, r):
#        return ((x[0]-beta[0])**2 + (x[1]-beta[1])**2 + (x[2]-beta[2])**2)/r**2 - 1
#    sphere_bias_fit = FitLeastSq(sphere_fit[:3], f_sphere_bias3, (zpoints, sphere_fit[3]))
#    if not sphere_bias_fit:
#        print 'sphere bias failed!!! ', len(points), points
#        return False
#    print 'sphere bias fit', sphere_bias_fit
#    sphere_fit = sphere_bias_fit + [sphere_fit[3]]

    if len(points) < 9:
        return False

    def f_sphere3(beta, x):
        return ((x[0]-beta[0])**2 + (x[1]-beta[1])**2 + (x[2]-beta[2])**2) - beta[3]
    sphere_fit = FitLeastSq([0, 0, 0, 30], f_sphere3, zpoints)
    if not sphere_fit:
        print 'FitLeastSq failed!!!! ', len(points), points
        return False
        #sphere_fit[3] = abs(sphere_fit[3])
    sphere_fit[3] = math.sqrt(sphere_fit[3])
    print 'sphere fit', sphere_fit

    def f_ellipsoid3(beta, x):
        return (x[0]-beta[0])**2 + (beta[4]*(x[1]-beta[1]))**2 + (beta[5]*(x[2]-beta[2]))**2 - beta[3]**2
    #ellipsoid_fit = FitLeastSq(sphere_fit + [1, 1], f_ellipsoid3, zpoints)
    #print 'ellipsoid_fit', ellipsoid_fit
    ellipsoid_fit = False

    def f_new_bias3(beta, x):
        #print 'beta', beta
        b = numpy.matrix(map(lambda a, b : a - b, x[:3], beta[:3]))
        m = list(numpy.array(b.transpose()))
        r0 = map(lambda y : beta[3]**2 - vector.dot(y, y), m)
        g = list(numpy.array(numpy.matrix(x[3:]).transpose()))
        r1 = map(lambda y, z : 600*(beta[4] - vector.dot(y, z)/vector.norm(y)), m, g)
        return r0+r1
        
    new_bias_fit = FitLeastSq(sphere_fit[:4] + [0], f_new_bias3, zpoints)
    print 'new bias fit', new_bias_fit, math.degrees(math.asin(new_bias_fit[4]))

    if not ellipsoid_fit:
        ellipsoid_fit = sphere_fit + [1, 1]
    return [new_bias_fit, sphere_fit, ellipsoid_fit]

def avg(fac, v0, v1):
    return map(lambda a, b : (1-fac)*a + fac*b, v0, v1)

class SigmaPoint(object):
    def __init__(self, compass, down):
        self.compass = compass
        self.down = down
        self.count = 1
        self.time = time.time()

    def add_measurement(self, compass, down):
        self.count += 1
        fac = max(1/self.count, .01)
        self.compass = avg(fac, self.compass, compass)
        self.down = avg(fac, self.down, down)
        self.time = time.time()

class SigmaPoints(object):
    sigma = 1.6**2 # distance between sigma points
    down_sigma = .05 # distance between down vectors
    max_sigma_points = 64

    def __init__(self):
        self.sigma_points = []
        self.lastpoint = False

    def AddPoint(self, compass, down):
        if not self.lastpoint:
            self.lastpoint = SigmaPoint(compass, down)
            return

        lastcompass = self.lastpoint.compass
        dist = (lastcompass[0] - compass[0])**2 + \
               (lastcompass[1] - compass[1])**2 + \
               (lastcompass[2] - compass[2])**2

        if dist < SigmaPoints.sigma:
            self.lastpoint.add_measurement(compass, down)
            return
            #self.lastpoint = compass, down

        compass = self.lastpoint.compass
        down = self.lastpoint.down
        self.lastpoint = False
        
        ind = 0
        for point in self.sigma_points:
            #dist = vector.dist(point.compass, compass)
            dist = (point.compass[0] - compass[0])**2 + \
                   (point.compass[1] - compass[1])**2 + \
                   (point.compass[2] - compass[2])**2
            #down_dist = vector.dist(point.down, down)
            #print 'dist', dist, 'down_dist', down_dist
            if dist < SigmaPoints.sigma:
            #and down_dist < SigmaPoints.down_sigma:
                point.add_measurement(compass, down)
                if ind > 0:
#                    self.sigma_points = [point] + self.sigma_points[:ind] + self.sigma_points[ind+1:]
                    self.sigma_points = self.sigma_points[:ind-1] + [point] + [self.sigma_points[ind-1]] + self.sigma_points[ind+1:]
                return
            ind += 1

        index = len(self.sigma_points)
        p = SigmaPoint(compass, down)
        if index == SigmaPoints.max_sigma_points:
            # replace point that is closest to other points
            mindisti = 0
            mindist = 1e20
            for i in range(len(self.sigma_points)):
                for j in range(i):
                    dist = vector.dist(self.sigma_points[i].compass, self.sigma_points[j].compass)
                    if dist < mindist:
                        # replace older point
                        if self.sigma_points[i].time < self.sigma_points[j].time:
                            mindisti = i
                        else:
                            mindisti = j
                        mindist = dist

            self.sigma_points[mindisti] = p
        else:
            self.sigma_points.append(p)

def ComputeCoverage(sigma_points, bias):
    def ang(p):
        v = rotvecquat(vector.sub(p.compass, bias), vec2vec2quat(p.down, [0, 0, 1]))
        return math.atan2(v[1], v[0])

    angles = sorted(map(ang, sigma_points))
    #print 'angles', angles
                    
    max_diff = 0
    for i in range(len(angles)):
        diff = -angles[i]
        j = i+1
        if j == len(angles):
            diff += 2*math.pi
            j = 0
        diff += angles[j]
        max_diff = max(max_diff, diff)
    return max_diff    

def CalibrationProcess(points, fit_output, initial):
    import os
    if os.system('sudo chrt -pi 0 %d 2> /dev/null > /dev/null' % os.getpid()):
      print 'warning, failed to make calibration process idle, trying renice'
      if os.system("renice 20 %d" % os.getpid()):
          print 'warning, failed to renice calibration process'

    cal = SigmaPoints()

    while True:
        t = time.time()
        addedpoint = False
        while time.time() - t < 60:
            p = points.recv(1)
            if p:
                cal.AddPoint(p[:3], p[3:6])
                addedpoint = True

        if not addedpoint: # don't bother to run fit if no new data
            continue
        # remove points with less than 5 measurements, or older than 1 hour
        p = []
        for sigma in cal.sigma_points:
            # require at least 2 measurements, and only use measurements in last hour
            if sigma.count >= 2 and time.time() - sigma.time < 3600:
                p.append(sigma)
        cal.sigma_points = p

        # attempt to perform least squares fit
        p = []
        for sigma in cal.sigma_points:
            p.append(sigma.compass + sigma.down)

        fit = FitPoints(p, initial)
        if not fit:
            continue

        print 'fit', fit
        mag = fit[0][3]
        if mag < 9 or mag > 70:
            print 'fit found field outside of normal earth field strength', fit
            continue

        bias = fit[0][:3]
        for sigma in cal.sigma_points:
            dev = map(lambda a, b: (a-b)/mag, sigma.compass, bias)
            dev = abs(1-vector.norm(dev))
            if dev > .05 and time.time()-sigma.time>300 and False:
                print 'remove bad sigma', sigma.compass, dev
                cal.sigma_points.remove(sigma)

        coverage = 360 - math.degrees(ComputeCoverage(cal.sigma_points, bias))
        if coverage < 60: # require 60 degrees
            print 'calibration: not enough coverage', coverage, 'degrees'
            continue

        # sphere fit should basically agree with new bias
        spherebias = fit[1][:3]
        sbd = vector.norm(vector.sub(bias, spherebias))
        if sbd > 5:
            print 'sphere and newbias disagree', sbd
            continue

        # if the bias has sufficiently changed
        n = map(lambda a, b: (a-b)**2, bias, initial[:3])
        d = n[0]+n[1]+n[2]
        initial = fit[0]
        if d < .1:
            print 'insufficient change in bias'
            continue

        print 'coverage', coverage
        print 'got new fit:', fit
        print 'calibration: sphere bias difference', sbd
        fit_output.send((fit, map(lambda p : p.compass + p.down, cal.sigma_points)), False)
                                 
class MagnetometerAutomaticCalibration(object):
    def __init__(self, cal_pipe, initial):
        self.cal_pipe = cal_pipe
        self.sphere_fit = initial
        points, self.points = NonBlockingPipe('points pipe', True)
        self.fit_output, fit_output = NonBlockingPipe('fit output', True)

        self.process = multiprocessing.Process(target=CalibrationProcess, args=(points, fit_output, self.sphere_fit))
        print 'start cal process'
        self.process.start()

    def __del__(self):
        print 'terminate calibration process'
        self.process.terminate()

    def AddPoint(self, point):
        self.points.send(point, False)
    
    def UpdatedCalibration(self):
        result = self.fit_output.recv()
        if not result:
            return

        # use new bias fit
        self.cal_pipe.send(tuple(result[0][0][:3]))
        return result

def ExtraFit():
#        return [sphere_fit, 1, ellipsoid_fit]
    def f_rotellipsoid3(beta, x):
        return x[0]**2 + (beta[1]*x[1] + beta[3]*x[0])**2 + (beta[2]*x[2] + beta[4]*x[0] + beta[5]*x[1])**2 - beta[0]**2
        a = x[0]-beta[0]
        b = x[1]-beta[1]
        c = x[2]-beta[2]
        return (a)**2 + (beta[4]*b + beta[6]*a)**2 + (beta[5]*c + beta[7]*a + beta[8]*b)**2 - beta[3]**2
    def f_ellipsoid3_cr(beta, x, cr):
        a = x[0]-beta[0]
        b = x[1]-beta[1]
        c = x[2]-beta[2]
        return (a)**2 + (beta[4]*b + cr[0]*a)**2 + (beta[5]*c + cr[1]*a + cr[2]*b)**2 - beta[3]**2

        # if the ellipsoid fit is sane
    if abs(ellipsoid_fit[4]-1) < .2 and abs(ellipsoid_fit[5]-1) < .2:
        cpoints = map(lambda a, b : a - b, zpoints[:3], ellipsoid_fit[:3])
        rotellipsoid_fit = FitLeastSq(ellipsoid_fit[3:] + [0, 0, 0], f_rotellipsoid3, cpoints)
        #print 'rotellipsoid_fit', rotellipsoid_fit
        ellipsoid_fit2 = FitLeastSq(ellipsoid_fit[:3] + rotellipsoid_fit[:3], f_ellipsoid3_cr, (zpoints, rotellipsoid_fit[3:]))
        #print 'ellipsoid_fit2', ellipsoid_fit2

        cpoints = map(lambda a, b : a - b, zpoints[:3], ellipsoid_fit2[:3])
        rotellipsoid_fit2 = FitLeastSq(ellipsoid_fit[3:] + [0, 0, 0], f_rotellipsoid3, cpoints)
        print 'rotellipsoid_fit2', rotellipsoid_fit2
    else:
        ellipsoid_fit = False

    def f_uppermatrixfit(beta, x):
            b = numpy.matrix(map(lambda a, b : a - b, x[:3], beta[:3]))
            r = numpy.matrix([beta[3:6], [0]+list(beta[6:8]), [0, 0]+[beta[8]]])
            print 'b', beta

            m = r * b
            m = list(numpy.array(m.transpose()))
            r0 = map(lambda y : 1 - vector.dot(y, y), m)

            return r0

    def f_matrixfit(beta, x, efit):
            b = numpy.matrix(map(lambda a, b : a - b, x[:3], efit[:3]))
            r = numpy.matrix([[1,       beta[0], beta[1]],
                              [beta[2], efit[4], beta[3]],
                              [beta[4], beta[5], efit[5]]])

            m = r * b
            m = list(numpy.array(m.transpose()))
            r0 = map(lambda y : efit[3]**2 - vector.dot(y, y), m)
            #return r0

            g = list(numpy.array(numpy.matrix(x[3:]).transpose()))
            r1 = map(lambda y, z : beta[6] - vector.dot(y, z), m, g)

            return r0+r1

    def f_matrix2fit(beta, x, efit):
            b = numpy.matrix(map(lambda a, b : a - b, x[:3], beta[:3]))
            r = numpy.matrix([[1,       efit[0], efit[1]],
                              [efit[2], beta[4], efit[3]],
                              [efit[4], efit[5], beta[5]]])

            m = r * b
            m = list(numpy.array(m.transpose()))
            r0 = map(lambda y : beta[3]**2 - vector.dot(y, y), m)
            #return r0

            g = list(numpy.array(numpy.matrix(x[3:]).transpose()))
            r1 = map(lambda y, z : beta[6] - vector.dot(y, z), m, g)

            return r0+r1

    if False:
         matrix_fit = FitLeastSq([0, 0, 0, 0, 0, 0, 0], f_matrixfit, (zpoints, ellipsoid_fit))
         #print 'matrix_fit', matrix_fit

         matrix2_fit = FitLeastSq(ellipsoid_fit + [matrix_fit[6]], f_matrix2fit, (zpoints, matrix_fit))
         #print 'matrix2_fit', matrix2_fit

         matrix_fit2 = FitLeastSq(matrix_fit, f_matrixfit, (zpoints, matrix2_fit))
         print 'matrix_fit2', matrix_fit2

         matrix2_fit2 = FitLeastSq(matrix2_fit[:6] + [matrix_fit2[6]], f_matrix2fit, (zpoints, matrix_fit2))
         print 'matrix2_fit2', matrix2_fit2

    def rot(v, beta):
        sin, cos = math.sin, math.cos
        v = vector.normalize(v)
        #            q = angvec2quat(beta[0], [0, 1, 0])
        #            return rotvecquat(v, q)
        v1 = [v[0]*cos(beta[0]) + v[2]*sin(beta[0]),
              v[1],
              v[2]*cos(beta[0]) - v[0]*sin(beta[0])]

        v2 = [v1[0],
              v1[1]*cos(beta[1]) + v1[2]*sin(beta[1]),
              v1[2]*cos(beta[1]) - v1[1]*sin(beta[1])]

        v3 = [v2[0]*cos(beta[2]) + v2[1]*sin(beta[2]),
              v2[1]*cos(beta[2]) - v2[0]*sin(beta[1]),
              v2[2]]
            
        return v3

    def f_quat(beta, x, sphere_fit):
        sphere_fit = numpy.array(sphere_fit)
        n = [x[0]-sphere_fit[0], x[1]-sphere_fit[1], x[2]-sphere_fit[2]]
        q = [1 - vector.norm(beta[:3])] + list(beta[:3])
        q = angvec2quat(vector.norm(beta[:3]), beta[:3])
        m = map(lambda v : rotvecquat(vector.normalize(v), q), zip(n[0], n[1], n[2]))
#        m = map(lambda v : rot(v, beta), zip(n[0], n[1], n[2]))

        m = numpy.array(zip(*m))
        d = m[0]*x[3] + m[1]*x[4] + m[2]*x[5]
        return beta[3] - d

    quat_fit = FitLeastSq([0, 0, 0, 0], f_quat, (zpoints, sphere_fit))
    #    q = [1 - vector.norm(quat_fit[:3])] + list(quat_fit[:3])
    q = angvec2quat(vector.norm(quat_fit[:3]), quat_fit[:3])

    print 'quat fit', q, math.degrees(angle(q)), math.degrees(math.asin(quat_fit[3]))
    
    def f_rot(beta, x, sphere_fit):
        sphere_fit = numpy.array(sphere_fit)
        n = [x[0]-sphere_fit[0], x[1]-sphere_fit[1], x[2]-sphere_fit[2]]
        m = map(lambda v : rot(v, beta), zip(n[0], n[1], n[2]))
        m = numpy.array(zip(*m))

        d = m[0]*x[3] + m[1]*x[4] + m[2]*x[5]
        return beta[3] - d

    rot_fit = FitLeastSq([0, 0, 0, 0], f_rot, (zpoints, sphere_fit))
    print 'rot fit', rot_fit, math.degrees(rot_fit[0]), math.degrees(rot_fit[1]), math.degrees(rot_fit[2]), math.degrees(math.asin(min(1, max(-1, rot_fit[3]))))
    
    

if __name__ == '__main__':
    r = 38.0
    s = math.sin(math.pi/4) * r
    points = [[ r, 0, 0, 0, 0, 1],
              [ s*1.1, s, 0, 0, 0, 1],
              [ 0, r, 0, 0, 0, 1],
              [-s*1.1, s, 0, 0, 0, 1],
              [-r, 0, 0, 0, 0, 1],
              [-s,-s, 0, 0, 0, 1],
              [ 0,-r, 0, 0, 0, 1],
              [ s,-s, 0, 0, 0, 1],

              [ r, 0, 0, 0, 1, 0],
              [ s*1.1, 0, s, 0, 1, 0],
              [ 0, 0, r, 0, 1, 0],
              [-s, 0, s, 0, 1, 0],
              [-r, 0, 0, 0, 1, 0],
              [-s, 0,-s, 0, 1, 0],
              [ 0, 0,-r, 0, 1, 0],
              [ s, 0,-s, 0, 1, 0]]

    
    FitPoints(points, [0, 0, 0, r])
    
    #allpoints = [points1, points2, points3, points4, points5]
    #for points in allpoints:
    #    FitPoints(points, [0, 0, 0, r])
