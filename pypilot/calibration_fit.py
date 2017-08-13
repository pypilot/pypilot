#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import sys
import math
import time
import vector
from quaternion import *
import multiprocessing
from signalk.pipeserver import NonBlockingPipe

import numpy

debug=False
calibration_fit_period = 60  # run every 60 seconds

def FitLeastSq(beta0, f, zpoints, dimensions=1):
    try:
        import scipy.optimize
    except:
        print "failed to load scientific library, cannot perform calibration update!"
        return False

    leastsq = scipy.optimize.leastsq(f, beta0, zpoints)
    return list(leastsq[0])

def FitLeastSq_odr(beta0, f, zpoints, dimensions=1):
    try:
        import scipy.odr
    except:
        print 'failed to load scientific library, cannot perform calibration update!'
        return False

#    try:
    if True:
        Model = scipy.odr.Model(f, implicit=1)
        Data = scipy.odr.RealData(zpoints, dimensions)
        Odr = scipy.odr.ODR(Data, Model, beta0, maxit = 1000)
        output = Odr.run()
        return list(output.beta)
#    except:
#        print 'exception running odr fit!'
        return False

def ComputeDeviation(points, fit):
    m, d  = 0, 0
    for p in points:
        v = vector.sub(p[:3], fit[:3])
        m += (1 - vector.dot(v, v) / fit[3]**2)**2

        if len(fit) > 4:
            n = vector.dot(v, p[3:]) / vector.norm(v)
            if abs(n) <= 1:
                ang = math.degrees(math.asin(n))
                d += (fit[4] - ang)**2
            else:
                d += 1e111
    m /= len(points)
    d /= len(points)
    return [m**.5, d**.5]

def AvgPoint(points):
    # find average point
    avg = [0.0, 0.0, 0.0]
    for p in points:
        for i in range(3):
            avg[i] += p[i]
    for i in range(3):
        avg[i] /= len(points)
    return avg

def PointFit(points):
    avg = AvgPoint(points)
    dev = 0
    max_dev = 0
    for p in points:
        v = vector.sub(p[:3], avg)
        d = vector.dot(v, v)
        max_dev = max(d, max_dev)
        dev += d
    dev /= len(points)
    return avg, dev**.5, max_dev**.5

# fit points to line and plane
def LinearFit(points):
    zpoints = [[], [], []]
    for i in range(3):
        zpoints[i] = map(lambda x : x[i], points)
        
    data = numpy.array(zip(zpoints[0], zpoints[1], zpoints[2]))
    datamean = data.mean(axis=0)
    uu, dd, vv = numpy.linalg.svd(data - datamean)

    line_fit = [datamean, vv[0]]
    plane_fit = [datamean, vv[2]]

    line_dev = 0
    max_line_dev = 0
    plane_dev = 0
    max_plane_dev = 0
    for p in data:
        t = vector.dot(p, line_fit[1]) - vector.dot(line_fit[0], line_fit[1])
        q = map(lambda o, n : o + t*n, line_fit[0], line_fit[1])
        v = vector.sub(p, q)
        d = vector.dot(v, v)
        max_line_dev = max(d, max_line_dev)
        line_dev += d

        t = vector.dot(p, plane_fit[1]) - vector.dot(plane_fit[0], plane_fit[1])
        v = map(lambda b : t*b, plane_fit[1])
        d = vector.dot(v, v)
        max_plane_dev = max(d, max_plane_dev)
        plane_dev += d
        
    line_dev /= len(points)
    plane_dev /= len(points)

    line = [line_fit, line_dev**.5, max_line_dev**.5]
    plane = [plane_fit, plane_dev**.5, max_plane_dev**.5]
    return line, plane
    

def FitPoints(points, initial, norm):
    if len(points) < 5:
        return False

    if debug:
        print 'fitpoints initial', initial
    zpoints = [[], [], [], [], [], []]
    for i in range(6):
        zpoints[i] = map(lambda x : x[i], points)

    # determine if we have 0D, 1D, 2D, or 3D set of points
    point_fit, point_dev, point_max_dev = PointFit(points)
    if point_max_dev < 7:
        if debug:
            print '0d fit, insufficient data', point_dev, point_max_dev, '< 7'
        return False

    line, plane = LinearFit(points)
    line_fit, line_dev, line_max_dev = line
    plane_fit, plane_dev, plane_max_dev = plane
    
    # attempt 'normal' fit along normal vector
    '''
    def f_sphere1(beta, x):
        bias = map(lambda x, n: x + beta[0]*n, initial[:3], norm)
        b = numpy.matrix(map(lambda a, b : a - b, x[:3], bias))
        m = list(numpy.array(b.transpose()))
        r0 = map(lambda y : beta[1] - vector.norm(y), m)
        return r0
    sphere1d_fit = FitLeastSq([0, initial[3]], f_sphere1, zpoints)
    if not sphere1d_fit or sphere1d_fit[1] < 0:
        print 'FitLeastSq sphere1d failed!!!! ', len(points)
        return False
    sphere1d_fit = map(lambda x, n: x + sphere1d_fit[0]*n, initial[:3], norm) + [sphere1d_fit[1]]
    if debug:
        print 'sphere1 fit', sphere1d_fit, ComputeDeviation(points, sphere1d_fit)
    '''

    def f_new_sphere1(beta, x):
        bias = map(lambda x, n: x + beta[0]*n, initial[:3], norm)
        b = numpy.matrix(map(lambda a, b : a - b, x[:3], bias))
        m = list(numpy.array(b.transpose()))
        r0 = map(lambda y : beta[1] - vector.norm(y), m)
        g = list(numpy.array(numpy.matrix(x[3:]).transpose()))
        fac = .03 # weight deviation as 1 degree ~ .03 mag
        r1 = map(lambda y, z : fac*beta[1]*(beta[2] - math.degrees(math.asin(vector.dot(y, z)/vector.norm(y)))), m, g)
        return r0 + r1
    new_sphere1d_fit = FitLeastSq([0, initial[3], 90], f_new_sphere1, zpoints, 2)
    if not new_sphere1d_fit or new_sphere1d_fit[2] < 0:
        if debug:
            print 'FitLeastSq new_sphere1 failed!!!! ', len(points)
        new_sphere1d_fit = initial
    else:
        new_sphere1d_fit = map(lambda x, a: x + new_sphere1d_fit[0]*a, initial[:3], norm) + new_sphere1d_fit[1:]
    new_sphere1d_fit = [new_sphere1d_fit, ComputeDeviation(points, new_sphere1d_fit), 1]
        #print 'new sphere1 fit', new_sphere1d_fit
        
    if line_max_dev < 3:
        if debug:
            print 'line fit found, insufficient data', line_dev, line_max_dev
        return [new_sphere1d_fit, False, False]
    
    # 2d sphere fit across normal vector
    u = vector.cross(norm, [norm[1]-norm[2], norm[2]-norm[0], norm[0]-norm[1]])
    v = vector.cross(norm, u)
    u = vector.normalize(u)
    v = vector.normalize(v)
    '''
    def f_sphere2(beta, x):
        bias = map(lambda x, a, b: x + beta[0]*a + beta[1]*b, initial[:3], u, v)
        b = numpy.matrix(map(lambda a, b : a - b, x[:3], bias))
        m = list(numpy.array(b.transpose()))
        r0 = map(lambda y : beta[2] - vector.norm(y), m)
        return r0
    sphere2d_fit = FitLeastSq([0, 0, initial[3]], f_sphere2, zpoints)
    if not sphere2d_fit or sphere2d_fit[2] < 0:
        print 'FitLeastSq sphere2d failed!!!! ', len(points)
        new_sphere2d_fit = initial
    else:
        sphere2d_fit = map(lambda x, a, b: x + sphere2d_fit[0]*a + sphere2d_fit[1]*b, initial[:3], u, v) + [sphere2d_fit[2]]
    if debug:
        print 'sphere2 fit', sphere2d_fit, ComputeDeviation(points, sphere2d_fit)
    '''
    def f_new_sphere2(beta, x):
        bias = map(lambda x, a, b: x + beta[0]*a + beta[1]*b, initial[:3], u, v)
        b = numpy.matrix(map(lambda a, b : a - b, x[:3], bias))
        m = list(numpy.array(b.transpose()))
        r0 = map(lambda y : beta[2] - vector.norm(y), m)
        g = list(numpy.array(numpy.matrix(x[3:]).transpose()))
        fac = .03 # weight deviation as 1 degree ~ .03 mag
        r1 = map(lambda y, z : fac*beta[2]*(beta[3] - math.degrees(math.asin(vector.dot(y, z)/vector.norm(y)))), m, g)
        return r0 + r1
    new_sphere2d_fit = FitLeastSq([0, 0, initial[3], 90], f_new_sphere2, zpoints, 2)
    if not new_sphere2d_fit or new_sphere2d_fit[3] < 0:
        if debug:
            print 'FitLeastSq sphere failed!!!! ', len(points)
        return False
    new_sphere2d_fit = map(lambda x, a, b: x + new_sphere2d_fit[0]*a + new_sphere2d_fit[1]*b, initial[:3], u, v) + new_sphere2d_fit[2:]
    new_sphere2d_fit = [new_sphere2d_fit, ComputeDeviation(points, new_sphere2d_fit), 2]
    #print 'new sphere2 fit', new_sphere2d_fit

    if plane_max_dev < 1.2:
        if debug:
            print 'plane fit found, 2D fit only', plane_fit, plane_dev, plane_max_dev
        return [new_sphere1d_fit, new_sphere2d_fit, False]

    '''
    def f_sphere3(beta, x):
        bias = beta[:3]
        b = numpy.matrix(map(lambda a, b : a - b, x[:3], bias))
        m = list(numpy.array(b.transpose()))
        r0 = map(lambda y : beta[3] - vector.norm(y), m)
        return r0
    sphere3d_fit = FitLeastSq(initial[:4], f_sphere3, zpoints)
    if not sphere3d_fit or sphere3d_fit[3] < 0:
        print 'FitLeastSq sphere failed!!!! ', len(points)
        return False
    if debug:
        print 'sphere3 fit', sphere3d_fit, ComputeDeviation(points, sphere3d_fit)
    '''
    def f_new_sphere3(beta, x):
        b = numpy.matrix(map(lambda a, b : a - b, x[:3], beta[:3]))
        m = list(numpy.array(b.transpose()))
        r0 = map(lambda y : beta[3] - vector.norm(y), m)
        g = list(numpy.array(numpy.matrix(x[3:]).transpose()))
        fac = .03 # weight deviation as 1 degree ~ .03 mag
        r1 = map(lambda y, z : fac*beta[3]*(beta[4] - math.degrees(math.asin(vector.dot(y, z)/vector.norm(y)))), m, g)
        return r0 + r1
    new_sphere3d_fit = FitLeastSq(initial[:4] + [90], f_new_sphere3, zpoints, 2)
    if not new_sphere3d_fit or new_sphere3d_fit[3] < 0:
        if debug:
            print 'FitLeastSq sphere failed!!!! ', len(points)
        return False
    new_sphere3d_fit = [new_sphere3d_fit, ComputeDeviation(points, new_sphere3d_fit), 3]
    #print 'new sphere3 fit', new_sphere3d_fit
    
    return [new_sphere1d_fit, new_sphere2d_fit, new_sphere3d_fit]


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
    sigma = 1.2**2 # distance between sigma points
    #down_sigma = .05 # distance between down vectors
    max_sigma_points = 18

    def __init__(self):
        self.sigma_points = []
        self.lastpoint = False

    def AddPoint(self, compass, down):
        if not self.lastpoint:
            self.lastpoint = SigmaPoint(compass, down)
            return

        if vector.dist2(self.lastpoint.compass, compass) < SigmaPoints.sigma:
            fac = .02
            for i in range(3):
                self.lastpoint.compass[i] = fac*compass[i] + (1-fac)*self.lastpoint.compass[i]
                self.lastpoint.down[i] += down[i]
            self.lastpoint.count += 1
            return

        if self.lastpoint.count < 3: # require 3 measurements
            self.lastpoint = False
            return

        compass, down = self.lastpoint.compass, self.lastpoint.down
        for i in range(3):
            down[i] /= self.lastpoint.count
        self.lastpoint = False

        ind = 0
        for point in self.sigma_points:
            if vector.dist2(point.compass, compass) < SigmaPoints.sigma:
                point.add_measurement(compass, down)
                if ind > 0:
                    # put at front of list to speed up future tests
                    self.sigma_points = self.sigma_points[:ind-1] + [point] + \
                                        [self.sigma_points[ind-1]] + self.sigma_points[ind+1:]
                return
            ind += 1

        index = len(self.sigma_points)
        p = SigmaPoint(compass, down)
        if index == SigmaPoints.max_sigma_points:
            # replace point that is closest to other points
            minweighti = 0
            minweight = 1e20
            for i in range(len(self.sigma_points)):
                for j in range(len(self.sigma_points)):
                    if i == j:
                        continue
                    dist = vector.dist(self.sigma_points[i].compass, self.sigma_points[j].compass)
                    count = min(self.sigma_points[i].count, 100)
                    dt = time.time() - self.sigma_points[i].time
                    weight = dist * count**.2 * 1/dt**.1
                    #print 'ij', i, j, dist, count, dt, weight
                    if weight < minweight:
                        minweighti = i
                        minweight = weight

            #print 'replace', minweighti, self.sigma_points[minweighti].count, time.time() - self.sigma_points[minweighti].time
            self.sigma_points[minweighti] = p
        else:
            self.sigma_points.append(p)

    def RemoveOldest(self):
        oldest_sigma = self.sigma_points[0]
        for sigma in self.sigma_points:
            if sigma.time < oldest_sigma.time:
                oldest_sigma = sigma
        # don't remove if < 1 minute old
        if time.time() - oldest_sigma.time >= 60:
            self.sigma_points.remove(oldest_sigma)


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

def CalibrationProcess(points, norm_pipe, fit_output, initial):
    import os
    if os.system('sudo chrt -pi 0 %d 2> /dev/null > /dev/null' % os.getpid()):
      print 'warning, failed to make calibration process idle, trying renice'
      if os.system("renice 20 %d" % os.getpid()):
          print 'warning, failed to renice calibration process'

    cal = SigmaPoints()
    norm = [0, 0, 0]

    while True:
        # each iteration remove oldest point if we have more than 12
        #if len(cal.sigma_points) > 1:
         #   cal.RemoveOldest()
        
        t = time.time()
        addedpoint = False
        while time.time() - t < calibration_fit_period:
            p = points.recv(1)
            if p:
                cal.AddPoint(p[:3], p[3:6])
                addedpoint = True

        while True:
            n = norm_pipe.recv()
            if not n:
                break
            norm = n
            cal.sigma_points = []            
            #print 'set norm', norm

        if not addedpoint: # don't bother to run fit if no new data
            continue

        # remove points older than 1 hour
        p = []
        for sigma in cal.sigma_points:
            # only use measurements in last hour
            if time.time() - sigma.time < 3600:
                p.append(sigma)
        cal.sigma_points = p

        #inject
        if False:
            p = [[17.075,-31.64,-45.076,-0.086,-0.991,-0.1],[21.847,-30.689,-48.462,-0.05,-0.99,-0.133],[16.051,-32.177,-44.571,-0.097,-0.991,-0.094],[13.457,-31.441,-42.311,-0.098,-0.991,-0.089],[25.293,-31.523,-47.56,-0.138,-0.986,-0.087],[30.506,-30.632,-46.143,-0.13,-0.989,-0.066],[15.105,-31.553,-44.026,-0.101,-0.99,-0.093],[19.94,-31.41,-46.694,-0.082,-0.991,-0.102],[33.701,-30.181,-44.632,-0.13,-0.989,-0.069],[32.759,-30.278,-47.585,-0.113,-0.988,-0.103],[26.59,-31.057,-48.531,-0.095,-0.991,-0.095],[13.113,-31.935,-40.363,-0.082,-0.991,-0.102],[23.999,-31.035,-48.444,-0.114,-0.988,-0.104],[11.351,-31.323,-39.761,-0.113,-0.99,-0.079],[11.104,-31.363,-37.824,-0.103,-0.991,-0.087],[8.841,-30.778,-33.896,-0.111,-0.991,-0.078],[8.186,-30.269,-29.92,-0.102,-0.992,-0.077],[8.429,-30.116,-26.976,-0.098,-0.992,-0.078]]
            cal.sigma_points = []
            for q in p:
                cal.sigma_points.append(SigmaPoint(q[:3], q[3:6]))

        # attempt to perform least squares fit
        p = []
        for sigma in cal.sigma_points:
            p.append(sigma.compass + sigma.down)

        if debug:
            print 'FitPoints', len(p), norm

        # for now, require at least 6 points to agree well for update
        if len(p) < 6:
            continue

        gpoints = []
        for q in p:
            gpoints.append(q[3:])
            
        fit = FitPoints(p, initial, norm)
        if not fit:
            continue
        if debug:
            print 'fit', fit

        g_required_dev = .15 # must have more than this to allow 1d or 3d fit
        avg, g_dev, g_max_dev = PointFit(gpoints)
        if debug:
            print 'gdev', g_dev, g_max_dev
        if g_max_dev < g_required_dev:
            c = fit[1] # use 2d fit
            if debug:
                print 'sigmapoints flat, 2d fit only'
        else:
            c = fit[2] # 3d fit
            if not c:
                c = fit[0] # 1d fit only possible

        if not c:
            continue
                
        coverage = 360 - math.degrees(ComputeCoverage(cal.sigma_points, c[0][:3]))
        if coverage < 120: # require 120 degrees
            if debug:
                print 'calibration: not enough coverage', coverage, 'degrees'
            if c == fit[1]: # must have had 3d fit to use 1d fit
                continue
            c = fit[0] # 1d fit ok with insufficient coverage

        # make sure the magnitude is sane
        mag = c[0][3]
        if mag < 7 or mag > 80:
            if debug:
                print 'fit found field outside of normal earth field strength', mag
            continue

        # sphere fit should basically agree with new bias
        '''
        sbd = 0
        if fit[0] != fit[1]:
            spherebias = fit[2][:3]
            bias = fit[0][:3]
            sbd = vector.norm(vector.sub(bias, spherebias))
            if sbd > 6:
                if debug:
                    print 'sphere and newbias disagree', sbd
                    fit[0] = fit[1]
        print 'sphere bias difference', sbd
        '''
        # test points for deviation, all must fall on a sphere
        deviation = c[1]
        if deviation[0] > .15 or deviation[1] > 3:
            if debug:
                print 'bad fit:', deviation
            cal.RemoveOldest()            # remove oldest point if too much deviation
            continue # don't use this fit
        
        # if the bias has not sufficiently changed,
        # the fit didn't change much, so don't bother to report this update
        if vector.dist2(c[0], initial) < .1:
            if debug:
                print 'insufficient change in bias, calibration ok'
            continue

        if debug:
            print 'coverage', coverage, 'new fit:', c
        fit_output.send((c, map(lambda p : p.compass + p.down, cal.sigma_points)), False)
        initial = c[0]
                                 
class MagnetometerAutomaticCalibration(object):
    def __init__(self, cal_pipe, initial):
        self.cal_pipe = cal_pipe
        self.sphere_fit = initial
        points, self.points = NonBlockingPipe('points pipe', True)
        norm_pipe, self.norm_pipe = NonBlockingPipe('norm pipe', True)
        self.fit_output, fit_output = NonBlockingPipe('fit output', True)

        self.process = multiprocessing.Process(target=CalibrationProcess, args=(points, norm_pipe, fit_output, self.sphere_fit))
        #print 'start cal process'
        self.process.start()

    def __del__(self):
        print 'terminate calibration process'
        self.process.terminate()

    def AddPoint(self, point):
        self.points.send(point, False)

    def SetNorm(self, norm):
        self.norm_pipe.send(norm)
    
    def UpdatedCalibration(self):
        result = self.fit_output.recv()
        if not result:
            return

        # use new bias fit
        self.cal_pipe.send(tuple(result[0][0][:3]))
        return result

def ExtraFit():
    ellipsoid_fit = False
    '''
    if len(points) >= 10:
        def f_ellipsoid3(beta, x):
            return (x[0]-beta[0])**2 + (beta[4]*(x[1]-beta[1]))**2 + (beta[5]*(x[2]-beta[2]))**2 - beta[3]**2
        ellipsoid_fit = FitLeastSq(sphere_fit + [1, 1], f_ellipsoid3, zpoints)
        print 'ellipsoid_fit', ellipsoid_fit
    '''

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
    debug = True
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

    #FitPoints(points, [0, 0, 0, r, 0])

    points = [[9.076,19.17,32.66,-0.078,-0.037,0.996],[8.106,14.431,32.2,-0.077,-0.042,0.996],[9.184,16.653,32.451,-0.07,-0.032,0.997],[11.645,21.557,32.988,-0.077,-0.042,0.996],[20.508,27.569,32.798,-0.075,-0.044,0.996],[22.091,28.787,32.86,-0.076,-0.046,0.996],[11.541,19.82,32.848,-0.075,-0.046,0.996],[10.679,18.367,32.569,-0.076,-0.043,0.996],[8.628,11.927,31.855,-0.075,-0.045,0.996],[14.149,22.908,33.247,-0.072,-0.04,0.997],[18.136,25.664,32.971,-0.074,-0.038,0.997],[16.213,24.721,33.405,-0.071,-0.048,0.996]]
    #points += [[9.0, 19.0, 0.0], [21, -30, -10]]
    #points = [[1, 1, 0], [0, 2, 0], [0, 1, 1], [-1, 1, 0], [0, 0, 0], [0, 1, -1]]

    points = [[11.764,-1.151,27.153,-0.065,-0.105,0.992],[11.702,3.219,26.906,-0.057,-0.112,0.992],[10.517,1.995,27.191,-0.062,-0.117,0.991],[11.133,5.738,26.847,-0.045,-0.115,0.992],[12.842,7.647,27.133,-0.039,-0.106,0.994],[15.971,16.307,25.975,-0.047,-0.107,0.993],[13.051,5.068,27.136,-0.038,-0.123,0.992],[17.595,20.089,25.537,-0.062,-0.096,0.993],[21.079,22.803,24.875,-0.05,-0.107,0.993],[19.639,21.227,25.403,-0.058,-0.099,0.993],[23.543,24.305,25.153,-0.046,-0.111,0.993],[17.631,18.304,26.199,-0.035,-0.125,0.992],[13.984,14.98,25.636,-0.06,-0.108,0.992],[13.453,11.609,26.173,-0.056,-0.113,0.992],[11.932,0.497,27.024,-0.049,-0.103,0.993],[11.972,10.353,26.104,-0.065,-0.106,0.992],[12.784,-2.731,27.486,-0.049,-0.106,0.993],[13.686,2.345,27.619,-0.023,-0.134,0.991]]

#    points = [[12.556,10.052,26.396,-0.064,-0.107,0.992],[12.258,-1.208,27.648,-0.062,-0.105,0.992],[26.41,-10.516,28.514,-0.123,-0.063,0.99],[14.982,-7.251,27.337,-0.065,-0.099,0.993],[11.067,5.68,26.855,-0.064,-0.11,0.992],[14.174,12.875,26.003,-0.053,-0.112,0.992],[12.148,0.924,27.725,-0.032,-0.135,0.99],[20.759,-10.419,28.529,-0.108,-0.073,0.991],[29.375,-9.592,28.482,-0.126,-0.06,0.99],[18.678,-9.438,28.597,-0.063,-0.087,0.994],[23.277,-11.94,28.369,-0.101,-0.078,0.992],[12.625,11.604,25.661,-0.067,-0.106,0.992],[31.725,-8.313,28.618,-0.128,-0.059,0.99],[11.842,7.645,26.323,-0.067,-0.104,0.992],[13.248,-3.298,27.743,-0.041,-0.123,0.991],[14.121,-5.302,27.771,-0.046,-0.118,0.992],[13.975,-1.675,27.695,-0.043,-0.108,0.993]]

    # does it need bias???
#    points = [[45.562,-5.522,33.315,-0.061,-0.092,0.994],[23.026,-8.641,28.703,-0.066,-0.094,0.993],[20.332,-6.312,28.221,-0.063,-0.093,0.994],[16.104,-1.745,26.547,-0.061,-0.095,0.994],[25.644,-9.722,29.58,-0.055,-0.089,0.994],[14.588,1.891,25.98,-0.064,-0.09,0.994],[29.512,-10.713,30.51,-0.064,-0.093,0.994],[35.995,-10.538,31.583,-0.067,-0.093,0.993],[14.685,4.282,26.031,-0.04,-0.086,0.995],[43.786,-6.879,32.929,-0.066,-0.094,0.993],[39.797,-9.533,32.066,-0.067,-0.094,0.993],[18.007,1.79,27.622,-0.019,-0.083,0.996],[18.014,-3.11,27.07,-0.06,-0.092,0.994],[17.014,4.562,27.521,-0.007,-0.077,0.997],[15.951,10.254,26.188,-0.006,-0.066,0.998],[12.914,3.218,25.17,-0.106,-0.103,0.989],[16.371,7.899,26.567,-0.021,-0.077,0.997],[12.094,6.481,24.372,-0.102,-0.108,0.989]]


#    points = [[9.547,15.814,25.357,-0.095,-0.093,0.991],[12.748,0.766,28.28,-0.031,-0.11,0.993],[8.354,13.292,25.38,-0.146,-0.1,0.984],[7.681,0.778,25.9,-0.132,-0.112,0.985],[13.14,12.719,27.497,-0.022,-0.136,0.99],[10.46,-0.396,27.08,-0.088,-0.116,0.989],[13.393,5.816,28.676,-0.015,-0.103,0.995],[5.794,11.304,24.671,-0.174,-0.097,0.98],[10.963,12.799,26.198,-0.073,-0.123,0.99],[18.058,-10.868,28.864,-0.109,-0.11,0.988],[23.494,-12.895,30.156,-0.075,-0.11,0.99],[29.441,-13.663,30.612,-0.098,-0.118,0.988]]
    points = [[22.26,-8.318,20.497,-0.003,-0.122,0.993],[20.502,-7.608,20.276,-0.004,-0.123,0.992],[29.682,-11.309,21.141,-0.012,-0.104,0.994],[40.273,-8.135,21.231,-0.044,-0.136,0.99],[26.505,-11.144,20.312,-0.021,-0.131,0.991],[38.574,-8.504,21.746,-0.038,-0.117,0.992],[41.647,-6.351,21.824,-0.041,-0.12,0.992],[33.8,-11.266,21.233,-0.021,-0.118,0.993],[35.684,-10.43,21.714,-0.014,-0.119,0.993],[37.737,-10.095,21.298,-0.015,-0.13,0.991],[25.122,-10.063,20.654,-0.006,-0.12,0.993],[44.095,-3.681,21.099,-0.037,-0.123,0.992]]
    points = [[17.075,-31.64,-45.076,-0.086,-0.991,-0.1],[21.847,-30.689,-48.462,-0.05,-0.99,-0.133],[16.051,-32.177,-44.571,-0.097,-0.991,-0.094],[13.457,-31.441,-42.311,-0.098,-0.991,-0.089],[25.293,-31.523,-47.56,-0.138,-0.986,-0.087],[30.506,-30.632,-46.143,-0.13,-0.989,-0.066],[15.105,-31.553,-44.026,-0.101,-0.99,-0.093],[19.94,-31.41,-46.694,-0.082,-0.991,-0.102],[33.701,-30.181,-44.632,-0.13,-0.989,-0.069],[32.759,-30.278,-47.585,-0.113,-0.988,-0.103],[26.59,-31.057,-48.531,-0.095,-0.991,-0.095],[13.113,-31.935,-40.363,-0.082,-0.991,-0.102],[23.999,-31.035,-48.444,-0.114,-0.988,-0.104],[11.351,-31.323,-39.761,-0.113,-0.99,-0.079],[11.104,-31.363,-37.824,-0.103,-0.991,-0.087],[8.841,-30.778,-33.896,-0.111,-0.991,-0.078],[8.186,-30.269,-29.92,-0.102,-0.992,-0.077],[8.429,-30.116,-26.976,-0.098,-0.992,-0.078]]

    #misaligned_points
    points = [[65.299,7.578,0.454,0.902,0.133,0.41],[74.488,4.087,-12.591,0.906,0.105,0.409],[83.525,22.406,-27.578,0.904,0.091,0.417],[82.467,30.119,-26.81,0.901,0.099,0.422],[61.307,30.917,3.18,0.899,0.127,0.419],[67.521,4.139,-3.877,0.905,0.106,0.411],[60.373,23.948,5.609,0.904,0.116,0.41],[71.203,42.094,-12.381,0.904,0.11,0.413],[61.773,34.322,0.838,0.901,0.12,0.416],[75.925,41.229,-20.102,0.9,0.113,0.421],[74.003,41.597,-15.45,0.904,0.105,0.415],[80.875,12.007,-23.44,0.902,0.107,0.417],[63.748,37.645,-1.744,0.899,0.12,0.421],[82.446,16.972,-26.449,0.901,0.113,0.418],[66.011,6.755,-1.007,0.903,0.13,0.41],[81.506,34.262,-26.143,0.903,0.114,0.415],[76.887,6.445,-16.939,0.902,0.135,0.41],[66.601,41.088,-6.481,0.9,0.13,0.416]]

    points = [[45.272,-31.058,-52.332,-0.052,-0.991,-0.125],[24.568,-32.022,-56.101,-0.071,-0.99,-0.121],[42.653,-29.042,-20.928,-0.05,-0.993,-0.107],[30.213,-29.224,-16.322,-0.071,-0.991,-0.111],[14.084,-29.238,-25.445,-0.064,-0.991,-0.115],[50.121,-30.268,-36.204,-0.046,-0.993,-0.112],[48.689,-31.376,-44.139,-0.071,-0.991,-0.111],[24.09,-29.268,-17.286,-0.062,-0.992,-0.112],[32.664,-32.406,-56.606,-0.031,-0.993,-0.116],[13.186,-30.979,-43.341,-0.062,-0.992,-0.106],[12.31,-30.229,-34.594,-0.043,-0.993,-0.113],[38.382,-29.641,-17.973,-0.05,-0.993,-0.11],[17.997,-31.383,-53.101,-0.077,-0.991,-0.107],[45.486,-31.693,-50.546,-0.061,-0.991,-0.118],[14.923,-31.307,-48.001,-0.068,-0.992,-0.107],[16.873,-29.398,-20.403,-0.051,-0.993,-0.104],[47.566,-29.815,-29.019,-0.063,-0.992,-0.108],[45.311,-29.207,-25.163,-0.07,-0.992,-0.108]]

    points = [[24.399,18.208,-69.99,0.065,-0.07,-0.995],[62.035,12.139,-71.774,0.072,-0.027,-0.997],[39.638,-9.873,-67.951,0.069,-0.076,-0.995],[55.201,24.424,-73.12,0.08,-0.079,-0.994],[43.593,-8.991,-67.82,0.063,-0.057,-0.996],[47.125,-9.127,-67.938,0.07,-0.087,-0.994],[61.677,5.153,-70.407,0.09,-0.109,-0.99],[54.685,-3.623,-69.439,0.085,-0.071,-0.994],[62.2,10.506,-71.378,0.078,-0.025,-0.997],[32.466,-10.595,-66.422,0.064,-0.082,-0.995],[29.802,21.995,-70.758,0.069,-0.113,-0.991],[61.92,7.391,-70.907,0.085,-0.128,-0.988],[32.16,24.8,-71.531,0.063,-0.074,-0.995],[43.171,27.456,-72.484,0.068,-0.099,-0.993],[21.07,12.538,-68.628,0.067,-0.077,-0.995],[20.503,0.553,-66.75,0.073,-0.088,-0.993],[22.263,-2.934,-66.945,0.073,-0.097,-0.993],[19.978,5.812,-67.76,0.07,-0.094,-0.993]]
    FitPoints(points, [25, 14, 0, 30], [0, 0, 1])
    
    #allpoints = [points1, points2, points3, points4, points5]
    #for points in allpoints:
    #    FitPoints(points, [0, 0, 0, r])
