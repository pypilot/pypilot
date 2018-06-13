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

debug=True
calibration_fit_period = 10  # run every 60 seconds

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
    

def FitPoints(points, current, norm):
    if len(points) < 5:
        return False

    # ensure current and norm are float
    current = map(float, current)
    norm = map(float, norm)

    zpoints = [[], [], [], [], [], []]
    for i in range(6):
        zpoints[i] = map(lambda x : x[i], points)
        
    # determine if we have 0D, 1D, 2D, or 3D set of points
    point_fit, point_dev, point_max_dev = PointFit(points)
    if point_max_dev < 9:
        if debug:
            print '0d fit, insufficient data', point_dev, point_max_dev, '< 9'
        return False

    line, plane = LinearFit(points)
    line_fit, line_dev, line_max_dev = line
    plane_fit, plane_dev, plane_max_dev = plane

    # initial guess average min and max for bias, and average range for radius
    minc = [1000, 1000, 1000]
    maxc = [-1000, -1000, -1000]
    for p in points:
        minc = map(min, p[:3], minc)
        maxc = map(max, p[:3], maxc)

    guess = map(lambda a, b : (a+b)/2, minc, maxc)
    diff = map(lambda a, b : b-a, minc, maxc)
    guess.append((diff[0]+diff[1]+diff[2])/3)
    if debug:
        print 'initial guess', guess

    # initial is the closest to guess on the uv plane containing current
    initial = vector.add(current[:3], vector.project(vector.sub(guess[:3], current[:3]), norm))
    initial.append(current[3])
    if debug:
        print 'initial 1d fit', initial

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
        fac = 1 # weight deviation
        def dip(y, z):
            n = min(max(vector.dot(y, z)/vector.norm(y), -1), 1)
            return n
        r1 = map(lambda y, z : fac*beta[1]*(beta[2]-dip(y, z)), m, g)
        return r0 + r1
    new_sphere1d_fit = FitLeastSq([0, initial[3], 0], f_new_sphere1, zpoints, 2)
    if not new_sphere1d_fit or new_sphere1d_fit[1] < 0:
        if debug:
            print 'FitLeastSq new_sphere1 failed!!!! ', len(points)
        new_sphere1d_fit = current
    else:
        new_sphere1d_fit = map(lambda x, a: x + new_sphere1d_fit[0]*a, initial[:3], norm) + [new_sphere1d_fit[1], math.degrees(math.asin(new_sphere1d_fit[2]))]
    new_sphere1d_fit = [new_sphere1d_fit, ComputeDeviation(points, new_sphere1d_fit), 1]
        #print 'new sphere1 fit', new_sphere1d_fit

    if line_max_dev < 2:
        if debug:
            print 'line fit found, insufficient data', line_dev, line_max_dev
        return False
    
    # 2d sphere fit across normal vector
    u = vector.cross(norm, [norm[1]-norm[2], norm[2]-norm[0], norm[0]-norm[1]])
    v = vector.cross(norm, u)
    u = vector.normalize(u)
    v = vector.normalize(v)

    # initial is the closest to guess on the uv plane containing current
    initial = vector.add(guess[:3], vector.project(vector.sub(current[:3], guess[:3]), norm))
    initial.append(current[3])
    if debug:
        print 'initial 2d fit', initial
    
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
        #r0 = map(lambda y : 1 - vector.norm(y)/beta[2], m)
        g = list(numpy.array(numpy.matrix(x[3:]).transpose()))
        fac = 1 # weight deviation
        def dip(y, z):
            n = min(max(vector.dot(y, z)/vector.norm(y), -1), 1)
            return n
        r1 = map(lambda y, z : fac*beta[2]*(beta[3]-dip(y, z)), m, g)
        return r0 + r1
    new_sphere2d_fit = FitLeastSq([0, 0, initial[3], 0], f_new_sphere2, zpoints, 2)
    if not new_sphere2d_fit or new_sphere2d_fit[2] < 0:
        if debug:
            print 'FitLeastSq sphere2 failed!!!! ', len(points)
        return False
    new_sphere2d_fit = map(lambda x, a, b: x + new_sphere2d_fit[0]*a + new_sphere2d_fit[1]*b, initial[:3], u, v) + [new_sphere2d_fit[2], math.degrees(math.asin(new_sphere2d_fit[3]))]
    new_sphere2d_fit = [new_sphere2d_fit, ComputeDeviation(points, new_sphere2d_fit), 2]

    if plane_max_dev < 1.2:
        ang = math.degrees(math.asin(vector.norm(vector.cross(plane_fit[1], norm))))
        
        if debug:
            print 'plane fit found, 2D fit only', ang, plane_fit, plane_dev, plane_max_dev
        if ang > 16: # is 15 degrees too much?
            if debug:
                print 'angle of plane not aligned to normal: no 2d fit'
            new_sphere2d_fit = False

        return [new_sphere1d_fit, new_sphere2d_fit, False]

    # ok to use best guess for 3d fit
    initial = guess
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
        #r0 = map(lambda y : 1 - vector.norm(y)/beta[3], m)
        g = list(numpy.array(numpy.matrix(x[3:]).transpose()))
        fac = 1 # weight deviation
        def dip(y, z):
            n = min(max(vector.dot(y, z)/vector.norm(y), -1), 1)
            return n
        r1 = map(lambda y, z : fac*beta[3]*(beta[4]-dip(y, z)), m, g)

        return r0 + r1
    new_sphere3d_fit = FitLeastSq(initial[:4] + [0], f_new_sphere3, zpoints, 2)
    if not new_sphere3d_fit or new_sphere3d_fit[3] < 0 or abs(new_sphere3d_fit[4]) >= 1:
        if debug:
            print 'FitLeastSq sphere3 failed!!!! ', len(points)
        return False
    new_sphere3d_fit[4] = math.degrees(math.asin(new_sphere3d_fit[4]))
    new_sphere3d_fit = [new_sphere3d_fit, ComputeDeviation(points, new_sphere3d_fit), 3]
    #if debug:
     #   print 'new sphere3 fit', new_sphere3d_fit
    
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

# calculate the largest angle in radians between any two measurements
# for a given calibration bias and normal vector
def ComputeCoverage(sigma_points, bias, norm):
    q = vec2vec2quat(norm, [0, 0, 1])
    def ang(p):
        c = rotvecquat(vector.sub(p.compass, bias), q)
        d = rotvecquat(p.down, q)
        v = rotvecquat(c, vec2vec2quat(d, [0, 0, 1]))
        v = vector.normalize(v)
        return math.degrees(math.atan2(v[1], v[0])), abs(math.degrees(math.acos(v[2])))

    angles = []
    for a in map(ang, sigma_points):
        if True or a[1] > 10:  # only use points more than 10 degrees from magnetic vector
            angles.append(a[0])
    angles = sorted(angles)
                    
    max_diff = 0
    for i in range(len(angles)):
        diff = -angles[i]
        j = i+1
        if j == len(angles):
            diff += 360
            j = 0
        diff += angles[j]
        max_diff = max(max_diff, diff)
    if max_diff == 0:
        return 360
    return max_diff    

def CalibrationProcess(points, norm_pipe, fit_output, current):
    import os
    if os.system('sudo chrt -pi 0 %d 2> /dev/null > /dev/null' % os.getpid()):
      print 'warning, failed to make calibration process idle, trying renice'
      if os.system("renice 20 %d" % os.getpid()):
          print 'warning, failed to renice calibration process'

    cal = SigmaPoints()
    norm = [0, 0, 1]
    lastp = []

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
            p = [[99.61022395739637, -32.0896372233238, -46.19833893428187,0.9959545553521548, 0.01881086049517002,  -0.08709012682673913], [99.41269965838998,  -29.18933915381635, -50.87318433884822, 0.9959240391187345,  0.025598832598336896, -0.08624850966044939], [99.63371012917594, -30.564020033256263, -49.51652671143861,  0.9959725515496398, 0.021270918093455812,  -0.08636070074997991], [99.55269985551303,  -31.555431659129688, -48.67149985377176, 0.996239375494907,  0.008675418089256124, -0.08608916000100958], [99.85053925286313, -32.804657891397724, -43.07924450281722,  0.9958921542039714, 0.020566397536647005,  -0.08774229505974987], [99.09234339278912,  -33.6539144001473, -39.992952093580584, 0.9960624906154125,  0.012102753257664767, -0.08734126093468247], [99.36440514827613, -27.83050871518071, -23.86624704383829,  0.9961120564762204, 0.010738915365523884,  -0.08721644126707587], [99.27881542065676,  -31.07749102140572, -29.11323456675227, 0.9959733742690939,  0.018797671301392187, -0.08734021276986086], [98.66466785647648, -33.77910428722979, -35.88069749493785,  0.9960232490868438, 0.00878364968675116,  -0.08807145988954777], [98.25940791866283,  -33.38075607725948, -43.511240257178336, 0.9959494378584233,  0.008111556392145083, -0.08921317309205794], [99.23090625079405, -31.96668639422272, -31.79659116533928,  0.995697680058246, 0.021258988302123628,  -0.08989484314049596], [98.05279227664792,  -34.04676396012794, -38.32184662393996, 0.9959648038133849,  0.007050180604718743, -0.08913339420101368], [99.3094944746132, -33.063143800282035, -37.07277486136846,  0.9959314585521511, 0.021396914991636075,  -0.0870363232937691], [99.46857288800534, -25.70920675991828, -22.212079862195036, 0.9960765103312297, 0.022150356953854627, -0.0853784440140581], [98.06189470192125, -33.6074115180095, -33.95455487369483,  0.9959882312382028, 0.008958823339376289,  -0.08858464837243196], [97.91227795273693,  -31.233531578042207, -27.844580754671917,  0.9959462962950375, 0.009456129369926113,  -0.08901938128629384], [97.4962095079903,  -33.224780551340096, -31.15958596160746, 0.9959328787036706,  0.003916703460782957, -0.0896401065300612], [97.94596614832525, -32.33912173753667, -29.559069049234942,  0.9959474803084559, 0.007237095190038474,  -0.08914248460492485]]
            norm = [0.9957405037490841, 0.010161766950568168, -0.09163835270214449]
            cal.sigma_points = []
            for q in p:
                cal.sigma_points.append(SigmaPoint(q[:3], q[3:6]))

        # attempt to perform least squares fit
        p = []
        for sigma in cal.sigma_points:
            p.append(sigma.compass + sigma.down)
        
        # for now, require at least 6 points to agree well for update
        if len(p) < 6:
            continue

        gpoints = []
        for q in p:
            gpoints.append(q[3:])
            
        if debug:
            print 'FitPoints', p, current, norm

        fit = FitPoints(p, current, norm)
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
                print 'sigmapoints flat, 2D fit only'
        else:
            c = fit[2] # 3d fit
            if not c:
                c = fit[0] # 1d fit only possible

        if not c:
            continue

        
        coverage = 360 - ComputeCoverage(cal.sigma_points, c[0][:3], norm)
        if coverage < 20: # require 120 degrees
            if debug:
                print 'calibration: not enough coverage', coverage, 'degrees'
            if c == fit[1]: # must have had 3d fit to use 1d fit
                continue
            if debug:
                print 'insufficient coverage, use 1d fit'
            c = fit[0] # 1d fit ok with insufficient coverage

        if c == fit[0]: # 1d fit must be upgraded
            if not lastp:
                if debug:
                    print 'no previous cal, no fit'
                continue
            fit2 = FitPoints(p+lastp, current, norm)
            if debug:
                print 'upgraded was fit', fit[0]
            if not fit2:
                continue
            if debug:
                print 'upgraded fit to', fit2[0]
            c = fit2[0]
            if not c:
                continue
            c[2] = 'u' # mark as upgraded fit

        # make sure the magnitude is sane
        mag = c[0][3]
        if mag < 7 or mag > 180:
            if debug:
                print 'fit found field outside of normal earth field strength', mag
            continue

        # require inclination less than 82 degrees, with so much inclination,
        # the fit is inaccurate (near magnetic pole?)
        inc = c[0][4]
        if abs(inc) > 82:
            if debug:
                print 'incline greater than 82 degrees, no fit',
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
        if vector.dist2(c[0], current) < .1:
            if debug:
                print 'insufficient change in bias, calibration already ok'
            #continue

        if debug:
            print 'coverage', coverage, 'new fit:', c
        if c[2] != 'u': # save working points for future 1d fit update
            lastp = p
        fit_output.send((c, map(lambda p : p.compass + p.down, cal.sigma_points)), False)
        current = c[0]
                                 
class MagnetometerAutomaticCalibration(object):
    def __init__(self, cal_pipe, current):
        self.cal_pipe = cal_pipe
        self.sphere_fit = current
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

#    points = [[11.764,-1.151,27.153,-0.065,-0.105,0.992],[11.702,3.219,26.906,-0.057,-0.112,0.992],[10.517,1.995,27.191,-0.062,-0.117,0.991],[11.133,5.738,26.847,-0.045,-0.115,0.992],[12.842,7.647,27.133,-0.039,-0.106,0.994],[15.971,16.307,25.975,-0.047,-0.107,0.993],[13.051,5.068,27.136,-0.038,-0.123,0.992],[17.595,20.089,25.537,-0.062,-0.096,0.993],[21.079,22.803,24.875,-0.05,-0.107,0.993],[19.639,21.227,25.403,-0.058,-0.099,0.993],[23.543,24.305,25.153,-0.046,-0.111,0.993],[17.631,18.304,26.199,-0.035,-0.125,0.992],[13.984,14.98,25.636,-0.06,-0.108,0.992],[13.453,11.609,26.173,-0.056,-0.113,0.992],[11.932,0.497,27.024,-0.049,-0.103,0.993],[11.972,10.353,26.104,-0.065,-0.106,0.992],[12.784,-2.731,27.486,-0.049,-0.106,0.993],[13.686,2.345,27.619,-0.023,-0.134,0.991]]

#    points = [[12.556,10.052,26.396,-0.064,-0.107,0.992],[12.258,-1.208,27.648,-0.062,-0.105,0.992],[26.41,-10.516,28.514,-0.123,-0.063,0.99],[14.982,-7.251,27.337,-0.065,-0.099,0.993],[11.067,5.68,26.855,-0.064,-0.11,0.992],[14.174,12.875,26.003,-0.053,-0.112,0.992],[12.148,0.924,27.725,-0.032,-0.135,0.99],[20.759,-10.419,28.529,-0.108,-0.073,0.991],[29.375,-9.592,28.482,-0.126,-0.06,0.99],[18.678,-9.438,28.597,-0.063,-0.087,0.994],[23.277,-11.94,28.369,-0.101,-0.078,0.992],[12.625,11.604,25.661,-0.067,-0.106,0.992],[31.725,-8.313,28.618,-0.128,-0.059,0.99],[11.842,7.645,26.323,-0.067,-0.104,0.992],[13.248,-3.298,27.743,-0.041,-0.123,0.991],[14.121,-5.302,27.771,-0.046,-0.118,0.992],[13.975,-1.675,27.695,-0.043,-0.108,0.993]]

    # does it need bias???
#    points = [[45.562,-5.522,33.315,-0.061,-0.092,0.994],[23.026,-8.641,28.703,-0.066,-0.094,0.993],[20.332,-6.312,28.221,-0.063,-0.093,0.994],[16.104,-1.745,26.547,-0.061,-0.095,0.994],[25.644,-9.722,29.58,-0.055,-0.089,0.994],[14.588,1.891,25.98,-0.064,-0.09,0.994],[29.512,-10.713,30.51,-0.064,-0.093,0.994],[35.995,-10.538,31.583,-0.067,-0.093,0.993],[14.685,4.282,26.031,-0.04,-0.086,0.995],[43.786,-6.879,32.929,-0.066,-0.094,0.993],[39.797,-9.533,32.066,-0.067,-0.094,0.993],[18.007,1.79,27.622,-0.019,-0.083,0.996],[18.014,-3.11,27.07,-0.06,-0.092,0.994],[17.014,4.562,27.521,-0.007,-0.077,0.997],[15.951,10.254,26.188,-0.006,-0.066,0.998],[12.914,3.218,25.17,-0.106,-0.103,0.989],[16.371,7.899,26.567,-0.021,-0.077,0.997],[12.094,6.481,24.372,-0.102,-0.108,0.989]]


#    points = [[9.547,15.814,25.357,-0.095,-0.093,0.991],[12.748,0.766,28.28,-0.031,-0.11,0.993],[8.354,13.292,25.38,-0.146,-0.1,0.984],[7.681,0.778,25.9,-0.132,-0.112,0.985],[13.14,12.719,27.497,-0.022,-0.136,0.99],[10.46,-0.396,27.08,-0.088,-0.116,0.989],[13.393,5.816,28.676,-0.015,-0.103,0.995],[5.794,11.304,24.671,-0.174,-0.097,0.98],[10.963,12.799,26.198,-0.073,-0.123,0.99],[18.058,-10.868,28.864,-0.109,-0.11,0.988],[23.494,-12.895,30.156,-0.075,-0.11,0.99],[29.441,-13.663,30.612,-0.098,-0.118,0.988]]
#    points = [[22.26,-8.318,20.497,-0.003,-0.122,0.993],[20.502,-7.608,20.276,-0.004,-0.123,0.992],[29.682,-11.309,21.141,-0.012,-0.104,0.994],[40.273,-8.135,21.231,-0.044,-0.136,0.99],[26.505,-11.144,20.312,-0.021,-0.131,0.991],[38.574,-8.504,21.746,-0.038,-0.117,0.992],[41.647,-6.351,21.824,-0.041,-0.12,0.992],[33.8,-11.266,21.233,-0.021,-0.118,0.993],[35.684,-10.43,21.714,-0.014,-0.119,0.993],[37.737,-10.095,21.298,-0.015,-0.13,0.991],[25.122,-10.063,20.654,-0.006,-0.12,0.993],[44.095,-3.681,21.099,-0.037,-0.123,0.992]]
#    points = [[17.075,-31.64,-45.076,-0.086,-0.991,-0.1],[21.847,-30.689,-48.462,-0.05,-0.99,-0.133],[16.051,-32.177,-44.571,-0.097,-0.991,-0.094],[13.457,-31.441,-42.311,-0.098,-0.991,-0.089],[25.293,-31.523,-47.56,-0.138,-0.986,-0.087],[30.506,-30.632,-46.143,-0.13,-0.989,-0.066],[15.105,-31.553,-44.026,-0.101,-0.99,-0.093],[19.94,-31.41,-46.694,-0.082,-0.991,-0.102],[33.701,-30.181,-44.632,-0.13,-0.989,-0.069],[32.759,-30.278,-47.585,-0.113,-0.988,-0.103],[26.59,-31.057,-48.531,-0.095,-0.991,-0.095],[13.113,-31.935,-40.363,-0.082,-0.991,-0.102],[23.999,-31.035,-48.444,-0.114,-0.988,-0.104],[11.351,-31.323,-39.761,-0.113,-0.99,-0.079],[11.104,-31.363,-37.824,-0.103,-0.991,-0.087],[8.841,-30.778,-33.896,-0.111,-0.991,-0.078],[8.186,-30.269,-29.92,-0.102,-0.992,-0.077],[8.429,-30.116,-26.976,-0.098,-0.992,-0.078]]

    #misaligned_points
    #points = [[65.299,7.578,0.454,0.902,0.133,0.41],[74.488,4.087,-12.591,0.906,0.105,0.409],[83.525,22.406,-27.578,0.904,0.091,0.417],[82.467,30.119,-26.81,0.901,0.099,0.422],[61.307,30.917,3.18,0.899,0.127,0.419],[67.521,4.139,-3.877,0.905,0.106,0.411],[60.373,23.948,5.609,0.904,0.116,0.41],[71.203,42.094,-12.381,0.904,0.11,0.413],[61.773,34.322,0.838,0.901,0.12,0.416],[75.925,41.229,-20.102,0.9,0.113,0.421],[74.003,41.597,-15.45,0.904,0.105,0.415],[80.875,12.007,-23.44,0.902,0.107,0.417],[63.748,37.645,-1.744,0.899,0.12,0.421],[82.446,16.972,-26.449,0.901,0.113,0.418],[66.011,6.755,-1.007,0.903,0.13,0.41],[81.506,34.262,-26.143,0.903,0.114,0.415],[76.887,6.445,-16.939,0.902,0.135,0.41],[66.601,41.088,-6.481,0.9,0.13,0.416]]

#    points = [[45.272,-31.058,-52.332,-0.052,-0.991,-0.125],[24.568,-32.022,-56.101,-0.071,-0.99,-0.121],[42.653,-29.042,-20.928,-0.05,-0.993,-0.107],[30.213,-29.224,-16.322,-0.071,-0.991,-0.111],[14.084,-29.238,-25.445,-0.064,-0.991,-0.115],[50.121,-30.268,-36.204,-0.046,-0.993,-0.112],[48.689,-31.376,-44.139,-0.071,-0.991,-0.111],[24.09,-29.268,-17.286,-0.062,-0.992,-0.112],[32.664,-32.406,-56.606,-0.031,-0.993,-0.116],[13.186,-30.979,-43.341,-0.062,-0.992,-0.106],[12.31,-30.229,-34.594,-0.043,-0.993,-0.113],[38.382,-29.641,-17.973,-0.05,-0.993,-0.11],[17.997,-31.383,-53.101,-0.077,-0.991,-0.107],[45.486,-31.693,-50.546,-0.061,-0.991,-0.118],[14.923,-31.307,-48.001,-0.068,-0.992,-0.107],[16.873,-29.398,-20.403,-0.051,-0.993,-0.104],[47.566,-29.815,-29.019,-0.063,-0.992,-0.108],[45.311,-29.207,-25.163,-0.07,-0.992,-0.108]]

    points = [[24.399,18.208,-69.99,0.065,-0.07,-0.995],[62.035,12.139,-71.774,0.072,-0.027,-0.997],[39.638,-9.873,-67.951,0.069,-0.076,-0.995],[55.201,24.424,-73.12,0.08,-0.079,-0.994],[43.593,-8.991,-67.82,0.063,-0.057,-0.996],[47.125,-9.127,-67.938,0.07,-0.087,-0.994],[61.677,5.153,-70.407,0.09,-0.109,-0.99],[54.685,-3.623,-69.439,0.085,-0.071,-0.994],[62.2,10.506,-71.378,0.078,-0.025,-0.997],[32.466,-10.595,-66.422,0.064,-0.082,-0.995],[29.802,21.995,-70.758,0.069,-0.113,-0.991],[61.92,7.391,-70.907,0.085,-0.128,-0.988],[32.16,24.8,-71.531,0.063,-0.074,-0.995],[43.171,27.456,-72.484,0.068,-0.099,-0.993],[21.07,12.538,-68.628,0.067,-0.077,-0.995],[20.503,0.553,-66.75,0.073,-0.088,-0.993],[22.263,-2.934,-66.945,0.073,-0.097,-0.993],[19.978,5.812,-67.76,0.07,-0.094,-0.993]]


#    points = [[-23.451, 39.229, -36.553, 0.054, 0.998, -0.018000000000000002], [-22.299, 39.382, -37.2, 0.053, 0.998, -0.019], [-21.342, 38.912, -38.644, 0.017, 0.999, -0.024], [-19.87, 38.951, -39.376, 0.016, 0.999, -0.025], [-17.782, 39.025, -40.23, 0.009000000000000001, 0.999, -0.028], [-15.342, 39.157, -40.645, 0.001, 1.0, -0.029], [-11.06, 40.037, -3.184, -0.02, 0.999, -0.04], [-14.565, 39.799, -2.9, -0.023, 0.999, -0.039], [-7.75, 40.268, -3.334, -0.022, 0.999, -0.039], [-11.936, 39.411, -40.812, 0.006, 1.0, -0.029], [-19.694, 39.301, -4.011, -0.023, 0.999, -0.04], [-30.614, 37.657, -31.575, -0.008, 0.999, -0.039], [-25.157, 38.744, -7.039, -0.019, 0.999, -0.04], [-8.628, 39.525, -40.124, -0.022, 0.999, -0.04], [-32.665, 37.578, -27.703, -0.004, 1.0, -0.028], [-34.084, 37.337, -24.673000000000002, -0.018000000000000002, 0.999, -0.04], [-28.268, 38.294, -9.537, -0.018000000000000002, 0.999, -0.04], [-31.316, 37.825, -13.466, -0.016, 0.999, -0.04]]
#    points = [[-22.13262341799924, 40.10256199743345, -34.0895510229438, 0.13493010401799868, 0.9907470536353504, -0.011483798595139419], [-20.528098671191195, 39.914509239729796, -35.7387464173146, 0.1080999000592032, 0.9939336588221768, -0.01494740653771828], [-22.69314474093234, 40.56989639444824, -32.1432567468154, 0.15675826227376122, 0.9875011920380479, -0.010479346325888088], [-20.444593592203162, 41.56461648269237, -31.47392251153546, 0.22367769508584448, 0.9746321219929361, -0.0019442567449359874], [-23.508142109302376, 41.086545204871484, -29.539349467073873, 0.18234218766327062, 0.9830472516260267, -0.007156958114401978], [-23.569721675601226, 41.45929846555332, -27.706425916471478, 0.199931009958043, 0.9796203240689705, -0.0052447933035150865], [-21.93302390303147, 41.61889610177532, -30.281732648352218, 0.20484718292447096, 0.9786329262008865, -0.002700687965366747], [-26.252930643529183, 39.97190498168743, -29.765864111751064, 0.11739470038522987, 0.9928072665683287, -0.012989874194686446], [-18.597154178559652, 41.10598895581591, -33.46766083867792, 0.23372552662117083, 0.972245834293036, -0.000960408372669913], [-24.665121586131438, 40.205417840373876, -31.364989891122654, 0.12102862453249479, 0.9922999774673801, -0.012359288490750627], [-24.83144754101345, 39.279958131237116, -34.17555232754592, 0.06981860336610785, 0.9973010915681371, -0.019487426976688132], [-17.433924316452188, 43.16134321481757, -28.803201908252518, 0.3063899364075105, 0.9517924400068138, 0.012731404397632422], [-25.377623224539253, 40.68147287527951, -27.975143185878764, 0.15794293732334025, 0.9872186618943001, -0.009029847190956813], [-21.63155551251615, 42.235862051712445, -27.169303309322878, 0.24457254951070895, 0.9694730714286601, 0.002002144284130482], [-24.390403865315708, 41.35622578522288, -26.059007919726046, 0.19934023417787478, 0.9797509963239548, -0.0029129580628783555], [-22.599863110754693, 42.26102674826029, -25.570255053716163, 0.2362322041803897, 0.9715620707372711, 0.0016232796385275828], [-19.641864226614732, 43.08799965668093, -25.761770922513417, 0.2874667891750411, 0.9576484542017784, 0.0070690639014506906], [-27.44240444320148, 39.846821457476736, -28.15241851042904, 0.1119828540763647, 0.993515377750091, -0.012758897850141579]]

    points = [[53.285282690160855, 58.871106047115504, -28.090253257729376, 0.992966676176644, 0.1009939149402317, 0.04599320155318153], [53.02685065179349, 60.711634128512486, -28.82566083403166, 0.9911848036674772, 0.12093681690774409, 0.038936019190791656], [52.822950249615914, 63.15749743594719, -29.000021423035715, 0.9866654876285239, 0.15208235550073845, 0.04182044419175561], [54.27413224198153, 57.574977964195085, -25.871109582078482, 0.9929614674029262, 0.06358615899395223, 0.09550227129180672], [53.8312419735853, 61.82211236231175, -27.124918235022008, 0.9874736852871288, 0.13749487353977763, 0.06786576134603123], [53.33024206558424, 64.09780264205895, -27.59282154629943, 0.9829598832373241, 0.16408211831123945, 0.0675350187952051], [53.268313186030504, 66.27638329752997, -26.918623859029044, 0.9774147504573961, 0.18753219893998782, 0.08321646778692335], [51.97772097450074, 64.64229471561777, -30.228604589833324, 0.9850422618388216, 0.16679391410192057, 0.0253055838410764], [52.74729690149773, 65.56743571556623, -28.634559831158978, 0.9811720718450707, 0.18058039918796598, 0.05311553006342866], [52.98378678131104, 50.75919114990234, -25.782258224487304, 0.9979137825264384, 0.018581690619357143, 0.06076959242185106], [51.5155051978988, 61.095289058346474, -31.648215319698508, 0.9923110783402697, 0.12253422946064824, -0.012053116334400649], [52.40508329028649, 67.81328958498463, -28.34205352222136, 0.9760766430314248, 0.20427039711992462, 0.06213661796259325], [52.52845936529267, 55.170462979950166, -28.96701624720117, 0.9982079523787111, 0.04967978048894871, 0.019789439073877996], [53.653602420656426, 50.276720907427986, -23.879918220776513, 0.9976565053322691, 0.03300067479923961, 0.05892913465898901], [52.45180380993401, 69.25248830110294, -27.099967140672895, 0.97523423759751, 0.20057004314966465, 0.07766130621106589], [50.597637104797364, 42.44659485321045, -25.490598194885255, 0.9942575903096568, -0.1064352842641565, 0.007512783977333681], [55.0934706181759, 60.96392120395534, -24.953535791925404, 0.9582484439975373, 0.27828500951311064, 0.06450181318753877], [51.767599839940026, 66.77220358820772, -29.924118427130686, 0.9804353566779963, 0.1899871670180682, 0.02961749276702069]]

    n = [0,0,1]
    n = [0.9983170254035888, 0.03953416279367987, 0.042428372129188596]

    fit = FitPoints(points, [20,53,-7,30], n)
    print 'fit', fit
    
    #allpoints = [points1, points2, points3, points4, points5]
    #for points in allpoints:
    #    FitPoints(points, [0, 0, 0, r])
