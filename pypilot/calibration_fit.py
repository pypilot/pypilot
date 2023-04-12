#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import sys, time, math, numpy, scipy.optimize
import vector, resolv, quaternion
import boatimu
resolv = resolv.resolv

from values import *
from client import pypilotClientFromArgs
    
calibration_fit_period = 30  # run every 30 seconds

def lmap(*cargs):
    return list(map(*cargs))

def FitLeastSq(beta0, f, zpoints, debug, dimensions=1):
    try:
        import scipy.optimize
    except Exception as e:
        debug('failed to load scientific library:', e)
        debug('cannot perform calibration update!')
        return False

    leastsq = scipy.optimize.leastsq(f, beta0, zpoints)
    return list(leastsq[0])

def FitLeastSq_odr(beta0, f, zpoints, dimensions=1):
    try:
        import scipy.odr
    except:
        print(_('failed to load scientific library, cannot perform calibration update!'))
        return False
    try:
        Model = scipy.odr.Model(f, implicit=1)
        Data = scipy.odr.RealData(zpoints, dimensions)
        Odr = scipy.odr.ODR(Data, Model, beta0, maxit = 1000)
        output = Odr.run()
        return list(output.beta)
    except:
        print('exception running odr fit!')
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
        zpoints[i] = lmap(lambda x : x[i], points)
        
    data = numpy.array(list(zip(zpoints[0], zpoints[1], zpoints[2])))
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
        q = lmap(lambda o, n : o + t*n, line_fit[0], line_fit[1])
        v = vector.sub(p, q)
        d = vector.dot(v, v)
        max_line_dev = max(d, max_line_dev)
        line_dev += d

        t = vector.dot(p, plane_fit[1]) - vector.dot(plane_fit[0], plane_fit[1])
        v = lmap(lambda b : t*b, plane_fit[1])
        d = vector.dot(v, v)
        max_plane_dev = max(d, max_plane_dev)
        plane_dev += d
        
    line_dev /= len(points)
    plane_dev /= len(points)

    line = [line_fit, line_dev**.5, max_line_dev**.5]
    plane = [plane_fit, plane_dev**.5, max_plane_dev**.5]
    return line, plane

def FitPointsAccel(debug, points):
    zpoints = [[], [], []]
    for i in range(3):
        zpoints[i] = lmap(lambda x : x[i], points)
        
    # determine if we have 0D, 1D, 2D, or 3D set of points
    point_fit, point_dev, point_max_dev = PointFit(points)
    if point_max_dev < .1:
        debug('insufficient data for accel fit %.1f %.1f < 1' % (point_dev, point_max_dev))
        return False

    def f_sphere3(beta, x):
        bias = beta[:3]
        b = numpy.matrix(lmap(lambda a, b : a - b, x[:3], bias))
        m = list(numpy.array(b.transpose()))
        r0 = lmap(lambda y : beta[3] - vector.norm(y), m)
        return r0

    sphere3d_fit = FitLeastSq([0, 0, 0, 1], f_sphere3, zpoints, debug)
    if not sphere3d_fit or sphere3d_fit[3] < 0:
        debug('FitLeastSq sphere failed!!!! ', len(points))
        return False
    debug('sphere3 fit', sphere3d_fit, ComputeDeviation(points, sphere3d_fit))
    return sphere3d_fit

def FitPointsCompass(debug, points, current, norm):
    # ensure current and norm are float
    current = lmap(float, current)
    norm = lmap(float, norm)

    zpoints = [[], [], [], [], [], []]
    for i in range(6):
        zpoints[i] = lmap(lambda x : x[i], points)
        
    # determine if we have 0D, 1D, 2D, or 3D set of points
    point_fit, point_dev, point_max_dev = PointFit(points)
    if point_max_dev < 9:
        debug('0d fit, insufficient data %.1f %.1f < 9' % (point_dev, point_max_dev))
        return False

    line, plane = LinearFit(points)
    line_fit, line_dev, line_max_dev = line
    plane_fit, plane_dev, plane_max_dev = plane

    # initial guess average min and max for bias, and average range for radius
    minc = [1000, 1000, 1000]
    maxc = [-1000, -1000, -1000]
    for p in points:
        minc = lmap(min, p[:3], minc)
        maxc = lmap(max, p[:3], maxc)

    guess = lmap(lambda a, b : (a+b)/2, minc, maxc)
    diff = lmap(lambda a, b : b-a, minc, maxc)
    guess.append((diff[0]+diff[1]+diff[2])/3)
    #debug('initial guess', guess)

    # initial is the closest to guess on the uv plane containing current
    initial = vector.add(current[:3], vector.project(vector.sub(guess[:3], current[:3]), norm))
    initial.append(current[3])
    #debug('initial 1d fit', initial)

    # attempt 'normal' fit along normal vector
    '''
    def f_sphere1(beta, x):
        bias = lmap(lambda x, n: x + beta[0]*n, initial[:3], norm)
        b = numpy.matrix(lmap(lambda a, b : a - b, x[:3], bias))
        m = list(numpy.array(b.transpose()))
        r0 = lmap(lambda y : beta[1] - vector.norm(y), m)
        return r0
    sphere1d_fit = FitLeastSq([0, initial[3]], f_sphere1, zpoints)
    if not sphere1d_fit or sphere1d_fit[1] < 0:
        print('FitLeastSq sphere1d failed!!!! ', len(points))
        return False
    sphere1d_fit = lmap(lambda x, n: x + sphere1d_fit[0]*n, initial[:3], norm) + [sphere1d_fit[1]]
    debug('sphere1 fit', sphere1d_fit, ComputeDeviation(points, sphere1d_fit))
    '''

    def f_new_sphere1(beta, x):
        bias = lmap(lambda x, n: x + beta[0]*n, initial[:3], norm)
        b = numpy.matrix(lmap(lambda a, b : a - b, x[:3], bias))
        m = list(numpy.array(b.transpose()))
        r0 = lmap(lambda y : beta[1] - vector.norm(y), m)
        g = list(numpy.array(numpy.matrix(x[3:]).transpose()))
        fac = 1 # weight deviation
        def dip(y, z):
            n = min(max(vector.dot(y, z)/vector.norm(y), -1), 1)
            return n
        r1 = lmap(lambda y, z : fac*beta[1]*(beta[2]-dip(y, z)), m, g)
        return r0 + r1
    new_sphere1d_fit = FitLeastSq([0, initial[3], 0], f_new_sphere1, zpoints, debug, 2)
    if not new_sphere1d_fit or new_sphere1d_fit[1] < 0 or abs(new_sphere1d_fit[2]) > 1:
        debug('FitLeastSq new_sphere1 failed!!!! ', len(points), new_sphere1d_fit)
        new_sphere1d_fit = current
    else:
        new_sphere1d_fit = lmap(lambda x, a: x + new_sphere1d_fit[0]*a, initial[:3], norm) + [new_sphere1d_fit[1], math.degrees(math.asin(new_sphere1d_fit[2]))]
    new_sphere1d_fit = [new_sphere1d_fit, ComputeDeviation(points, new_sphere1d_fit), 1]
        #print('new sphere1 fit', new_sphere1d_fit)

    if line_max_dev < 2:
        debug('line fit found, insufficient data %.1f %.1f' % (line_dev, line_max_dev))
        return False
    
    # 2d sphere fit across normal vector
    u = vector.cross(norm, [norm[1]-norm[2], norm[2]-norm[0], norm[0]-norm[1]])
    v = vector.cross(norm, u)
    u = vector.normalize(u)
    v = vector.normalize(v)

    # initial is the closest to guess on the uv plane containing current
    initial = vector.add(guess[:3], vector.project(vector.sub(current[:3], guess[:3]), norm))
    initial.append(current[3])
    #debug('initial 2d fit', initial)
    
    '''
    def f_sphere2(beta, x):
        bias = lmap(lambda x, a, b: x + beta[0]*a + beta[1]*b, initial[:3], u, v)
        b = numpy.matrix(lmap(lambda a, b : a - b, x[:3], bias))
        m = list(numpy.array(b.transpose()))
        r0 = lmap(lambda y : beta[2] - vector.norm(y), m)
        return r0
    sphere2d_fit = FitLeastSq([0, 0, initial[3]], f_sphere2, zpoints)
    if not sphere2d_fit or sphere2d_fit[2] < 0:
        print('FitLeastSq sphere2d failed!!!! ', len(points))
        new_sphere2d_fit = initial
    else:
        sphere2d_fit = lmap(lambda x, a, b: x + sphere2d_fit[0]*a + sphere2d_fit[1]*b, initial[:3], u, v) + [sphere2d_fit[2]]
    debug('sphere2 fit', sphere2d_fit, ComputeDeviation(points, sphere2d_fit))
    '''

    def f_new_sphere2(beta, x):
        bias = lmap(lambda x, a, b: x + beta[0]*a + beta[1]*b, initial[:3], u, v)
        b = numpy.matrix(lmap(lambda a, b : a - b, x[:3], bias))
        m = list(numpy.array(b.transpose()))
        r0 = lmap(lambda y : beta[2] - vector.norm(y), m)
        #r0 = lmap(lambda y : 1 - vector.norm(y)/beta[2], m)
        g = list(numpy.array(numpy.matrix(x[3:]).transpose()))
        fac = 1 # weight deviation
        def dip(y, z):
            n = min(max(vector.dot(y, z)/vector.norm(y), -1), 1)
            return n
        r1 = lmap(lambda y, z : fac*beta[2]*(beta[3]-dip(y, z)), m, g)
        return r0 + r1
    new_sphere2d_fit = FitLeastSq([0, 0, initial[3], 0], f_new_sphere2, zpoints, debug, 2)
    if not new_sphere2d_fit or new_sphere2d_fit[2] < 0 or abs(new_sphere2d_fit[3]) >= 1:
        debug('FitLeastSq sphere2 failed!!!! ', len(points), new_sphere2d_fit)
        return False
    new_sphere2d_fit = lmap(lambda x, a, b: x + new_sphere2d_fit[0]*a + new_sphere2d_fit[1]*b, initial[:3], u, v) + [new_sphere2d_fit[2], math.degrees(math.asin(new_sphere2d_fit[3]))]
    new_sphere2d_fit = [new_sphere2d_fit, ComputeDeviation(points, new_sphere2d_fit), 2]

    if plane_max_dev < 1.2:
        plane_fit1 = vector.norm(plane_fit[1])
        ang = math.degrees(math.asin(vector.norm(vector.cross(plane_fit1, norm))))
        
        debug('plane fit found, 2D fit only', ang, plane_fit, plane_dev, plane_max_dev)
        if ang > 30:
            debug('angle of plane not aligned to normal: no 2d fit')
            new_sphere2d_fit = False

        return [new_sphere1d_fit, new_sphere2d_fit, False]

    # ok to use best guess for 3d fit
    initial = guess
    '''
    def f_sphere3(beta, x):
        bias = beta[:3]
        b = numpy.matrix(lmap(lambda a, b : a - b, x[:3], bias))
        m = list(numpy.array(b.transpose()))
        r0 = lmap(lambda y : beta[3] - vector.norm(y), m)
        return r0
    sphere3d_fit = FitLeastSq(initial[:4], f_sphere3, zpoints)
    if not sphere3d_fit or sphere3d_fit[3] < 0:
        print('FitLeastSq sphere failed!!!! ', len(points))
        return False
    debug('sphere3 fit', sphere3d_fit, ComputeDeviation(points, sphere3d_fit))
    '''
    def f_new_sphere3(beta, x):
        b = numpy.matrix(lmap(lambda a, b : a - b, x[:3], beta[:3]))
        m = list(numpy.array(b.transpose()))
        r0 = lmap(lambda y : beta[3] - vector.norm(y), m)
        #r0 = lmap(lambda y : 1 - vector.norm(y)/beta[3], m)
        g = list(numpy.array(numpy.matrix(x[3:]).transpose()))
        fac = 1 # weight deviation
        def dip(y, z):
            n = min(max(vector.dot(y, z)/vector.norm(y), -1), 1)
            return n
        r1 = lmap(lambda y, z : fac*beta[3]*(beta[4]-dip(y, z)), m, g)

        return r0 + r1
    new_sphere3d_fit = FitLeastSq(initial[:4] + [0], f_new_sphere3, zpoints, debug, 2)
    if not new_sphere3d_fit or new_sphere3d_fit[3] < 0 or abs(new_sphere3d_fit[4]) >= 1:
        debug('FitLeastSq sphere3 failed!!!! ', len(points))
        return False
    new_sphere3d_fit[4] = math.degrees(math.asin(new_sphere3d_fit[4]))
    new_sphere3d_fit = [new_sphere3d_fit, ComputeDeviation(points, new_sphere3d_fit), 3]
    #debug('new sphere3 fit', new_sphere3d_fit)
    
    return [new_sphere1d_fit, new_sphere2d_fit, new_sphere3d_fit]


def avg(fac, v0, v1):
    return lmap(lambda a, b : (1-fac)*a + fac*b, v0, v1)

class SigmaPoint(object):
    def __init__(self, sensor, down=False):
        self.sensor = sensor
        self.down = down
        self.count = 1
        self.time = time.monotonic()

    def add_measurement(self, sensor, down):
        self.count += 1
        fac = max(1/self.count, .01)
        self.sensor = avg(fac, self.sensor, sensor)
        if down:
            self.down = avg(fac, self.down, down)
        self.time = time.monotonic()

# store averaged sensore measurements over time for
# calibration curve fitting
class SigmaPoints(object):
    def __init__(self, sigma, max_sigma_points, min_count):
        self.sigma = sigma
        self.max_sigma_points = max_sigma_points
        self.min_count = min_count
        self.Reset()
        self.updated = False
        self.last_sample = False

    def Updated(self):
        if self.updated:
            self.updated = False
            return True
        return False

    # forget all knowledge of stored sensor points
    def Reset(self):
        self.sigma_points = []
        self.lastpoint = False

    def Points(self, down=False):
        def pt(p):
            if down:
                return p.sensor + p.down
            else:
                return p.sensor

        return lmap(pt, self.sigma_points)

    # store a new sensor
    def AddPoint(self, sensor, down=False):
        if not self.lastpoint:
            self.lastpoint = SigmaPoint(sensor, down)
            return

        if self.lastpoint.count < self.min_count: # require x measurements
            if vector.dist2(self.lastpoint.sensor, sensor) < self.sigma:
                self.lastpoint.add_measurement(sensor, down)
                return
            
            self.lastpoint = False
            return

        # use lastpoint as better sample
        sensor, down = self.lastpoint.sensor, self.lastpoint.down
        self.last_sample = sensor, down
        self.lastpoint = False

        ind = 0
        for point in self.sigma_points:
            if point.count > 100:
                continue
            if vector.dist2(point.sensor, sensor) < self.sigma:
                point.add_measurement(sensor, down)
                if ind > 0:
                    # move toward front of list to speed up future tests
                    self.sigma_points = self.sigma_points[:ind-1] + [point] + [self.sigma_points[ind-1]] + self.sigma_points[ind+1:]
                return
            ind += 1

        self.updated = True
        index = len(self.sigma_points)
        p = SigmaPoint(sensor, down)
        if index < self.max_sigma_points:
            # push to front of list
            self.sigma_points = [p] + self.sigma_points
            return


        # replace point that is closest to other points
        mindi = 0
        mind = 1e20
        for i in range(len(self.sigma_points)):
            dt = time.monotonic() - self.sigma_points[i].time
            d = []
            for j in range(len(self.sigma_points)):
                if i == j:
                    continue
                dist = vector.dist(self.sigma_points[i].sensor, self.sigma_points[j].sensor)
                count = min(self.sigma_points[i].count, 100)
                d.append(dist)

            d.sort()
            # weight based on distance to closest 2 points and time
            total = (d[0]+d[1])*1/dt**.2
            if total < mind:
                mindi = i
                mind = total

        self.sigma_points[mindi] = p

    def RemoveOlder(self, dt=3600):
        p = []
        for sigma in self.sigma_points:
            if time.monotonic() - sigma.time < dt:
                p.append(sigma)
        self.sigma_points = p

    def RemoveOldest(self):
        oldest_sigma = self.sigma_points[0]
        for sigma in self.sigma_points:
            if sigma.time < oldest_sigma.time:
                oldest_sigma = sigma

        # don't remove if < 1 minute old
        if time.monotonic() - oldest_sigma.time >= 60:
            self.sigma_points.remove(oldest_sigma)

# calculate how well these datapoints cover the space by
# counting how many 20 degree segments have at least 1 datapoint
def ComputeCoverage(p, bias, norm):
    q = quaternion.vec2vec2quat(norm, [0, 0, 1])
    def ang(p):
        c = quaternion.rotvecquat(vector.sub(p[:3], bias), q)
        d = quaternion.rotvecquat(p[3:6], q)
        v = quaternion.rotvecquat(c, quaternion.vec2vec2quat(d, [0, 0, 1]))
        v = vector.normalize(v)
        return math.degrees(math.atan2(v[1], v[0]))
    #, abs(math.degrees(math.acos(v[2])))

    spacing = 20 # 20 degree segments
    angles = [False] * int(360 / spacing)
    count = 0
    for a in lmap(ang, p):
        i = int(resolv(a, 180) / spacing)
        if not angles[i]:
            angles[i] = True
            count += 1
    return count

def FitAccel(debug, accel_cal):
    p = accel_cal.Points()
    if len(p) < 5:
        return False

    mina = lmap(min, *p)
    maxa = lmap(max, *p)
    diff = vector.sub(maxa[:3], mina[:3])
    if min(*diff) < 1.2:
        debug('need more range', '%.4f' % min(*diff))
        return # require sufficient range on all axes
    if sum(diff) < 4.5:
        debug('need more spread', '%.4f' % sum(diff))
        return # require more spread
    fit = FitPointsAccel(debug, p)
    if not fit:
        debug('FitPointsAccel failed', fit)
        return False

    if abs(1-fit[3]) > .1:
        debug('scale factor out of range', fit)
        return

    dev = ComputeDeviation(p, fit)
    return [fit, dev]

def FitCompass(debug, compass_points, compass_calibration, norm):
    p = compass_points.Points(True)
    if len(p) < 8:
        return

    fit = FitPointsCompass(debug, p, compass_calibration, norm)
    if not fit:
        return
    #debug('FitCompass', fit)

    g_required_dev = .25 # must have more than this to allow 1d or 3d fit
    gpoints = []
    for q in p:
        gpoints.append(q[3:])
    avg, g_dev, g_max_dev = PointFit(gpoints)
    #debug('gdev', g_dev, g_max_dev)
    c = fit[1] # use 2d fit
    if g_max_dev < g_required_dev:
        debug('sigmapoints flat, 2D fit only', g_max_dev, g_required_dev)
    else:
        if fit[2]:
            c = fit[2] # 3d fit
        # for now do not allow 1d fit
        if not c:
            debug('would be using 1d fit!')
            #c = fit[0] # 1d fit only possible

    if not c:
        debug('No Fit available', fit)
        return

    coverage = ComputeCoverage(p, c[0][:3], norm)
    #debug('coverage', coverage)
    if coverage < 14: # require 280 degrees
        debug('insufficient coverage:', coverage, ' need 14')
        if c == fit[1]: # must have had 3d fit to use 1d fit
            return
        c = fit[0]
        return # for now disallow 1d fit
        debug('using 1d fit')

    # make sure the magnitude is sane
    mag = c[0][3]
    if mag < 12 or mag > 120:
        debug('fit found field outside of normal earth field strength', mag)
        return

    # require inclination less than 82 degrees, with so much inclination,
    # the fit is inaccurate (near magnetic pole?)
    inc = c[0][4]
    if abs(inc) > 82:
        debug('incline greater than 82 degrees, no fit',)
        return

    # test points for deviation, all must fall on a sphere
    deviation = c[1]
    if deviation[0] > .15 or deviation[1] > 3:
        curdeviation = ComputeDeviation(p, compass_calibration)
        debug('bad fit:', deviation, 'cur dev:', curdeviation)
        # if compass_calibration calibration is really terrible
        if deviation[0]/curdeviation[0] + deviation[1]/curdeviation[1] < 2.5 or curdeviation[0] > .2 or curdeviation[1] > 10:
            debug('allowing bad fit')
        else:
            compass_points.RemoveOldest()  # remove oldest point if too much deviation
            return # don't use this fit

    # if the bias has not sufficiently changed,
    # the fit didn't change much, so don't bother to report this update
    if vector.dist2(c[0], compass_calibration) < .1:
        debug('new calibration same as previous')
        return

    c[1].append(coverage)
    return c

class CalibrationProperty(RoundedValue):
    def __init__(self, name, default):
        self.default = default
        self.client_can_set = True
        super(CalibrationProperty, self).__init__(name+'.calibration', default, persistent=True)

    def set(self, value):
        if not value:
            value = self.default
        try:
            if self.value and self.locked.value:
                return
            self.age.reset()
        except:
            pass # startup before locked is initiated
        super(CalibrationProperty, self).set(value)

class AgeValue(StringValue):
    def __init__(self, name, **kwargs):
        super(AgeValue, self).__init__(name, 0, **kwargs)
        self.lastreadable = 0

    def reset(self):
        self.set(time.monotonic())

    def update(self):
        t = time.monotonic()
        if t - self.lastreadable > 1:
            self.lastreadable = t
            self.set(self.value)

    def get_msg(self):
        t = time.monotonic()
        if self.value:
            return '"' + boatimu.readable_timespan(t - self.value) + '"'
        return '"N/A"'

def RegisterCalibration(client, name, default):
    calibration = client.register(CalibrationProperty(name, default))
    calibration.age = client.register(AgeValue(name+'.calibration.age'))
    calibration.locked = client.register(BooleanProperty(name+'.calibration.locked', False, persistent=True))
    calibration.sigmapoints = client.register(RoundedValue(name+'.calibration.sigmapoints', False))
    calibration.points = client.register(RoundedValue(name+'.calibration.points', False, persistent=True))
    calibration.log = client.register(Property(name+'.calibration.log', ''))
    return calibration
        
def CalibrationProcess(cal_pipe, client):
    accel_points = SigmaPoints(.05**2, 12, 10)
    compass_points = SigmaPoints(1.1**2, 28, 3)

    norm = [0, 0, 1]

    accel_calibration = RegisterCalibration(client, 'imu.accel', [[0, 0, 0, 1], 1])
    compass_calibration = RegisterCalibration(client, 'imu.compass', [[0, 0, 0, 30, 0], [1, 1], 0])
    compass_calibration.field_strength = client.register(SensorValue('imu.compass.calibration.field_strength'))
    compass_calibration.inclination = client.register(SensorValue('imu.compass.calibration.inclination'))
    
    client.watch('imu.alignmentQ')
    if not cal_pipe: # get these through client rather than direct pipe
        client.watch('imu.accel')
        client.watch('imu.compass')
        client.watch('imu.fusionQPose')

    def debug(name):
        def debug_by_name(*args):
            s = ''
            for a in args:
                try:
                    s = '%.5f' % value
                except Exception as e:
                    pass
                s += str(a) + ' '
                #print("debug", name, s)
            client.set('imu.'+name+'.calibration.log', s)
        return debug_by_name

    last_compass_coverage = 0

    warnings = {}
    def warnings_update(sensor, warning, value):
        if value:
            if sensor in warnings and warnings[sensor] == warning:
                return
            warnings[sensor] = warning
        else:
            if not sensor in warnings:
                return
            del warnings[sensor]

        str_warnings = ''
        for sensor, warning in warnings.items():
            str_warnings += sensor + ' ' + warning
        cal_pipe.send(str_warnings)
    
    while True:
        t = time.monotonic()
        addedpoint = False
        down = False
        while time.monotonic() - t < calibration_fit_period:
            # receive pypilot messages
            msg = client.receive(1)
            for name in msg:
                value = msg[name]
                if name == 'imu.alignmentQ' and value:
                    value = quaternion.normalize(value)
                    norm = quaternion.rotvecquat([0, 0, 1], value)
                    compass_points.Reset()
                elif name == 'imu.accel':
                    if value:
                        accel_points.AddPoint(value)
                    addedpoint = True
                elif name == 'imu.compass' and down:
                    if value and down:
                         compass_points.AddPoint(value, down)                            
                    addedpoint = True
                elif name == 'imu.fusionQPose':
                    if value:
                        value = quaternion.normalize(value)
                        down = quaternion.rotvecquat([0, 0, 1], quaternion.conjugate(value))

            # receive calibration data
            if cal_pipe:
                p = cal_pipe.recv()
                while p:
                    if 'accel' in p:
                        accel_points.AddPoint(p['accel'])
                        addedpoint = True
                    if 'compass' in p:
                        fusionQPose = quaternion.normalize(p['fusionQPose'])
                        down = quaternion.rotvecquat([0, 0, 1], quaternion.conjugate(fusionQPose))
                        compass_points.AddPoint(p['compass'], down)
                        addedpoint = True
                    p = cal_pipe.recv()

            if accel_points.last_sample:
                sensor, down = accel_points.last_sample
                accel_points.last_sample = False
                cal = accel_calibration.value
                # apply calibration
                value = vector.sub(sensor, cal[0][:3])
                g = vector.norm(value)
                # check that calibration points are near magnitude of 1
                warnings_update('accel', 'warning', abs(g-1) > .1)

            if compass_points.last_sample:
                sensor, down = compass_points.last_sample
                accel_points.last_sample = False
                cal = compass_calibration.value
                # apply calibration
                value = vector.sub(sensor, cal[0][:3])
                # check that rate of change of compass magnitude and inclination is not too high
                warn = False
                gauss = vector.norm(value)
                d = .0001
                if compass_calibration.field_strength.value is False:
                    d = 1
                field_strength = compass_calibration.field_strength.value * (1-d) + gauss * d
                compass_calibration.field_strength.set(field_strength)
                if abs(field_strength - gauss) > 3:
                    warn = True

                d = .001
                if compass_calibration.inclination.value is False:
                    d = 1

                c = vector.dot(value, down) / vector.norm(value)
                c = min(max(-1, c), 1)
                angle = math.degrees(math.asin(c))
                inclination = compass_calibration.inclination.value * (1-d) + angle * d
                compass_calibration.inclination.set(inclination)

                if abs(inclination - angle) > 8:
                    warn = True

                warnings_update('compass', 'distortions', warn)
                #if warn:
                #print('mag distortions debug', field_strength, gauss, inclination, angle)


            cals = [(accel_calibration, accel_points), (compass_calibration, compass_points)]
            for calibration, points in cals:
                calibration.age.update()
                if points.Updated():                    
                    calibration.sigmapoints.set(points.Points())

        if not addedpoint: # don't bother to run fit if no new data
            continue

        accel_points.RemoveOlder(10*60) # 10 minutes
        fit = FitAccel(debug('accel'), accel_points)
        if fit: # reset compass sigmapoints on accel cal
            dist = vector.dist(fit[0][:3], accel_calibration.value[0][:3])
            if dist > .01: # only update when bias changes more than this
                if dist > .08: # reset compass cal from large change in accel bias
                    compass_points.Reset()
                    debug('accel')('reset compass from large accel bias')
                accel_calibration.set(fit)
                accel_calibration.points.set(accel_points.Points())
            else:
                debug('accel')('calibration distance too small ', dist)

        compass_points.RemoveOlder(20*60) # 20 minutes
        fit = FitCompass(debug('compass'), compass_points, compass_calibration.value[0], norm)
        if fit:
            # ignore decreasing compass coverage
            new_coverage = fit[1][2]
            if new_coverage < last_compass_coverage:
                debug('compass')('ignoring decreasing coverage')
            else:
                compass_calibration.set(fit)
                compass_calibration.points.set(compass_points.Points())

                compass_calibration.field_strength.set(False)
                compass_calibration.inclination.set(False)
        else:
            last_compass_coverage = 0 # reset

def ExtraFit():
    ellipsoid_fit = False
    '''
    if len(points) >= 10:
        def f_ellipsoid3(beta, x):
            return (x[0]-beta[0])**2 + (beta[4]*(x[1]-beta[1]))**2 + (beta[5]*(x[2]-beta[2]))**2 - beta[3]**2
        ellipsoid_fit = FitLeastSq(sphere_fit + [1, 1], f_ellipsoid3, zpoints)
        print('ellipsoid_fit', ellipsoid_fit)
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
        cpoints = lmap(lambda a, b : a - b, zpoints[:3], ellipsoid_fit[:3])
        rotellipsoid_fit = FitLeastSq(ellipsoid_fit[3:] + [0, 0, 0], f_rotellipsoid3, cpoints)
        #print('rotellipsoid_fit', rotellipsoid_fit)
        ellipsoid_fit2 = FitLeastSq(ellipsoid_fit[:3] + rotellipsoid_fit[:3], f_ellipsoid3_cr, (zpoints, rotellipsoid_fit[3:]))
        #print('ellipsoid_fit2', ellipsoid_fit2)

        cpoints = lmap(lambda a, b : a - b, zpoints[:3], ellipsoid_fit2[:3])
        rotellipsoid_fit2 = FitLeastSq(ellipsoid_fit[3:] + [0, 0, 0], f_rotellipsoid3, cpoints)
        print('rotellipsoid_fit2', rotellipsoid_fit2)
    else:
        ellipsoid_fit = False

    def f_uppermatrixfit(beta, x):
            b = numpy.matrix(lmap(lambda a, b : a - b, x[:3], beta[:3]))
            r = numpy.matrix([beta[3:6], [0]+list(beta[6:8]), [0, 0]+[beta[8]]])
            print('b', beta)

            m = r * b
            m = list(numpy.array(m.transpose()))
            r0 = lmap(lambda y : 1 - vector.dot(y, y), m)

            return r0

    def f_matrixfit(beta, x, efit):
            b = numpy.matrix(lmap(lambda a, b : a - b, x[:3], efit[:3]))
            r = numpy.matrix([[1,       beta[0], beta[1]],
                              [beta[2], efit[4], beta[3]],
                              [beta[4], beta[5], efit[5]]])

            m = r * b
            m = list(numpy.array(m.transpose()))
            r0 = lmap(lambda y : efit[3]**2 - vector.dot(y, y), m)
            #return r0

            g = list(numpy.array(numpy.matrix(x[3:]).transpose()))
            r1 = lmap(lambda y, z : beta[6] - vector.dot(y, z), m, g)

            return r0+r1

    def f_matrix2fit(beta, x, efit):
            b = numpy.matrix(lmap(lambda a, b : a - b, x[:3], beta[:3]))
            r = numpy.matrix([[1,       efit[0], efit[1]],
                              [efit[2], beta[4], efit[3]],
                              [efit[4], efit[5], beta[5]]])

            m = r * b
            m = list(numpy.array(m.transpose()))
            r0 = lmap(lambda y : beta[3]**2 - vector.dot(y, y), m)
            #return r0

            g = list(numpy.array(numpy.matrix(x[3:]).transpose()))
            r1 = lmap(lambda y, z : beta[6] - vector.dot(y, z), m, g)

            return r0+r1

    if False:
         matrix_fit = FitLeastSq([0, 0, 0, 0, 0, 0, 0], f_matrixfit, (zpoints, ellipsoid_fit))
         #print('matrix_fit', matrix_fit)

         matrix2_fit = FitLeastSq(ellipsoid_fit + [matrix_fit[6]], f_matrix2fit, (zpoints, matrix_fit))
         #print('matrix2_fit', matrix2_fit)

         matrix_fit2 = FitLeastSq(matrix_fit, f_matrixfit, (zpoints, matrix2_fit))
         print('matrix_fit2', matrix_fit2)

         matrix2_fit2 = FitLeastSq(matrix2_fit[:6] + [matrix_fit2[6]], f_matrix2fit, (zpoints, matrix_fit2))
         print('matrix2_fit2', matrix2_fit2)

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
        m = lmap(lambda v : rotvecquat(vector.normalize(v), q), list(zip(n[0], n[1], n[2])))
#        m = lmap(lambda v : rot(v, beta), list(zip(n[0], n[1], n[2])))

        m = numpy.array(list(zip(*m)))
        d = m[0]*x[3] + m[1]*x[4] + m[2]*x[5]
        return beta[3] - d

    quat_fit = FitLeastSq([0, 0, 0, 0], f_quat, (zpoints, sphere_fit))
    #    q = [1 - vector.norm(quat_fit[:3])] + list(quat_fit[:3])
    q = angvec2quat(vector.norm(quat_fit[:3]), quat_fit[:3])

    print('quat fit', q, math.degrees(angle(q)), math.degrees(math.asin(quat_fit[3])))
    
    def f_rot(beta, x, sphere_fit):
        sphere_fit = numpy.array(sphere_fit)
        n = [x[0]-sphere_fit[0], x[1]-sphere_fit[1], x[2]-sphere_fit[2]]
        m = lmap(lambda v : rot(v, beta), zip(n[0], n[1], n[2]))
        m = numpy.array(list(zip(*m)))

        d = m[0]*x[3] + m[1]*x[4] + m[2]*x[5]
        return beta[3] - d

    rot_fit = FitLeastSq([0, 0, 0, 0], f_rot, (zpoints, sphere_fit))
    print('rot fit', rot_fit, math.degrees(rot_fit[0]), math.degrees(rot_fit[1]), math.degrees(rot_fit[2]), math.degrees(math.asin(min(1, max(-1, rot_fit[3])))))
    

def main():
    print('running remote calibration')
    client = pypilotClientFromArgs(sys.argv)
    CalibrationProcess(False, client)

if __name__ == '__main__':
    main()
