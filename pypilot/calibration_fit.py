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

import numpy
        
def FitLeastSq(beta0, f, zpoints):
    try:
        import scipy.optimize
    except:
        print "failed to load scientific library, cannot perform calibration update!"
        return False

    leastsq = scipy.optimize.leastsq(f, beta0, zpoints)
    fit = list(leastsq[0])

    if type(zpoints) == type(tuple()):
        res = f(fit, *zpoints)
    else:
        res = f(fit, zpoints)
    total = 0
    for r in res:
        total += r**2
    #print 'res', total/len(res)
    
    return fit

def CalcError(beta, f, points):
    Ri = map(lambda p : f(beta, p)**2, points)
    mean = 0
    for R in Ri:
        mean += R
    return math.sqrt(mean)

def FitPoints(points, sphere_fit):
    if len(points) < 4:
        return False

    zpoints = [[], [], [], [], [], []]
    for i in range(6):
        zpoints[i] = map(lambda x : x[i], points)

    def f_3dfit(beta, x, sphere_fit, down):
        return \
            (x[0] - beta[0]*down[0] - sphere_fit[0])**2 + \
            (x[1] - beta[0]*down[1] - sphere_fit[1])**2 + \
            (x[2] - beta[0]*down[2] - sphere_fit[2])**2 - beta[1]**2

    down = [0, 0, 1]
    new_3d_fit = FitLeastSq([0, sphere_fit[3]], f_3dfit, (zpoints, sphere_fit, down))
    print 'new 3d fit', new_3d_fit
    new_sphere_fit = map(lambda a, b : a + new_3d_fit[0]*b, sphere_fit[:3], down) + [new_3d_fit[1]]
    print 'new sphere fit', new_sphere_fit

    
    # with few sigma points, adjust only bias
    if len(points) < 9: # useful only if geographic location is the same?!/
        def f_sphere_bias3(beta, x, r):
            return (x[0]-beta[0])**2 + (x[1]-beta[1])**2 + (x[2]-beta[2])**2 - r**2

        sphere_bias_fit = FitLeastSq(sphere_fit[:3], f_sphere_bias3, (zpoints, sphere_fit[3]))

        print 'sphere bias fit', sphere_bias_fit
#        sphere_fit = sphere_bias_fit + [sphere_fit[3]]

        ellipsoid_fit = False
    else:
        def f_sphere3(beta, x):
            return (x[0]-beta[0])**2 + (x[1]-beta[1])**2 + (x[2]-beta[2])**2 - beta[3]**2
        sphere_fit = FitLeastSq([0, 0, 0, 30], f_sphere3, zpoints)
        if not sphere_fit:
            print 'FitLeastSq failed!!!! ', len(points), points
            return False
        sphere_fit[3] = abs(sphere_fit[3])
        print 'sphere fit', sphere_fit

        def f_ellipsoid3(beta, x):
            return (x[0]-beta[0])**2 + (beta[4]*(x[1]-beta[1]))**2 + (beta[5]*(x[2]-beta[2]))**2 - beta[3]**2
        ellipsoid_fit = FitLeastSq(sphere_fit + [1, 1], f_ellipsoid3, zpoints)
        print 'ellipsoid_fit', ellipsoid_fit

#        return [sphere_fit, 1, ellipsoid_fit]
        def f_rotellipsoid3(beta, x):
            return x[0]**2 + (beta[1]*x[1] + beta[3]*x[0])**2 + (beta[2]*x[2] + beta[4]*x[0] + beta[5]*x[1])**2 - beta[0]**2
            a = x[0]-beta[0]
            b = x[1]-beta[1]
            c = x[2]-beta[2]
            return (a)**2 + (beta[4]*b + beta[6]*a)**2 + (beta[5]*c + beta[7]*a + beta[8]*b)**2 - beta[3]**2
        cpoints = map(lambda a, b : a - b, zpoints[:3], ellipsoid_fit[:3])
        rotellipsoid_fit = FitLeastSq(ellipsoid_fit[3:] + [0, 0, 0], f_rotellipsoid3, cpoints)
        #print 'rotellipsoid_fit', rotellipsoid_fit

        def f_ellipsoid3_cr(beta, x, cr):
            a = x[0]-beta[0]
            b = x[1]-beta[1]
            c = x[2]-beta[2]
            return (a)**2 + (beta[4]*b + cr[0]*a)**2 + (beta[5]*c + cr[1]*a + cr[2]*b)**2 - beta[3]**2
        ellipsoid_fit2 = FitLeastSq(ellipsoid_fit[:3] + rotellipsoid_fit[:3], f_ellipsoid3_cr, (zpoints, rotellipsoid_fit[3:]))
        #print 'ellipsoid_fit2', ellipsoid_fit2

        cpoints = map(lambda a, b : a - b, zpoints[:3], ellipsoid_fit2[:3])
        rotellipsoid_fit2 = FitLeastSq(ellipsoid_fit[3:] + [0, 0, 0], f_rotellipsoid3, cpoints)
        print 'rotellipsoid_fit2', rotellipsoid_fit2

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

    def f_new_bias3(beta, x):
        #print 'beta', beta
        b = numpy.matrix(map(lambda a, b : a - b, x[:3], beta[:3]))
        m = list(numpy.array(b.transpose()))
        r0 = map(lambda y : beta[3]**2 - vector.dot(y, y), m)
        g = list(numpy.array(numpy.matrix(x[3:]).transpose()))
        r1 = map(lambda y, z : 1200*(beta[4] - vector.dot(y, z)/vector.norm(y)), m, g)
        return r0+r1
        
    new_bias_fit = FitLeastSq(sphere_fit[:4] + [0], f_new_bias3, zpoints)
    print 'new bias fit', new_bias_fit, math.degrees(math.asin(new_bias_fit[4]))
    new_bias_fit = new_bias_fit[:3] + [sphere_fit[3]]

    #import numpy
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
    new_bias_fit = new_bias_fit[:3] + [sphere_fit[3]]

    def dip_error(label, cal):
        ds = []
        dtotal = 0
        for p in points:
            c = cal(p[:3])
            d = vector.dot(c, p[3:6])
            ds += [d]
            dtotal += d

        dtotal /= len(points)

        ddev = 0
        for d in ds:
            ddev += (d - dtotal)**2

        ddev = math.sqrt(ddev / len(ds))

        #print 'dip', label, math.degrees(math.asin(dtotal)), math.degrees(math.asin(ddev))


    def apply_sphere(p):
        c = vector.sub(p, sphere_fit[:3])
        return vector.normalize(c)
    dip_error('sphere', apply_sphere)

    def apply_ellipse(p):
        c = vector.sub(p, ellipsoid_fit[:3])
        c[1] *= ellipsoid_fit[4]
        c[2] *= ellipsoid_fit[5]
        return vector.normalize(c)
    if ellipsoid_fit:
        dip_error('ellipse', apply_ellipse)

    def apply_sphere_rot(p):
        c = vector.sub(p, sphere_fit[:3])
        return rotvecquat(vector.normalize(c), q)

    dip_error('sphere rot', apply_sphere_rot)

    def apply_ellipsoid_rot(p):
        c = vector.sub(p, ellipsoid_fit[:3])
        c[1] *= ellipsoid_fit[4]
        c[2] *= ellipsoid_fit[5]
        return rot(c, rot_fit);
    if ellipsoid_fit:
        dip_error('ellipsoid rot', apply_ellipsoid_rot)

    def apply_sphere_quat(p):
        c = vector.sub(p, sphere_fit[:3])
        #        q = [1 - vector.norm(quat_fit[:3])] + list(quat_fit[:3])
        q = angvec2quat(vector.norm(quat_fit[:3]), quat_fit[:3])
        return rotvecquat(vector.normalize(c), q)
    dip_error('sphere quat', apply_sphere_quat)

    def apply_ellipsoid_rot(p):
        c = vector.sub(p, ellipsoid_fit[:3])
        c[1] *= ellipsoid_fit[4]
        c[2] *= ellipsoid_fit[5]
        q = angvec2quat(vector.norm(quat_fit[:3]), quat_fit[:3])
        return rotvecquat(vector.normalize(c), q)

    if ellipsoid_fit:
        dip_error('ellipsoid quat', apply_ellipsoid_rot)

#    return [sphere_fit, CalcError(sphere_fit, f_sphere3, points)]
    q = angvec2quat(vector.norm(quat_fit[:3]), quat_fit[:3])

    if not ellipsoid_fit:
        ellipsoid_fit = sphere_fit + [1, 1]
    return [sphere_fit, 1, ellipsoid_fit, new_bias_fit, q, new_sphere_fit]

def avg(fac, v0, v1):
    return map(lambda a, b : (1-fac)*a + fac*b, v0, v1)

class SigmaPoint():
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

class SigmaPoints():
    sigma = 1.6 # distance between sigma points
    down_sigma = .05 # distance between down vectors
    max_sigma_points = 64

    def __init__(self):
        self.sigma_points = []

    def AddPoint(self, compass, down):
        for point in self.sigma_points:
            dist = vector.dist(point.compass, compass)
            down_dist = vector.dist(point.down, down)
            #print 'dist', dist, 'down_dist', down_dist
            if dist < SigmaPoints.sigma and down_dist < SigmaPoints.down_sigma:
                point.add_measurement(compass, down)
                return

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
                        mindisti = i
                        mindist = dist

            self.sigma_points[mindisti] = p
        else:
            self.sigma_points.append(p)



def CalibrationProcess(points, fit_output, initial):
    try:
        import os
        os.system("renice 20 %d" % os.getpid())
    except:
        print "warning, failed to renice calibration process"

    cal = SigmaPoints()

    while True:
        time.sleep(120) # wait 2 minutes then run fit algorithm again

        for i in range(points.qsize()):
            p = points.get()
            cal.AddPoint(p[:3], p[3:6])

        # remove points with less than 5 measurements, or older than 1 hour
        p = []
        hour_ago = time.time() - 3600
        for sigma in cal.sigma_points:
            if sigma.count >= 2 and sigma.time > hour_ago:
                p.append(sigma)
        cal.sigma_points = p

        # attempt to perform least squares fit
        p = []
        for sigma in cal.sigma_points:
            p.append(sigma.compass + sigma.down)
        print 'sigma:', len(p)
        fit = FitPoints(p, initial)
        if not fit:
            continue
        
        mag = fit[0][3]
        dev = fit[1]
        if mag < 9 or mag > 70:
            print 'fit found field outside of normal earth field strength', fit
        elif dev > 500:
            print 'deviation too high', dev
        else:
            print 'deviation', dev
            bias = fit[0][:3]
            n = map(lambda a, b: (a-b)**2, bias, initial[:3])
            d = n[0]+n[1]+n[2]
            initial = fit[0]
            print 'd', d

            # if the bias has sufficiently changed
            if d > .1 or True:
                def ang(p):
                    v = rotvecquat(vector.sub(p.compass, bias), vec2vec2quat(p.down, [0, 0, 1]))
                    return math.atan2(v[1], v[0])

                angles = sorted(map(ang, cal.sigma_points))
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

                    #print 'max diff', max_diff

                if max_diff < 2*math.pi*(1 - 1/3.):  # require 1/3 of circle coverage
                    print 'have 1/3'
                    fit_output.put((fit, map(lambda p : p.compass + p.down, cal.sigma_points)))
                else:
                    sphere_fit, d, ellipsoid_fit, new_bias_fit, q, new_sphere_fit = fit
                    fit = new_sphere_fit, d, ellipsoid_fit, new_bias_fit, q, new_sphere_fit
                    print 'use new sphere fit', new_sphere_fit

                    fit_output.put((fit, map(lambda p : p.compass + p.down, cal.sigma_points)))


class MagnetometerAutomaticCalibration():
    def __init__(self, cal_queue, initial):
        self.cal_queue = cal_queue
        self.sphere_fit = initial
        self.points = multiprocessing.Queue()
        self.fit_output = multiprocessing.Queue()
        self.process = multiprocessing.Process(target=CalibrationProcess, args=(self.points, self.fit_output, self.sphere_fit))
        self.process.start()

    def AddPoint(self, point):
        if self.points.qsize() < 256:
            self.points.put(point)
    
    def UpdatedCalibration(self):
        if self.fit_output.empty():
            return False
        while self.fit_output.qsize():
            cal, sigma_points = self.fit_output.get()

        if cal:
            self.cal_queue.put(tuple(cal[0][:3]))
            sphere_fit = cal[0]

        return cal, sigma_points

    def ApplyCalibration(self, point):
        def f_apply_sphere(beta, x):
            return (x-beta[:3])/beta[3]

        n = f_apply_sphere(self.sphere_fit, point)
        return n / vector.norm(n)

    def ApplyHeading(self, accel, mag):
        c = self.ApplyCalibration(mag)

        # rotateout
        alignment = vec2vec2quat(accel, [0, 0, 1])
        r = rotvecquat(c, alignment)
        heading = math.degrees(math.atan2(-r[1], r[0]))
        if heading < 0:
            heading += 360

        return heading

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

    points1 = [[5.05117828453113, 17.051436622454098, -25.318723568202902, -0.14227481163553327, 0.06829921682256473, 0.9874505725022091, 148], [8.732518424198469, 7.006966961340647, -24.30457682360707, -0.1442048912118849, 0.07271291766865261, 0.9868311674912553, 65], [11.263544189812087, 4.074976282196616, -24.028064684918892, -0.14919110669937463, 0.07427166547554676, 0.985974137240136, 26], [19.633246023006098, 1.2551264561409508, -23.32645987304558, -0.15534279480583796, 0.07373219177028445, 0.9850723101816699, 5], [22.91168779213515, 2.304483866516312, -22.847935053403514, -0.14912174558445732, 0.07214640978291842, 0.9861466475438648, 11], [26.415928630259913, 2.160081268079489, -22.113887626981487, -0.15392961373184863, 0.06782141162906284, 0.985733811276944, 110], [35.86023459973186, 19.587277313780955, -22.02848585264407, -0.161510347518978, 0.06509501632021177, 0.9847160601398717, 80], [30.355716969431963, 29.188622199216326, -23.407400439956948, -0.15896023160157302, 0.06359325390491613, 0.9852229370422413, 48], [23.732365597811775, 32.01796736749446, -24.225357074402773, -0.1594501735642048, 0.06857782937525884, 0.9847305015292412, 34], [13.111435547910087, 30.067050163854123, -25.10702998294466, -0.14983899288292415, 0.06616516435687689, 0.9864725083726535, 501], [-9.93304314174144, 4.16310736192217, -60.84709214952585, -0.9962240398019715, 0.08350604630335465, -0.02236148419413661, 23], [-9.918822484742885, 2.9209888873582184, -57.740296613218725, -0.9967410420799044, 0.07754825375658014, -0.02207938732536703, 22], [-9.633938676970697, 1.6945902055471391, -54.76340830007356, -0.9965628605770118, 0.07944690971818039, -0.02335539099024624, 18], [-8.085794599441083, 13.28755849347119, -39.83482131032131, -0.9969091046859793, 0.07564427674887135, -0.017555796749983827, 64], [21.90286094347957, 9.720096398175148, -83.88837601203464, 0.42097594250840625, -0.004232582727554934, -0.9070300719099799, 5], [25.43422492236449, 8.882844007573446, -83.05201159441424, 0.5395010658694754, -0.021128369608106237, -0.8416960214794107, 34], [30.006221842247424, 29.466001465358804, -81.09973596262111, 0.45102059806065553, 0.0005823168793045823, -0.8924930636221058, 15], [33.284771891966955, 31.491977729322212, -79.25074775446885, 0.44202849855303356, 0.002472450536016113, -0.896904938137871, 38], [36.264989613964346, 31.832937060004816, -78.40691567793482, 0.4431111349329784, 0.0034225313975714752, -0.8964486028387313, 25], [40.591620854995526, 30.567128694158427, -76.65877402485776, 0.44368716314080875, 0.002312736266001061, -0.8961725540605476, 113], [43.90646692937339, 28.257611425008196, -75.06811630037629, 0.44254123141565993, 0.0016692524058756029, -0.8966817995022942, 8], [45.650971804112395, 25.79754913583357, -73.80800267852415, 0.4421923158193786, 0.005216969093472346, -0.8968772043931311, 6], [48.0381899661598, 23.785075542445405, -72.9639922755936, 0.44361165168478345, 0.007650654547790376, -0.8961807823036235, 8], [49.54281053860807, 20.912951370950545, -72.3603682632933, 0.4428964445649088, 0.007094015503474294, -0.8965013172153412, 10], [48.84117853965919, 12.591973672394104, -72.5969515290099, 0.43206640662407014, 0.009822363471506775, -0.9017744899445442, 856], [47.54168004020538, 9.343600286246074, -73.22557874067903, 0.4341853235984482, 0.007549965344367311, -0.900751013611358, 12], [45.5750652797432, 6.995613636893353, -73.04390907337806, 0.43256902584711027, 0.006695729089453932, -0.9015494712914408, 12], [44.278850722219104, 4.864309870383154, -74.64296852033608, 0.430987389682572, 0.005336081911647776, -0.9023351992365382, 11], [3.446220961108523, 33.15437633783026, -73.6860555076697, -0.5522570691074112, 0.06452960505250455, -0.8309908355330797, 5], [-4.298163565528005, 33.7547464433339, -65.69775721245591, -0.8381694967933723, 0.0775945402551929, -0.5397977956908677, 5], [19.667380797723442, 32.55457646558946, -24.186735635690077, -0.13985590788978797, 0.07090367071803899, 0.9876277539993797, 35], [11.392731249104894, 26.77614394377033, -26.109844898609335, -0.1619761832719089, 0.06306741360831518, 0.9847732144906292, 391], [8.219839236117494, 24.79251133493356, -25.23647451343768, -0.14759437830521305, 0.06480845105926124, 0.9869109293692189, 12], [6.770180549588927, 21.778193923942002, -25.167999644478225, -0.143599798311144, 0.06720316751059886, 0.9873056544467383, 12], [5.652055361708353, 13.236584458988153, -24.691342514049303, -0.1535032644393711, 0.06738009234791856, 0.9858178694398945, 10], [13.95685494829457, 2.556674552325677, -23.09104198581273, -0.16174817873364583, 0.06951907765286995, 0.9843504463023257, 7], [33.887517698431914, 21.849412225013054, -23.311719003352803, -0.16255295988856558, 0.06384870449798748, 0.984608578858611, 8], [32.24315956002899, 25.071359052541137, -22.597843045756633, -0.1696428090631929, 0.05712764085246054, 0.9838119325346659, 14], [9.23170537830785, -7.963505012063304, -33.70659513770719, -0.11892473248454834, -0.8967307113244821, 0.42620354895474194, 7], [4.595711818178659, -9.875780889917811, -38.741384804170174, -0.1729330036264682, -0.8772083845809082, 0.4475832813086185, 6], [11.209751251813785, -17.644928467385988, -50.85703437247418, -0.13180110807387704, -0.8900647732993301, 0.4361319900518729, 5], [14.32300770101948, -18.319439133314905, -51.71051683843626, -0.11005445061746172, -0.8914037903578699, 0.43950325358952147, 23], [30.981685686952755, -18.308554352821613, -48.55622963835884, -0.13352413925827325, -0.8879975180607769, 0.43996022966975434, 6], [35.99567394906452, -16.488403967241045, -43.58243211847582, -0.12154225125750966, -0.89004554436333, 0.43923809242203726, 5], [36.46979057641855, -15.28552792264907, -40.746538401584914, -0.12329580628068453, -0.889622992299395, 0.43965410652069914, 6], [36.85373940844684, -13.573529222605373, -37.967730233717326, -0.12566963257333497, -0.8903515458472041, 0.4375608025985714, 12], [36.14323185707493, -11.555519723013525, -35.61143838636463, -0.12246142799163325, -0.8898644961401206, 0.4394726904559561, 10], [34.924511129661624, -10.075438732273282, -32.80164318909254, -0.11653210745820004, -0.8903939467720305, 0.43999655939592397, 21]]
        
    allpoints = [points1, points2, points3, points4, points5]
    
    #FitPoints(points, [0, 0, 0, r])
    
    for points in allpoints:
        FitPoints(points, [0, 0, 0, r])


def unused():
    pass
