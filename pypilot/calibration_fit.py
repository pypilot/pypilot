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

    # with few sigma points, adjust only bias
    def f_sphere_bias3(beta, x, r):
        return ((x[0]-beta[0])**2 + (x[1]-beta[1])**2 + (x[2]-beta[2])**2)/r**2 - 1

    sphere_bias_fit = FitLeastSq(sphere_fit[:3], f_sphere_bias3, (zpoints, sphere_fit[3]))
    if not sphere_bias_fit:
        print 'sphere bias failed!!! ', len(points), points
        return False

    print 'sphere bias fit', sphere_bias_fit
    sphere_fit = sphere_bias_fit + [sphere_fit[3]]

    ellipsoid_fit = False
    if len(points) >= 9:
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
        ellipsoid_fit = FitLeastSq(sphere_fit + [1, 1], f_ellipsoid3, zpoints)
        print 'ellipsoid_fit', ellipsoid_fit

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
    return [sphere_fit, 1, ellipsoid_fit, new_bias_fit, q]

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
    import os
    if os.system('sudo chrt -pi 0 %d 2> /dev/null > /dev/null' % os.getpid()):
      print 'warning, failed to make calibration process idle, trying renice'
      if os.system("renice 20 %d" % os.getpid()):
          print 'warning, failed to renice calibration process'

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
                    #sphere_fit, d, ellipsoid_fit, new_bias_fit, q = fit
                    #fit = new_sphere_fit, d, ellipsoid_fit, new_bias_fit, q
                    fit_output.put((fit, map(lambda p : p.compass + p.down, cal.sigma_points)))
                    pass


class MagnetometerAutomaticCalibration():
    def __init__(self, cal_pipe, initial):
        self.cal_pipe = cal_pipe
        self.sphere_fit = initial
        self.points = multiprocessing.Queue()
        self.fit_output = multiprocessing.Queue()
        self.process = multiprocessing.Process(target=CalibrationProcess, args=(self.points, self.fit_output, self.sphere_fit))
        self.process.start()

    def __del__(self):
        self.process.terminate()

    def AddPoint(self, point):
        if self.points.qsize() < 256:
            self.points.put(point)
    
    def UpdatedCalibration(self):
        if self.fit_output.empty():
            return False
        while self.fit_output.qsize():
            cal, sigma_points = self.fit_output.get()

        if cal:
            self.cal_pipe.send(tuple(cal[0][:3]))
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


    points = [[46.24826930749252, -32.00949248803704, -1.7190904873778483, 0.007612359198180472, 0.07375274000171238, 0.997121201536174], [47.41547773181635, -26.831039065870158, -1.6451752978389955, 0.009700708786381778, 0.05989273022844886, 0.9980137821579036], [42.00206323319404, -29.049435146153087, -1.873565459854781, 0.0012222290711491899, 0.07401232348188994, 0.9970619728356747], [39.86194990948929, -31.460158746732695, -1.7361118072844706, 0.0040503662655241406, 0.07664346299746724, 0.9969038509544844], [49.10410361714734, -29.88697049707063, -1.8456860448301928, 0.015542309695007223, 0.07025601421988717, 0.9972417687821991], [37.75969914738995, -33.30745700471077, -1.6697122742075006, 0.003915537167343369, 0.0781633143812972, 0.9968500198558474], [50.83446342646086, -23.712075727313078, -1.6999577188510462, 0.01008970996097344, 0.0676431957878635, 0.9974246625072722], [53.97771443303051, -19.713079599778496, -1.9624329138072458, 0.023255025026208055, 0.07333717827898693, 0.9969272462236617], [51.598230552911176, -27.75940894421805, -1.9292928065547406, 0.015449015889895754, 0.07086222504904838, 0.9972563873097728], [55.13117073659507, -21.355809790541265, -1.1655952113179562, 0.011250994342576636, 0.0775211242586796, 0.9967060443024054], [53.86527893537522, -22.362283513639056, -2.5199573842074976, 0.03453913655284192, 0.07150658709232106, 0.9967637506384438], [52.95930806832864, -22.33057519965794, 0.5793389166085656, 0.00019420219283519354, 0.06458870942027359, 0.9978839689384776], [52.57825681613688, -25.710404410483047, -1.5078382782410376, 0.014460991346225957, 0.07612070189628221, 0.9968406823120567], [32.03863482797772, -30.86024441496319, -2.7744288797126173, -0.005918353588396505, 0.07229970571286373, 0.9973503794143072], [34.24046157181815, -29.720108754500213, -2.5499616105288463, 0.0004447672782495059, 0.07547434802576501, 0.997060384041991], [41.7942083288337, -33.60639691436594, -1.8466340042688085, 0.007868659661928382, 0.07327149595624272, 0.9971875637249342], [45.49025964949454, -33.8188935786706, -2.148468669073711, 0.007689609298668305, 0.07231665747035312, 0.9972905470011133], [49.18853291373969, -32.943287374866706, -1.526963193526442, 0.016647093583371488, 0.08615418459892077, 0.9960204852510831], [36.571670585039584, -34.892402146529264, -1.2194985045363422, 0.0006862505584586333, 0.0760446243849144, 0.9970268388414236], [47.2136605377609, -33.189883363468276, -0.10528173587814844, 0.02838537632594183, 0.09366500501161518, 0.9951346046335138], [45.161876850333215, -28.150875451917347, -1.9632916020752587, 0.012315841823715458, 0.07130523777124076, 0.9972347195621203], [49.18425446017889, -27.58031459186831, -1.335109993918163, 0.009952917817656034, 0.07887476414205613, 0.9966308278412307], [49.04773970627044, -25.380129578067862, -2.0287181668418777, 0.013529460076681167, 0.06894702446402483, 0.9973333866087587], [54.15058578325548, -27.98098204591116, -2.3136284678786225, 0.04392344310444213, 0.06562995137508461, 0.9967478044161988], [55.28673682747873, -15.623290067831423, -0.32155035768743156, 0.0040812007813356395, 0.07292611976633787, 0.9971243204594616], [54.86929240182823, -16.66832476851648, -2.0519854206500288, 0.03022258913272961, 0.08034737960622365, 0.996287543010749], [39.265972025328686, -34.26819071532968, -1.0681490486702745, 0.014404084702663677, 0.07825579731939845, 0.9968102145669935], [39.16127292389467, -29.44978976145934, -1.8979894448954042, 0.007759595152598594, 0.07619617140812886, 0.9969501232307846], [42.93601116931139, -28.426596210370608, -0.05001233420805637, 0.008018102843070422, 0.08485271722467763, 0.996258686158682], [50.85785753740108, -25.626988001444246, -3.1561360128419977, 0.014435159496755286, 0.04288766754929751, 0.9989609537965113], [52.462998768202226, -21.219355281357885, -1.4830964335589083, 0.014607782950663162, 0.07304683543356755, 0.9970692125604266], [46.856576218545825, -28.773454428289632, -2.789317606196338, 0.01719218211089816, 0.06653266735426758, 0.997563499928525], [43.36746730248659, -33.51040828751695, 0.21804526172975497, 0.012087314289996202, 0.10139349161051768, 0.9947380079119807], [41.89443413523108, -31.74102603549178, -0.4876700508593291, 0.0011240749579037414, 0.0854988739097377, 0.9963100439333991], [56.64733096493326, -13.13473174842994, -1.9069755115394935, 0.03648034657066007, 0.06402814971604351, 0.9972185987762499], [46.52930136435826, -25.983891388782975, -0.3585305532364356, -0.006077698137516237, 0.09255923271090909, 0.9955420185069006], [32.30382980362104, -28.21985740886658, -2.432021468861085, 0.009134404074646, 0.07960758729624173, 0.9967727143287418], [46.57030815937252, -23.176608483961925, -1.4047566619174405, 0.00928804619913782, 0.08250906437432991, 0.996535283007413], [49.43514759162836, -29.635609415086613, -0.04759042156149769, -0.013367536087726253, 0.07602058468775474, 0.9969921516563068], [56.66323102587774, -13.076089334466838, 0.5593608270906426, -0.0058221791883423385, 0.08997474006374172, 0.9958425677070457], [55.934414081573486, -19.487016010284425, -0.0035039064288139342, 0.04176045742897003, 0.08613634072899239, 0.9954052390373197], [43.96760860478295, -24.940494199427306, -1.0649787177343488, 0.007516652038359034, 0.10342307564318841, 0.9945903644809364], [49.01569765176248, -25.945374827922553, -0.012290398468386365, -0.010474119421432093, 0.08089464937812686, 0.9966420982631896], [44.211641803870265, -30.578235574808126, -2.0036655369966483, 0.003774383707312021, 0.069048570495209, 0.9974882802969623], [50.85207490843731, -20.27604443507815, -2.1055722638772747, -0.007681417253482115, 0.07700868566844642, 0.9969755542894254], [43.06835452354269, -27.012650804329972, -1.6348632856674639, -0.02321995630252213, 0.07717386823635611, 0.9967062059866417], [47.5425647922196, -28.496996905759463, -0.7291030266354989, 0.0042855102913887015, 0.08217096074233081, 0.996576474705445], [35.241915950775144, -33.92090854644775, -2.7996212357282637, -0.01057341632131431, 0.0419366184374863, 0.9990574519828787], [50.91217320247847, -22.07029261047727, -0.42633954511074, 0.008787729628409267, 0.0791727554085091, 0.9966989875873705], [38.47726542887954, -28.20911443567174, -0.41651916215779117, 0.003962582479425479, 0.08614431555694783, 0.9962637670983256], [34.892696534971606, -33.22415795763344, -1.0445113312847059, 0.0029497805555794513, 0.09949618875584439, 0.9950236513048689], [38.84749215591477, -35.346133239646285, -2.7428954811089663, 0.011260624141989898, 0.05510420472698978, 0.9983966053189867], [53.03187334594726, -24.1685210989275, -3.139535111057758, 0.020346757523059453, 0.04814708930792133, 0.9986289380412436], [32.677224870491024, -29.607704258060455, -0.7146917936921121, 0.00785649772331594, 0.08124560813697139, 0.9966569857528075], [44.72636292933872, -26.35169208545773, -2.4629013127559882, 0.011536985806340534, 0.08465100091006456, 0.9963347248573067], [35.612354223754885, -31.02636912053013, -2.8031947521699068, -0.01987148963127438, 0.05436861075212174, 0.9983129513692441], [43.21971176219496, -28.33854990897235, -3.0881760523180577, 0.014768975500953766, 0.06319496058848983, 0.9978311445283468], [37.396389896578505, -30.6513866351224, -1.7625332370713405, 0.00249034268078889, 0.07572592523297599, 0.9971240216001279], [36.28941478248285, -28.899486002750773, -2.3028244101699427, 0.007765223971388632, 0.08383740850471712, 0.996444240058924], [56.6646820690155, -16.235077510976794, -1.4049963992357255, 0.0302822326867212, 0.089701608767523, 0.9955041159205664], [54.04580031621554, -17.08916463559185, 0.42817237454922036, -0.02503762138675145, 0.06983788238697973, 0.9972310814589125], [35.23582305631637, -28.893928120050433, -0.7163918894857565, 0.013956471157136687, 0.09400523587210019, 0.9954711274936455], [49.034505847616195, -23.819531140191355, -0.6812284367180765, -0.010455556814131876, 0.09341395545202856, 0.9955610917309312]]
    
    
    FitPoints(points, [0, 0, 0, r])
    
    #allpoints = [points1, points2, points3, points4, points5]
    #for points in allpoints:
    #    FitPoints(points, [0, 0, 0, r])
