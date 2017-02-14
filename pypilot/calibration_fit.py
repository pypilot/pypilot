#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
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
import quaternion
import multiprocessing

def FitLeastSq(beta0, f, zpoints):
    try:
        import scipy.optimize
    except:
        print "failed to load scientific library, cannot perform calibration update!"
        return False

    leastsq = scipy.optimize.leastsq(f, beta0, zpoints)
    return list(leastsq[0])

def CalcError(beta, f, points):
    Ri = map(lambda p : f(beta, p)**2, points)
    mean = 0
    for R in Ri:
        mean += R
    return math.sqrt(mean)

def FitPoints(points, sphere_fit):
    if len(points) < 7:
        return False

    zpoints = [[], [], [], [], [], []]
    for i in range(6):
        zpoints[i] = map(lambda x : x[i], points)

    def f_sphere3(beta, x):
        return (x[0]-beta[0])**2 + (x[1]-beta[1])**2 + (x[2]-beta[2])**2 - beta[3]**2

    print 'fit points with', sphere_fit
    
    # with few sigma points, adjust only bias
    if len(points) < 10: # useful only if geographic location is the same?
        def f_sphere_bias3(beta, x, r):
            return (x[0]-beta[0])**2 + (x[1]-beta[1])**2 + (x[2]-beta[2])**2 - r**2

        sphere_bias_fit = FitLeastSq(sphere_fit[:3], f_sphere_bias3, (zpoints, sphere_fit[3]))
        if not sphere_bias_fit:
            return False

        sphere_fit = sphere_bias_fit + [sphere_fit[3]]
        print 'sphere bias fit', sphere_bias_fit

        def f_new_bias3(beta, x):
#            print 'beta', beta
#            print 'x', x
            n = [x[0]-beta[0], x[1]-beta[1], x[2]-beta[2]]
            d = n[0]*x[3] + n[1]*x[4] + n[2]*x[5]
            m = map(vector.norm, zip(n[0], n[1], n[2]))
            
            d = d / m
            return d  - beta[3]
        new_bias_fit = FitLeastSq([0, 0, 0, 0], f_new_bias3, zpoints)
        print 'new bias fit', new_bias_fit

        def f_new_combined_bias3(beta, x, r):
            return [f_sphere_bias3(beta, x, r), f_new_bias3(beta, x)]

#        new_combined_bias_fit = FitLeastSq([0, 0, 0, 0], f_new_combined_bias3, (zpoints, 30))
 #       print 'new combined bias fit', new_combined_bias_fit

        
    else:
        sphere_fit = FitLeastSq([0, 0, 0, 30], f_sphere3, zpoints)
        if not sphere_fit:
            print 'FitLeastSq failed!!!! ', len(points), points
            return False
        sphere_fit[3] = abs(sphere_fit[3])
        print 'sphere fit', sphere_fit

    return [sphere_fit, CalcError(sphere_fit, f_sphere3, points)]

class SigmaPoints():
    sigma = 4.5 # distance between sigma points
    max_sigma_points = 16

    def __init__(self):
        self.sigma_points = []
        self.lastmindisti = 0

    def AddPoint(self, point):
        for i in range(len(self.sigma_points)):
            dist = vector.norm(map(lambda x, y: x - y, self.sigma_points[i][:3], point[:3]))
            if dist < SigmaPoints.sigma:
                #fac = 1/self.sigma_points[i][3]
                fac = .01
                self.sigma_points[i] = map(lambda a, b : (1-fac)*a + fac*b, self.sigma_points[i][:6], point) + [self.sigma_points[i][6] + 1]
                return

        index = len(self.sigma_points)
        p = list(point) + [1]
        if index == SigmaPoints.max_sigma_points:
            if True:
                self.lastmindisti += 1
                if self.lastmindisti == SigmaPoints.max_sigma_points:
                    self.lastmindisti = 0
                mindisti = self.lastmindisti
            else:
                # replace point that is closest to other points
                mindisti = 0
                mindist = 1e20
                for i in range(len(self.sigma_points)):
                    dist = 0
                    for p in self.sigma_points:
                        dist += vector.norm(map(lambda x, y : x - y, p[:3], self.sigma_points[i][:3]))

                    if dist < mindist:
                        mindisti = i
                        mindist = dist

                        #            print "mindist", mindisti
            self.sigma_points[mindisti] = p
        else:
            self.sigma_points.append(p)


def CalibrationProcess(points, fit_output, initial):
    cal = SigmaPoints()

    while True:
        for i in range(points.qsize()):
            cal.AddPoint(points.get())

        p = []
        for sigma in cal.sigma_points:
            print sigma

            if sigma[6] > 20:
                p.append(sigma)

        print 'sigma:', len(p)

        fit = FitPoints(p, initial)

        if fit:
            mag = fit[0][3]
            dev = fit[1]
            if mag < 15 or mag > 70:
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
                if True:
                    if d > .1:
                        fit_output.put((fit, cal.sigma_points))
                        last_bias = bias
                else:
                    fit_output.put((fit, cal.sigma_points))

            time.sleep(120) # wait 2 minutes then run fit algorithm again
        else:
            time.sleep(15) # wait 15 seconds then run fit algorithm again

def CalibrationProcessExceptions(*args):
  try:
    CalibrationProcess(*args)
  except KeyboardInterrupt:
    print 'Keyboard interrupt, calibration fit process exit'
    pass

class MagnetometerAutomaticCalibration():
    def __init__(self, cal_queue, initial):
        self.cal_queue = cal_queue
        self.sphere_fit = initial
        self.points = multiprocessing.Queue()
        self.fit_output = multiprocessing.Queue()
        self.process = multiprocessing.Process(target=CalibrationProcessExceptions, args=(self.points, self.fit_output, self.sphere_fit))
        self.process.start()

    def AddPoint(self, point):
        if self.points.qsize() < 64:
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
        alignment = quaternion.vec2vec2quat(accel, [0, 0, 1])
        r = quaternion.rotvecquat(c, alignment)
        heading = math.degrees(math.atan2(-r[1], r[0]))
        if heading < 0:
            heading += 360

        return heading

if __name__ == '__main__':
    import signalk
    client = False
    if len(sys.argv) == 1:
        sys.argv.append('-')

    compass_calibration = MagnetometerAutomaticCalibration()
    for arg in sys.argv[1:]:
        if arg != '-' and not client:
            try:
                print "connecting to", arg
                client = signalk.client.SignalKClient(arg)
                continue
            except:
                print "Failed to connect:", arg

        if arg == '-':
            print "reading input file from stdin"
            f = sys.stdin
        else:
            f = open(arg, 'r')

        for line in f.readlines():
            try:
                data = json.loads(line.rstrip())
                value = data['value']
                compass_calibration.AddPoint(value)
            except:
                print "invalid input line", line

    compass_calibration.UpdateCalibration()
    if client:
        cnt = 0
        last_accel = [0, 0, 0]
        heading_off = 0
        lp_heading = 0
        while True:
            result = client.receive(0)
            if not result:
                time.sleep(.01)
                continue

            if result['name'] == 'fusionQPose':
                fusionQPose = result['value']

            if result['name'] == 'compass':
                compass_calibration.AddPoint(result['value'])
                heading = compass_calibration.ApplyHeading(last_accel, result['value'])
                
                cnt+=1
                if cnt == 50:
                    cnt = 0
                    compass_calibration.UpdateCalibration()
                    new_heading = ApplyHeading(QPose, result['value'])
                    heading_off += new_heading - heading
                    while heading_off > 180: heading_off -= 180
                    while heading_off < -180: heading_off += 180
                    heading = new_heading

                heading = ApplyHeading(QPose, result['value'])
                lp_heading = .95*lp_heading + .05*(heading - heading_off)
                if cnt%10 == 0:
                    print 'alignment %.0f %.0f %.1f' % (heading, lp_heading, heading_off)
                    
    if False:
        points = []
        for line in file.readlines():
            data = json.loads(line.rstrip())
            value = data['value']
            points.append(value)

        print FitPoints(30, points)
    
