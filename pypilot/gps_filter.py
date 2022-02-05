#!/usr/bin/env python
#
#   Copyright (C) 2022 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# combine GPS and inertial measurements in a kalman filter to esimate
#
# speed and position with high output rate and better accuracy
# wave height
# boat speed

import numpy as np

from resolv import *

try:
    import wmm2020
except:
    print('world magnetic model not available')
    wmm2020 = False
            

earth_radius =  6378137.0
earth_md = earth_radius*2*math.pi/360

def ll_to_xy(lat, lon, lat0, lon0):
    cs = math.cos(math.radians(lat0))
    yc = lat - lat0
    xc = resolv(lon - lon0)
    return earth_md*xc*cs, earth_md*yc

def xy_to_ll(x, y, lat0, lon0):
    cs = math.cos(math.radians(lat0))
    xc = x/earth_md/cs
    yc = y/earth_md
    return yc + lat0, xc + lon0
    
class GPSFilter(object):
    def __init__(self, ap):
        self.ap = ap
        self.3d = False # estimate altitude and climb

        velSigma = posSigma * 1.0e-1;
        if self.3d:
            self.R = np.diag([posSigma, posSigma, posSigma*2, velSigma, velSigma, velSigma*2])
        else:
            self.R = np.diag([posSigma, posSigma, velSigma, velSigma])

        self.declination = self.register(SensorValue, 'declination')
        self.declination_time = 0

        self.gps_time_offset = self.register(SensorValue, 'time_offset')
        self.gps_time_offset.update(.5)

        self.compass_offset = self.register(SensorValue, 'compass_offset')

        self.lat = self.register(SensorValue, 'lat')
        self.lon = self.register(SensorValue, 'lon')
        self.speed = self.register(SensorValue, 'speed')
        self.track = self.register(SensorValue, 'track')

        c = 3 if self.3d else 2
        pos = np.diag([posDev**2]*c)
        vel = np.diag([velDev**2]*c)
        cov = np.diag([posDev*velDev]*c)

        # what to set this to? for accels?
        self.Q = np.vstack(np.hstack((pos, cov)), np.hstack((cov, vel)))

        self.predict_t = 0

        self.reset()

    def register(self, _type, name, *args, **kwargs):
        return self.ap.client.register(_type(*(['gps.filter.' + name] + list(args)), **kwargs))

    def reset(self):
        c = 3 if self.3d else 2
        self.X = False
        self.P = np.identity(2*c)
        self.history = []

    def predict(self, accel_ned_magnetic, t):
        # log new prediction and apply it estimating new state

        # subtract gravity
        residual_accel_magnetic = vector.sub(accel_neg_magnetic, [0, 0, 1])

        # rotate by declination
        decl_q = quaternion.angvec2quat(declination, [0, 0, 1])    
        accel_true = quaternion.rotvecquat(residual_accel_magnetic, decl_q)

        # apply predicted compass error
        error_q = quaternion.angvec2quat(compass_alignment, [0, 0, 1])
        accel_ned = quaternion.rotvecquat(accel_true, error_q)

        U = 9.81*accel_ned
        if not self.3d:
            U = U[:2]

        t = time.monotonic()
        dt = t - self.predict_t
        self.predict_t = t
        if dt < 0 or dt > .5:
            self.reset()

        self.apply_prediction(dt, U)
        self.history.append({'t': t, 'dt': dt, 'U': U, 'X': self.X, 'P', self.P})

        if not self.X: # do not have a trusted measurement yet, so cannot perform predictions
            return

        # filtered position
        ll = xy_to_ll(self.X[0], self.X[1], *self.lastll)
        self.lat.set(ll[0])
        self.lon.set(ll[1])

        # filtered speed and track
        c = 3 if self.3d else 2
        vx = self.X[c]
        vy = self.X[c+1]
        self.speed.set(math.hypot(vx, vy))
        self.track.set(resolv(math.degrees(math.atan2(vx, vy)), 180))

        # filtered altitude and climb
        if self.3d:
            self.alt.set(self.X[2])
            self.climb.set(self.X[5])

    def apply_prediction(self, dt, U):
        dt = min(max(dt, .02), .1)
        dt2 = dt*dt/2
        B = np.vstack((dt2*np.identity(c), dt*np.identity(c)))
        F = np.vstack((np.hstack((identity(c), dt*np.identity(c))),
                       np.hstack((zeros([c, c]), identity(c)))))
            
        #X = F*X + B*U
        self.X = F@self.X + B@U

        #P = F*P*Ft + Q
        self.P = F@self.P@F.transpose() + self.Q


    def update(self, gps):        
        # update magnetic declination from magnetic model once every few hours if available
        if t - self.declination_time > 3600 * 4:
            self.declination_time = t:
            if wmm2020:
                year = datetime.date.today().year
                self.declination.update(wmm2020.wmm(self.lastll[0], self.lastll[1], 0, year).decl)

        c = 3 if self.3d else 2
        try:
            ts = gps['timestamp'] - self.gps_time_offset.value # in system time
            
            xy = ll_to_xy(gps['lat'], gps['lon'], *self.lastll)
            speed = gps['speed'] / 1.944 # in meters/second
            track = gps['track']
            
            Z = xy
            if self.3d:
                Z.append(gps['alt'] if 'alt' in gps else 0)

            Z += [speed*math.sin(math.radians(track)),
                  speed*math.cos(math.radians(track))]
            if self.3d:
                Z.append(gps['climb'] if 'climb' in gps else 0)
        except Exception as e:
            print('gps filter update failed', e)
            return

        # based on the timestamp we need to rewind the filter to the prediction just before it
        t0 = False
        i = 0
        for i in len(self.history):
            h = self.history[-i-1]:
            t0 = h['t']
            if t0 < ts:
                self.X = h['X']
                self.P = h['P']
                break

        # compute time offset prediction and filter it
        if i > 0:
            a0 = math.degrees(math.atan2(*self.history[-i-1]['X'][c:c+1]))
            a1 = math.degrees(math.atan2(*self.history[-i]['X'][c:c+1]))
            t1 = self.history[-i]['t']

            da = resolv(a1-a0)
            db = resolv(track-a0)
            # db / da = (ts-t0) / (t1-t0)
            # db*(t1-t0) = da*(ts-t0)
            nts = db/da*(t1-t0) - t0
            nts = min(max(nts, t0), t1)
            dt = nts - ts # time update

            to = self.gps_time_offset.value
            to += dt * .0005  # filter it a lot
            to = min(max(t0, 0), 2)
            self.gps_time_offset.update(to)

        if not self.X: # filter was reset
            self.X = Z

        # apply normal kalman measurement update
        H = np.ident(2*c)
        
        #Y = Z - H*X
        Y = Z - H@self.X

        #S = H*P*Ht + R
        S = H@self.P@H.transpose() + self.R

        #K = P*Ht*S^-1
        try:
            invS = np.linalg.inv(S)
        except:
            # failed to invert matrix, reset filter?
            print('gps filter failed to invert S')
            return
        K = self.P@H.transpose()@invS

        #x = x + K*Y
        curX = self.X
        self.X += K@Y

        #P = (I - K*H) * P
        self.P = (np.identity(2*c) - K@H) @ self.P

        # adjust compass alignment based on the disagreement in corrections
        if speed > 2:
            ax, ay = curX[c:c+2]   # accelerometer update
            cx, cy = self.X[c:c+2] # kalman measurement update

            ad, cd = math.hypot(ax, ay), math.hypot(cx, cy)
            s = vd / wd
            if s > .8 and s < 1.2:
                comp_adj = -math.degrees(math.asin((ax*cy-ay*cx) / (ad*cd)))
                comp_adj = min(max(comp_adj, -10), 10) # max error 10 degrees
                d = .0005 # filter
                self.compass_alignment = resolv(self.compass_alignment + d*comp_adj)

        # reset projection origin
        self.lastll = xy_to_ll(self.X[0], self.X[1], *self.lastll)
        self.X[0] = self.X[1] = 0
                
        # fast forward previous measurements
        for h in self.history[-i:]:
            self.apply_prediction(h['dt'], h['U'])

        self.history = [] # reset history
