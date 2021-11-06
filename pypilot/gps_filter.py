#!/usr/bin/env python
#
#   Copyright (C) 2022 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# combine GPS and inertial measurements in a kalman filter to estimate
# speed and position with high output rate and better accuracy
# wave height
# boat speed

import os
import multiprocessing
import numpy as np
import math

from values import *
from resolv import *

from client import pypilotClient
from nonblockingpipe import NonBlockingPipe
import vector
import quaternion

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


class GPSFilterProcess(multiprocessing.Process):
    def __init__(self, client):
        self.client = pypilotClient(client.server)
        self.process = self
        self.pipe, pipe = NonBlockingPipe('gps filter pipe', True)
        super(GPSFilterProcess, self).__init__(target=self.filter_process, args=(pipe,), daemon=True)
        self.start()

    def predict(self, accel, fusionQpose_ned_magnetic, t):
        self.pipe.send(('predict', (accel, fusionQpose, t)))

    def update(self, gps, t):
        self.pipe.send(('update', (gps, t)))
        
    def filter_process(self, pipe):
        print('gps filter process', os.getpid())
        f = GPSFilter(self.client)
        while True:
            while True:
                inp = pipe.recv()
                if not inp:
                    break
                cmd, args = inp
                if cmd == 'predict':
                    f.predict(*args)
                elif cmd == 'update':
                    f.update(*args)

            msgs = self.client.receive(.1)
            for msg in msgs:
                self.values[msg] = msgs[msg]
        

class GPSFilter(object):
    def __init__(self, client):
        self.client = client
        self.gps_system_time_offset = 0
        self.stale_count = 0
        
        self.use3d = False # estimate altitude and climb

        posSigma = 10 # meters
        velSigma = .25  # m/s
        if self.use3d:
            self.R = np.diag([posSigma, posSigma, posSigma*2, velSigma, velSigma, velSigma*2])
        else:
            self.R = np.diag([posSigma, posSigma, velSigma, velSigma])
            
        self.enabled = self.register(BooleanProperty, 'enabled', False, persistent=True)
        self.output = self.register(BooleanProperty, 'output', False, persistent=True)
        self.declination = self.register(SensorValue, 'declination')
        self.declination_time = 0

        self.gps_time_offset = self.register(SensorValue, 'time_offset')
        self.gps_time_offset.update(.7)

        self.compass_offset = self.register(SensorValue, 'compass_offset')

        self.fix = self.register(Value, 'fix', False)
        self.speed = self.register(SensorValue, 'speed')
        self.track = self.register(SensorValue, 'track', directional=True)

        posDev = 30
        velDev = 3
        c = 3 if self.use3d else 2
        pos = np.diag([posDev**2]*c)
        vel = np.diag([velDev**2]*c)
        cov = np.diag([posDev*velDev]*c)

        self.Q = np.vstack((np.hstack((pos, cov)), np.hstack((cov, vel))))
        self.predict_t = 0

        self.reset()

    def register(self, _type, name, *args, **kwargs):
        return self.client.register(_type(*(['gps.filtered.' + name] + list(args)), **kwargs))

    def reset(self):
        c = 3 if self.use3d else 2
        self.X = False
        self.P = np.identity(2*c)
        self.history = []
        self.lastll = False

    def predict(self, accel, fusionQPose, t):
        if not self.enabled.value:
            return

        ta = time.monotonic()
        # convert accel to magnetic world frame
        accel_ned_magnetic = quaternion.rotvecquat(accel, fusionQPose)        
        
        # log new prediction and apply it estimating new state

        # subtract gravity
        residual_accel_magnetic = vector.sub(accel_ned_magnetic, [0, 0, 1])

        # rotate by declination
        decl_q = quaternion.angvec2quat(self.declination.value, [0, 0, 1])    
        accel_true = quaternion.rotvecquat(residual_accel_magnetic, decl_q)

        # apply predicted compass error
        error_q = quaternion.angvec2quat(self.compass_offset.value, [0, 0, 1])
        accel_ned = quaternion.rotvecquat(accel_true, error_q)

        U = 9.81*np.array(accel_ned)
        if not self.use3d:
            U = U[:2]

        #t = time.monotonic()
        dt = t - self.predict_t
        
        self.predict_t = t
        if dt < 0 or dt > .5:
            print('gpsfilter reset', dt)
            self.reset()

        if type(self.X) == bool and not self.X: # filter was reset            
            return # do not have a trusted measurement yet, so cannot perform predictions
        
        self.apply_prediction(dt, U)
        self.history.append({'t': t, 'dt': dt, 'U': U, 'X': self.X, 'P': self.P})

        # filtered position
        ll = xy_to_ll(self.X[0], self.X[1], *self.lastll)
        # filtered speed and track
        c = 3 if self.use3d else 2
        vx = self.X[c]
        vy = self.X[c+1]
        speed = math.hypot(vx, vy) * 1.94
        fix = {'lat': ll[0], 'lon': ll[1],
               'speed': speed, 'track': track}
        self.speed.set(speed)
        track = resolv(math.degrees(math.atan2(vx, vy)), 180)
        self.track.set(track)

        # filtered altitude and climb
        if self.use3d:
            self.fix['alt'] = self.X[2]
            self.fix['climb'] = self.X[5]

        tb = time.monotonic()

    def apply_prediction(self, dt, U):
        dt = min(max(dt, .02), .1)
        dt2 = dt*dt/2
        c = 3 if self.use3d else 2
        i = np.identity(c)
        B = np.vstack((dt2*i, dt*i))
        F = np.vstack((np.hstack((i, dt*i)),
                       np.hstack((np.zeros([c, c]), i))))
        #X = F*X + B*U
        self.X = F@self.X + B@U

        #P = F*P*Ft + Q
        self.P = F@self.P@F.transpose() + self.Q

    def update(self, data, t):
        if not self.enabled.value:
            return

        ts = data['timestamp']
        dt = t - ts + self.gps_system_time_offset
        if dt > 5: # older than 5 seconds
            print('gpsfilter stale time', dt)
            self.stale_count += 1
            if self.stale_count > 5:
                self.gps_system_time_offset = ts-t
                self.reset()
        else:
            self.stale_count = 0
            if dt < 0: # newer than now..
                print('gpsfilter reset time')
                self.gps_system_time_offset = ts-t
                self.reset()

        ts -= self.gps_system_time_offset # in system time
        ts -= self.gps_time_offset.value # apply gps reading lag

        # adjust coordinate frame
        ll = data['lat'], data['lon']
        if type(self.X) != bool:
            pll = xy_to_ll(self.X[0], self.X[1], *self.lastll)
            self.X[0], self.X[1] = ll_to_xy(pll[0], pll[1], *ll)
        self.lastll = ll

        # update magnetic declination from magnetic model once every few hours if available
        if t - self.declination_time > 3600 * 4:
            self.declination_time = t
            if wmm2020:
                year = datetime.date.today().year
                self.declination.update(wmm2020.wmm(self.lastll[0], self.lastll[1], 0, year).decl)

        c = 3 if self.use3d else 2
        try:
            
            #xy = ll_to_xy(data['lat'], data['lon'], *self.lastll)
            xy = 0, 0
            
            speed = data['speed'] / 1.944 # in meters/second
            track = data['track']
            
            Z = list(xy)
            if self.use3d:
                Z.append(data['alt'] if 'alt' in data else 0)

            Z += [speed*math.sin(math.radians(track)),
                  speed*math.cos(math.radians(track))]
            if self.use3d:
                Z.append(data['climb'] if 'climb' in data else 0)
        except Exception as e:
            print('gps filter update failed', e)
            return

        # based on the timestamp we need to rewind the filter to the prediction just before it
        t0 = False
        i = 0
        for i in range(len(self.history)):
            h = self.history[-i-1]
            t0 = h['t']
            if t0 < ts:
                self.X = h['X']
                self.P = h['P']
                break

        # compute time offset prediction and filter it
        if i > 0 and 0: # disable for now
            a0 = math.degrees(math.atan2(*self.history[-i-1]['X'][c:c+2]))
            a1 = math.degrees(math.atan2(*self.history[-i]['X'][c:c+2]))
            t1 = self.history[-i]['t']

            da = resolv(a1-a0)
            db = resolv(track-a0)
            # db / da = (ts-t0) / (t1-t0)
            # db*(t1-t0) = da*(ts-t0)
            nts = db/da*(t1-t0)

            nts = min(max(nts, t0), t1)
            dt = nts - ts # time update

            to = self.gps_time_offset.value
            to += dt * .0005  # filter it a lot
            to = min(max(t0, 0), 2)
            self.gps_time_offset.update(to)

        if type(self.X) == bool and not self.X: # filter was reset
            self.X = Z

        # apply normal kalman measurement update
        H = np.identity(2*c)
        
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
        curX = self.X.copy()
        self.X += K@Y

        #P = (I - K*H) * P
        self.P = (np.identity(2*c) - K@H) @ self.P

        # adjust compass alignment based on the disagreement in corrections
        if 0 and speed > 2:
            ax, ay = curX[c:c+2]   # accelerometer update
            cx, cy = self.X[c:c+2] # kalman measurement update

            ad, cd = math.hypot(ax, ay), math.hypot(cx, cy)
            s = ad / cd
            if s > .8 and s < 1.2:
                comp_adj = -math.degrees(math.asin((ax*cy-ay*cx) / (ad*cd)))
                comp_adj = min(max(comp_adj, -10), 10) # max error 10 degrees
                d = .0005 # filter
                self.compass_offset.set(resolv(self.compass_offset.value + d*comp_adj))

        # fast forward previous measurements
        for h in self.history[-i:]:
            self.apply_prediction(h['dt'], h['U'])

        self.history = self.history[-i:] # reset history
