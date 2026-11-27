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

import math
import datetime
import multiprocessing
import os
from collections import deque

import quaternion
from client import pypilotClient
from nonblockingpipe import NonBlockingPipe
from resolv import *
from values import *

try:
    import pygeomag
except ImportError:
    print('world magnetic model not available')
    pygeomag = None


earth_radius =  6378137.0
earth_md = earth_radius*2*math.pi/360
DEFAULT_GPS_DELAY = 0.12  # seconds for sources other than local serial and gpsd
GPS_HISTORY_SECONDS = 5
MAX_IMU_GAP = .5  # seconds
MPS_PER_KNOT = 1852 / 3600
# Effective per-axis velocity random walk in (m/s)/sqrt(s), allowing for
# attitude and gravity-removal errors over short gaps between GPS fixes.
VELOCITY_RANDOM_WALK = 0.1

def ll_to_xy(lat, lon, lat0, lon0):
    cs = math.cos(math.radians(lat0))
    yc = lat - lat0
    xc = resolv(lon - lon0)
    return earth_md*xc*cs, earth_md*yc

def xy_to_ll(x, y, lat0, lon0):
    cs = math.cos(math.radians(lat0))
    xc = x/earth_md/cs
    yc = y/earth_md
    return yc + lat0, resolv(xc + lon0)


def finite_number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


class GPSFilterProcess(multiprocessing.Process):
    def __init__(self, client):
        self.client = pypilotClient(client.server)
        self.process = self

        # this value is registered in main process
        self.output = client.register(BooleanProperty('gps.filtered.output', False, persistent=True))

        self.pipe, pipe = NonBlockingPipe('gps filter pipe', True)
        super().__init__(target=self.filter_process, args=(pipe,), daemon=True)
        self.start()

    def predict(self, accel, fusionQpose_ned_magnetic, t):
        self.pipe.send(('predict', (accel, fusionQpose_ned_magnetic, t)))

    def update(self, gps, t, source=None):
        self.pipe.send(('update', (gps, t, source)))

    def filter_process(self, pipe):
        # wait for numpy package to be available
        while True:
            try:
                global np
                import numpy as np
                break
            except ImportError:
                pass
            time.sleep(20)
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

            self.client.receive(.1)  # dispatch writes to the registered settings


class GPSFilter:
    def __init__(self, client):
        self.client = client
        self.gps_system_time_offset = 0
        self.stale_count = 0

        self.use3d = False # estimate altitude and climb

        self.enabled = self.register(BooleanProperty, 'enabled', False, persistent=True)
        self.position_error = self.register(RangeSetting, 'position_error', 10, .01, 1000, 'meters')
        self.velocity_error = self.register(RangeSetting, 'velocity_error', .5, .01, 10, 'knots')
        self.declination = self.register(SensorValue, 'declination')
        self.declination_time = None

        self.gps_time_offset = self.register(SensorValue, 'time_offset', DEFAULT_GPS_DELAY)
        self.gps_source = None

        self.compass_offset = self.register(SensorValue, 'compass_offset')

        self.fix = self.register(JSONValue, 'fix', False)
        self.speed = self.register(SensorValue, 'speed')
        self.track = self.register(SensorValue, 'track', directional=True)

        self.reset()

    def register(self, _type, name, *args, **kwargs):
        return self.client.register(_type(*(['gps.filtered.' + name] + list(args)), **kwargs))

    def reset(self):
        c = 3 if self.use3d else 2
        self.X = False
        self.P = np.identity(2*c)
        # The checkpoint and live state share the origin lastll. The checkpoint
        # includes the last GPS correction; only IMU inputs need to be replayed.
        self.checkpoint = None  # (monotonic time, X, P)
        self.history = deque()  # (sample time, acceleration held until the next sample)
        self.lastll = False
        self.fix_epoch = None
        self.fix_rows = set()

    def predict(self, accel, fusionQPose, t):
        if not self.enabled.value:
            return

        # Store magnetic-frame inputs so even samples received before the first
        # GPS fix can use its declination when replayed after that fix.
        north, east, down = quaternion.rotvecquat(accel, fusionQPose)
        U = (-9.81 * east, -9.81 * north, 9.81 * (down - 1))

        if self.history:
            dt = t - self.history[-1][0]
            if dt == 0:
                return
            if dt < 0 or dt > MAX_IMU_GAP:
                print('gpsfilter reset', dt)
                self.reset()

        if self.checkpoint is not None:
            start = self.checkpoint[0]
            if t < start:
                return
            last_t, accel = self.history[-1] if self.history else (start, None)
            self.X, self.P = self.predict_interval(self.X, self.P, t - max(last_t, start), accel)
        self.history.append((t, U))

        # During a GPS outage, move the checkpoint forward before discarding
        # old inputs. Live predictions continue and memory use stays bounded.
        cutoff = t - GPS_HISTORY_SECONDS
        if len(self.history) > 1 and self.history[1][0] <= cutoff:
            if self.checkpoint is not None and self.checkpoint[0] < cutoff:
                self.checkpoint = (cutoff, *self.replay(*self.checkpoint, until=cutoff))
            self.discard_before(cutoff)
        if self.checkpoint is None:
            return

        # filtered position
        ll = xy_to_ll(self.X[0], self.X[1], *self.lastll)
        # filtered speed and track
        c = 3 if self.use3d else 2
        vx = self.X[c]
        vy = self.X[c+1]
        speed = math.hypot(vx, vy) / MPS_PER_KNOT
        self.speed.set(speed)
        track = resolv(math.degrees(math.atan2(vx, vy)), 180)
        self.track.set(track)

        fix = {'lat': ll[0], 'lon': ll[1],
               'speed': speed, 'track': track,
               'timestamp': time.time()}
        # filtered altitude and climb
        if self.use3d:
            fix['alt'] = self.X[2]
            fix['climb'] = self.X[5]
        self.fix.set(fix)

    def true_accel(self, accel):
        # A positive correction turns magnetic north toward true east.
        e, n, up = accel
        angle = math.radians(self.declination.value + self.compass_offset.value)
        cs, sn = math.cos(angle), math.sin(angle)
        rotated = (e*cs + n*sn, n*cs - e*sn, up)
        return np.array(rotated if self.use3d else rotated[:2])

    def predict_interval(self, X, P, dt, accel):
        if not dt:
            return X, P
        c = len(X) // 2
        axes = np.identity(c)
        F = np.identity(2*c)
        F[:c, c:] = dt * axes
        X = F @ X
        if accel is not None:
            accel = self.true_accel(accel)
            X[:c] += .5 * dt**2 * accel
            X[c:] += dt * accel
        P = F @ P @ F.T
        # Integrated white acceleration noise: q * [[dt³/3, dt²/2], [dt²/2, dt]]
        # per axis. Splitting dt at an IMU sample or a GPS fix gives the same covariance.
        velocity_noise = VELOCITY_RANDOM_WALK**2 * dt * axes
        cross_noise = velocity_noise * (dt / 2)
        P[:c, :c] += velocity_noise * (dt**2 / 3)
        P[:c, c:] += cross_noise
        P[c:, :c] += cross_noise
        P[c:, c:] += velocity_noise
        return X, P

    def replay(self, start, X, P, until=None):
        """Predict from the checkpoint to a fix, or through the latest IMU sample."""
        accel = None
        for t, sample in self.history:
            if until is not None and t >= until:
                break
            if t >= start:
                X, P = self.predict_interval(X, P, t - start, accel)
                start = t
            accel = sample
        if until is not None:
            X, P = self.predict_interval(X, P, until - start, accel)
        return X, P

    def discard_before(self, t):
        # Keep the acceleration in effect at t, even when t falls between samples.
        while len(self.history) > 1 and self.history[1][0] <= t:
            self.history.popleft()

    def update(self, data, t, source=None):
        if not self.enabled.value:
            return

        ts = data.get('timestamp')
        lat, lon = data.get('lat'), data.get('lon')
        if not (finite_number(ts) and finite_number(lat) and -90 <= lat <= 90 and
                finite_number(lon) and -180 <= lon <= 180):
            return

        if source != self.gps_source:
            self.gps_source = source
            self.gps_time_offset.update(0.09 if source in ('serial', 'gpsd') else DEFAULT_GPS_DELAY)
            # The clock offset is estimated from arrival times on this input.
            self.gps_system_time_offset = ts - t

        dt = t - ts + self.gps_system_time_offset
        if dt > GPS_HISTORY_SECONDS:
            self.stale_count += 1
            if self.stale_count <= 5:
                return
            self.gps_system_time_offset = ts-t
            self.reset()
        else:
            self.stale_count = 0
            if dt < 0:
                # A faster delivery improves the clock estimate. Only a clock
                # jump warrants throwing away the navigation state.
                if dt < -GPS_HISTORY_SECONDS and self.checkpoint is not None:
                    self.reset()
                self.gps_system_time_offset = ts-t

        fix_timestamp = ts
        ts -= self.gps_system_time_offset # in system time
        ts -= self.gps_time_offset.value # apply gps reading lag
        if ((self.checkpoint is not None and ts < self.checkpoint[0]) or
                (self.history and ts < self.history[-1][0] - GPS_HISTORY_SECONDS)):
            return
        if self.history and ts - self.history[-1][0] > MAX_IMU_GAP:
            # GPS can resume before IMU predictions do. Start with this fix
            # instead of integrating the last acceleration across the outage.
            self.reset()

        c = 3 if self.use3d else 2
        rows, Z = [0, 1], [0., 0.]  # measure position relative to the new fix
        if self.use3d and finite_number(data.get('alt')):
            rows.append(2)
            Z.append(data['alt'])

        speed, track = data.get('speed'), data.get('track')
        have_velocity = finite_number(speed) and speed >= 0 and finite_number(track)
        if have_velocity:
            speed *= MPS_PER_KNOT
            angle = math.radians(track)
            rows.extend((c, c+1))
            Z.extend((speed*math.sin(angle), speed*math.cos(angle)))
        if self.use3d and finite_number(data.get('climb')):
            rows.append(5)
            Z.append(data['climb'])

        # Signal K can deliver velocity later for a position already fused.
        # At the same GPS epoch only assimilate newly supplied components.
        epoch = source, fix_timestamp
        if epoch == self.fix_epoch:
            new_measurements = [(row, z) for row, z in zip(rows, Z) if row not in self.fix_rows]
            if not new_measurements:
                return
            rows, Z = map(list, zip(*new_measurements))

        ll = lat, lon
        if self.checkpoint is None:
            X = np.zeros(2*c)
            P = self.P.copy()
            self.initial_epoch = epoch
        else:
            X, P = self.replay(*self.checkpoint, until=ts)
            # Rebase the prediction at the measurement time, before comparing
            # it with the GPS position. The saved state must stay unmodified.
            east_scale = math.cos(math.radians(lat)) / math.cos(math.radians(self.lastll[0]))
            X = X.copy()
            X[0] *= east_scale
            X[:2] += ll_to_xy(*self.lastll, *ll)
            P = P.copy()
            P[0, :] *= east_scale
            P[:, 0] *= east_scale

        # Disabled experiment: interpolate GPS course between two predicted tracks.
        if 0 and have_velocity and self.checkpoint is not None:
            for (t0, accel), (t1, _) in zip(self.history, list(self.history)[1:]):
                if t0 < ts <= t1:
                    v = X[c:c+2]
                    accel = self.true_accel(accel)
                    a0 = math.degrees(math.atan2(*(v + (t0-ts)*accel[:2])))
                    a1 = math.degrees(math.atan2(*(v + (t1-ts)*accel[:2])))
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
                    break

        # Per-axis GPS standard deviations. Use twice the horizontal error
        # for altitude and climb when estimating in 3D.
        sigma = np.array([self.position_error.value]*c + [self.velocity_error.value * MPS_PER_KNOT]*c)
        if self.use3d:
            sigma[[2, 5]] *= 2
        variances = sigma[rows]**2
        curX = X
        if epoch == self.initial_epoch:
            # The first observation supplies both the estimate and its uncertainty.
            X[rows] = Z
            P[rows, rows] = variances
        else:
            I = np.identity(2*c)
            H = I[rows]
            R = np.diag(variances)
            Y = Z - H@X
            S = H@P@H.T + R
            try:
                K = P@H.T@np.linalg.inv(S)
            except np.linalg.LinAlgError:
                print('gps filter failed to invert S')
                return
            X = X + K@Y
            P = (I - K@H) @ P

        # adjust compass alignment based on the disagreement in corrections
        if 0 and have_velocity and speed > 2:
            ax, ay = curX[c:c+2]   # accelerometer update
            cx, cy = X[c:c+2] # kalman measurement update

            ad, cd = math.hypot(ax, ay), math.hypot(cx, cy)
            s = ad / cd
            if s > .8 and s < 1.2:
                comp_adj = -math.degrees(math.asin((ax*cy-ay*cx) / (ad*cd)))
                comp_adj = min(max(comp_adj, -10), 10) # max error 10 degrees
                d = .0005 # filter
                self.compass_offset.set(resolv(self.compass_offset.value + d*comp_adj))

        # Refresh on the first fix and every four hours of GPS time.
        if pygeomag and (self.declination_time is None or abs(fix_timestamp - self.declination_time) >= 4 * 3600):
            self.declination_time = fix_timestamp
            try:
                date = datetime.datetime.fromtimestamp(fix_timestamp, datetime.timezone.utc)
                year = pygeomag.decimal_year_from_date(date)
                model = pygeomag.GeoMag(base_year=date.year)
                declination = model.calculate(lat, lon, 0, year).d
            except (OSError, OverflowError, ValueError) as e:
                print('gps filter magnetic declination unavailable:', e)
            else:
                self.declination.update(declination)

        self.lastll = ll
        if epoch != self.fix_epoch:
            self.fix_rows.clear()
        self.fix_rows.update(rows)
        self.fix_epoch = epoch
        self.checkpoint = ts, X, P
        self.discard_before(ts)
        self.X, self.P = self.replay(*self.checkpoint)
