#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import vector
import math

def angvec2quat(angle, v):
    n = vector.norm(v);
    if n == 0:
      fac = 0
    else:
        fac = math.sin(angle/2) / n

    return [math.cos(angle/2), v[0]*fac, v[1]*fac, v[2]*fac]


def angle(q):
    return 2*math.acos(q[0])


def vec2vec2quat(a, b):
    n = vector.cross(a, b);
    fac = vector.dot(a, b) / vector.norm(a) / vector.norm(b)
    fac = min(max(fac, -1), 1) # protect against possible slight numerical errors

    ang = math.acos(fac);
    return angvec2quat(ang, n);

def multiply(q1, q2):
    return [q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3], \
            q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2], \
            q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1], \
            q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0]]

# take a vector and quaternion, and rotate the vector by the quaternion
def rotvecquat(v, q):
    w = [0, v[0], v[1], v[2]]
    r = [q[0], -q[1], -q[2], -q[3]]
    return multiply(multiply(q, w), r)[1:]

def toeuler(q):
    roll = math.atan2(2.0 * (q[2] * q[3] + q[0] * q[1]), \
                      1 - 2.0 * (q[1] * q[1] + q[2] * q[2]))
    pitch = math.asin(2.0 * (q[0] * q[2] - q[1] * q[3]))
    heading = math.atan2(2.0 * (q[1] * q[2] + q[0] * q[3]), \
                         1 - 2.0 * (q[2] * q[2] + q[3] * q[3]))
    return roll, pitch, heading

def conjugate(q):
    return [q[0], -q[1], -q[2], -q[3]]

def normalize(q):
    d = math.sqrt(vector.dot(q, q))
    return [q[0] / d, q[1] / d, q[2] / d, q[3] / d]
