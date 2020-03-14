#!/usr/bin/env python
#
#   Copyright (C) 2021 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import math

from pilot import AutopilotPilot
#from pypilot.resolv import resolv
from pypilot.values import *
from pypilot import vector

matrixfilepath = os.getenv('HOME') + '/.pypilot/' + 'fuzzy.json'

def fuzzy_defaults(dimensions, c=0):
    if dimensions:
        ret = {'N/A': fuzzy_defaults(dimensions[1:])}
        dimension = dimensions[0]
        name, value, step = dimension
        if name == 'heading error':
            for i in range(-5, 5):
                v = i*step
                ret[i] = fuzzy_defaults(dimensions[1:], v*.003 + c)
        elif name == 'heading rate':
            for i in range(-5, 5):
                v = i*step
                ret[i] = fuzzy_defaults(dimensions[1:], v*.09 + c)
        return ret
    return c

def fuzzy_matrix(index, matrix):
    # work index toward 0 when data deficient
    while not index in matrix:
        if index < 0:
            index += 1
        elif index > 0:
            index -= 1
        else:
            index = 'N/A' # fallback to data not available
    return matrix[index]

def fuzzy_compute(dimensions, matrix):
    if not dimensions:
        return matrix
        
    dimension = dimensions[0]
    name, sensor, step = dimension
    value = sensor.value
    if value is False:
        return fuzzy_compute(dimensions[1:], matrix['N/A'])

    index = value / step
    indexl = math.floor(index)
    indexh = indexl + 1
    d = index - indexl

    matrixl = fuzzy_matrix(indexl, matrix)
    matrixh = fuzzy_matrix(indexh, matrix)

    # recursively compute weighted average over all dimensions
    l = fuzzy_compute(dimensions[1:], matrixl)
    h = fuzzy_compute(dimensions[1:], matrixh)
    return d*l + (1-d)*h

def fuzzy_get(matrix, indicies):
    if not indicies:
        return matrix
    return fuzzy_get(fuzzy_matrix(indicies[0], matrix), indicies[1:])

def fuzzy_set(matrix, indicies, update):
    if len(indicies) == 1:
        matrix[indicies[0]] = update
    else:
        if not indicies[0] in matrix:
            matrix[indicies[0]] = {}
        fuzzy_set(matrix[indicies[0]], indicies[1:], update)

def fuzzy_train(dimensions, matrix, state, error):
    # determine indicies in matrix for this state
    indicies = []
    for i in range(len(state)):
        dimension = dimensions[i]
        value = state[i]
        name, sensor, step = dimension
        if value is False:
            index = 'N/A'
        else:
            index = round(value / step)
        indicies.append(index)

    current = fuzzy_get(matrix, indicies) # get current correction

    d = .005 #learning rate
    update = current + d*error # add error
    update = min(max(update, -1), 1) # bound to range

    fuzzy_set(matrix, indicies, update)   # update correction in matrix

class FuzzyPilot(AutopilotPilot):
    def __init__(self, ap):
        super(FuzzyPilot, self).__init__('fuzzy', ap)

        # create simple pid filter

        self.gains = {}
        
        self.learningP = self.register(RangeProperty, 'learningP', .003, 0, .02)
        self.learningD = self.register(RangeProperty, 'learningD', .09, 0, 1)
        
        self.seastate = self.register(SensorValue, 'seastate')
        self.accelm = 1

        #self.reaction_time = self.register(RangeProperty, 'reaction_time', 2, .1, 10)

        self.history_count = 40  # for now fixed to 40 iterations (2 seconds)
        self.history = []
        self.history_time = 0

        self.dimensions = [('ground speed', ap.sensors.gps.speed, 2),
                           ('wind speed', ap.sensors.wind.speed, 5),
                           ('wind direction', ap.sensors.wind.direction, 10),
                           ('rudder angle', ap.sensors.rudder.angle, 5),
                           ('heel', ap.boatimu.SensorValues['heel'], 5),
                           ('sea state', self.seastate, .1),
                           ('heading error', ap.heading_error, 3),
                           ('heading rate', ap.boatimu.SensorValues['headingrate_lowpass'], 2)]

        # will eventually need matrix for each mode
        self.matrix = fuzzy_defaults(self.dimensions)
        self.load()
        self.matrix_time = 0

    def store(self):
        try:
            f = open(matrixfilepath, 'w')
            f.write(pyjson.dumps(self.matrix))
            f.close()
        except Exception as e:
            print('failed to store fuzzy data', e)

    def load(self):
        try:
            f = open(matrixfilepath)
            self.matrix = pyjson.loads(f.read())
            f.close()
        except Exception as e:
            print('failed to load fuzzy data', e)
            
    def process(self):
        ap = self.ap

        # update sea state calculation
        accel = ap.boatimu.SensorValues['accel'].value
        if accel:
            self.accelm = .1*vector.norm(accel) + .9*self.accelm
            self.seastate.set(max(self.seastate * .99, self.accelm))

        if not ap.enabled.value:
            return

        # compute fuzzy command from matrix and command servo
        command = fuzzy_compute(self.dimensions, self.matrix)
        ap.servo.command.set(command)

        # feedback to update fuzzy matrix
        P = ap.heading_error.value
        D = ap.boatimu.SensorValues['headingrate_lowpass'].value
        error = P*self.learningP.value + D*self.learningD.value

        state = list(map(lambda x : x[1].value, self.dimensions))
        t = time.monotonic() # store fuzzy matrix every 10 minutes
        if t - self.history_time > .2:
            self.history.clear() # forget history when time skip
        self.history_time = t

        self.history.append((state, error))
        if len(self.history) == self.history_count:
            prev, self.history = self.history[0], self.history[1:]
            prev_state, prev_error = prev
            fuzzy_train(self.dimensions, self.matrix, prev_state, error)

            if t - self.matrix_time > 600:
                self.store()
                self.matrix_time = t


pilot = FuzzyPilot
