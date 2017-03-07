#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time
from signalk.client import SignalKClient

class gain():
    def __init__(self, value, min_val, max_val, step):
        self.value, self.min_val, self.max_val, self.step = value, min_val, max_val, step

    def integrate(self):
        self.value += self.step

        if self.value > self.max_val:
            print 'gain reached max search'
            self.value = self.max_val
        elif self.value < self.min_val:
            print 'gain reached min search'
            self.value = self.min_val

    def reverse(self):
        self.step = -self.step

class autogain():
    def __init__(self):
        self.gains = {'P' : gain(.01, .005, .02, .001), 'D' : gain(.1, .05, .2, .01)}
        self.period = 60

        watchlist = ['ap/enabled', 'ap/heading_error', 'servo/current']

        def setup_watches(client, watch=True):
            for name in watchlist:
                client.watch(name, watch)
            for gain in self.gains:
                client.watch('ap/' + gain)

        def on_con(client):
            setup_watches(client)

        print 'connecting to server...'
        while True:
            try:
                self.client = SignalKClient(on_con, autoreconnect=True)
                break
            except:
                time.sleep(2)

        print 'connected'
        self.reset_fitness()

        for i in range(100):
            self.read_messages()
            time.sleep(.01)

        self.last_fitness = 0

    def reset_fitness(self):
        self.heading_error = (0, 0)
        self.current = (0, 0)

    def read_messages(self):
        msgs = self.client.receive()
        for name in msgs:
            data = msgs[name]
            value = data['value']

            for gain in self.gains:
                if name == 'ap/' + gain:
                    name = name[3:]
                    if abs(value - self.gains[name].value) > 1e-8:
                        print 'external program adjusting gain!!', name, value
                        self.gains[name].value = value
                    return

            def accumulate(var, val):
                return var[0] + val, var[1] + 1

            if name == 'ap/heading_error':
                self.heading_error = accumulate(self.heading_error, value**2)
            elif name == 'servo/current':
                self.current = accumulate(self.current, value)
            elif name == 'ap/enabled' and not value:
                print 'autopilot disabled!!'

    def send_gain(self, name):
        self.client.set('ap/' + name, self.gains[name].value)

    def optimize_gain(self, name):
        self.gains[name].integrate()
        self.send_gain(name)
        print 'trying', name, '=', self.gains[name].value

        self.reset_fitness()

        # record data to optimize gains
        t0 = time.time()
        while time.time() - t0 < self.period:
            self.read_messages()
            time.sleep(.01)

        if self.heading_error[1] == 0:
            print 'no fitness'
            return

        def average_total(var):
            return var[0] / var[1];
        
        heading_error = average_total(self.heading_error)
        current = average_total(self.current)
        print 'heading error', heading_error, 'average current', current

        fitness = heading_error + current
        print 'fitness', fitness
        
        if fitness < self.last_fitness:
            print 'improved fitness from', self.last_fitness, 'to', fitness
        else:
            print 'fitness is worse, resetting gain'
            self.gains[name].reverse()    # reverse direction
            self.gains[name].integrate()
            self.send_gain(name)
        self.last_fitness = fitness
            
    def run(self):
        gaini = 0
        while True:
            gain = list(self.gains)[gaini]
            gaini += 1
            if gaini == len(self.gains):
                gaini = 0

            self.optimize_gain(gain)

if __name__ == '__main__':
    ag = autogain()
    ag.run()
