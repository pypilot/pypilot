#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import sys, time
from signalk.client import SignalKClient

# list must be already sorted
def unique(l):
    if len(l) < 2:
        return l
    if l[0] == l[1]:
        return unique(l[1:])
    return [l[0]] + unique(l[1:])

def frange(min, max, step):
    def each(val):
        if val > max:
            return []
        return [val] + each(val+step)
    return each(min)

class autogain(object):
    def __init__(self):
        self.search = [ \
                       #{'name': 'ap/I', 'min': 0, 'max': .006, 'step': .003},
                       #{'name': 'ap/P2', 'min': 0, 'max': .006, 'step': .006},
                       {'name': 'ap/P', 'min': .002, 'max': .006, 'step': .001},
                       {'name': 'ap/D', 'min': .06, 'max': .12, 'step': .01}]
        self.variables = ['ap/heading_error', 'servo/Watts', 'servo/current', 'gps/speed']
        self.settle_period = 2
        self.period = 5

        self.watchlist = ['ap/enabled']
        for var in self.search:
            self.watchlist.append(var['name'])
        for var in self.variables:
            self.watchlist.append(var)

        def on_con(client):
            for name in self.watchlist:
                client.watch(name)

        print 'connecting to server...'
        host = False
        if len(sys.argv) > 1:
            host = sys.argv[1]

        while True:
            try:
                self.client = SignalKClient(on_con, host, autoreconnect=True)
                break
            except:
                time.sleep(2)

        print 'connected'

    def read_messages(self, log):
        msgs = self.client.receive()
        for name in msgs:
            data = msgs[name]
            value = data['value']

            for var in self.search:
                if name == var:
                    name = name[3:]
                    if abs(value - self.gains[name].value) > 1e-8:
                        print 'external program adjusting search variable!!, abrort', name, value
                        exit(0)

            if log:
                for var in self.variables:
                    if name == var:
                        self.total[name]['total'] += value
                        self.total[name]['count'] += 1
            if name == 'ap/enabled' and not value:
                #print 'autopilot disabled!!'
                #exit(0)
                pass

    def set(self, name, val):
        print 'setting', name, 'to', val
        self.searchval[name] = val
        self.client.set(name, val)

    def log(self):
        print 'logging for', self.searchval
        t0 = time.time()
        self.total = {}
        for var in self.variables:
            self.total[var] = {'total': 0, 'count': 0}
        while time.time() - t0 < self.period:
            self.read_messages(time.time() - t0 > self.settle_period)
            time.sleep(.05)

        for var in self.variables:
            if not var in self.results:
                self.results[var] = []
            count = self.total[var]['count']
            if count:
                self.results[var].append((self.searchval.copy(), self.total[var]['total'] / count))
            else:
                print 'warning, no results for', var
        
    def run_search(self, search):
        if search:
            s = search[0]
            for val in frange(s['min'], s['max'], s['step']):
                self.set(s['name'], val)
                self.run_search(search[1:])
        else:
            self.log()

    def result_range(self, results, name):
        r = []
        for result in results:
            vars, val = result
            r.append(vars[name])
        return unique(sorted(r))

    def result_value(self, results, vals):
        values = []
        for result in results:
            vars, val = result
            if vars == vals:
                values.append(val)
        if len(values) == 1:
            return '%.3f' % values[0]
        return values
        
    def print_results(self, results, search, vals):
        l = len(search)
        if l < 2:
            print 'error, need at least 2 search variables'
            exit(1)

        s = search[0]
        if l > 2:
            for val in self.result_range(results, s['name']):
                print s['name'], '=', val
                vals[s['name']] = val
                self.print_results(results, search[1:], vals)
                
        elif l == 2:
            t = search[1]
            print s['name'], '/', t['name']
            line = '\t'
            s_range = self.result_range(results, s['name'])
            for val0 in s_range:
                line += '%.3f\t' % val0
            print line
            for val1 in self.result_range(results, t['name']):
                line = '%.3f\t' % val1
                vals[t['name']] = val1
                for val0 in s_range:
                    vals[s['name']] = val0
                    line += str(self.result_value(results, vals)) + '\t'
                print line
            print ''

    def run(self):
        self.searchval = {}
        self.results = {}
        self.run_search(self.search)
        for var in self.variables:
            print 'Results for', var
            self.print_results(self.results[var], self.search, {})
            print ''

if __name__ == '__main__':
    ag = autogain()
    ag.run()
