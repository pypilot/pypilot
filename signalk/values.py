#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# a value visible to external clients

import os, time, json
persistent_path = os.getenv('HOME') + '/.pypilot/pypilot.conf'

class Value(object):
    def __init__(self, name, initial, **kwargs):
        self.name = name

        # load from persistent data...
        self.value = initial
        self.watchers = []
        self.persistent = False # value is stored to config file
        if 'persistent' in kwargs and kwargs['persistent']:
            timeout = 0
            if 'persistent_timeout' in kwargs:
                timeout = kwargs['persistent_timeout']
            self.make_persistent(timeout)

    def make_persistent(self, timeout=0):
        self.persistent = True
        self.persistent_timeout = timeout
        self.persistent_time = 0
        data = {}
        try:
            file = open(persistent_path)
            data = json.loads(file.readline())
            file.close()
        except:
            print 'failed to load', persistent_path

        if self.name in data:
            self.value = data[self.name]

    def processes(self):
        return {'ops'   : self.ops,
                'get'   : self.get,
                'watch' : self.watch}

    def get_request(self):
        return json.dumps({self.name : {'value' : self.value}})

    def get(self, socket, data={}):
        socket.send(self.get_request() + '\n')

    def setdata(self, socket, data):
        self.set(data['value'])

    def type(self):
        return 'Value'

    def store_persistent(self):
        t = time.time()
        if t-self.persistent_time < self.persistent_timeout:
            return

        self.persistent_time = t
        
        data = {}
        try:
            file = open(persistent_path, 'r')
            data = json.loads(file.readline().rstrip())
            file.close()
        except:
            print 'failed to open', persistent_path

        data[self.name] = self.value
            
        try:
            file = open(persistent_path, 'w')
            file.write(json.dumps(data)+'\n')
            file.close()
        except:
            print 'failed to write', persistent_path


    def set(self, value):
        self.value = value
        if self.persistent:
            self.store_persistent()    
        
        if not self.watchers:
            return

        request = self.get_request()
        for socket in self.watchers:
            socket.send(request + '\n')

    def update(self, value):
        if self.value != value:
            self.set(value)

    def watch(self, socket, data):
        value = True
        if 'value' in data:
            value = str(data['value'])
            if value == 'False':
                value = False
            elif value != 'True':
                print 'watch value invalid', value
                raise

        if socket in self.watchers:
            if not value:
                self.watchers.remove(socket)
        else:
            if value:    
                self.get(socket) # retrieve current value when starting to watch
                self.watchers.append(socket)

    def ops(self, socket, data):
        request = {self.name : {'ops', list(self.processes())}}
        socket.send(json.dumps(request) + '\n')

class SensorValue(Value): # same as a Value with added timestamp
    def __init__(self, name, initial=False):
        super(SensorValue, self).__init__(name, initial)
        self.timestampholder = self ## it's possible to share timestamp with another value
        self.timestamp = time.time()

    def set(self, value):
        if self.timestampholder == self:
            self.timestamp = time.time()
        super(SensorValue, self).set(value)

    def get_request(self):
        try: # round to places, plus it's faster than json dumping
            if type(self.value) == type([]):
                strvalue = '['
                first = True
                for value in self.value:
                    if not first:
                        strvalue += ','
                    first = False
                    strvalue += '%.4f' % value
                strvalue += ']'
            elif type(self.value) == type(True):
                strvalue = 'true' if self.value else 'false'
            else:
                strvalue = '%.4f' % self.value
            request = '{"' + self.name + ('": {"value": %s, "timestamp": %.3f }}' % (strvalue, self.timestampholder.timestamp))
        except:
            # fallback to json dump which is slower
            request = json.dumps({self.name : {'value' : self.value, 'timestamp' : self.timestampholder.timestamp}})
        return request

    def type(self):
        return 'SensorValue'

# a value that may be modified by external clients
class Property(Value):
    def __init__(self, name, initial, **kwargs):
        super(Property, self).__init__(name, initial, **kwargs)

    def processes(self):
        p = super(Property, self).processes()
        p['set'] = self.setdata
        return p

    def type(self):
        return 'Property'

class ResettableValue(Property):
    def __init__(self, name, initial, **kwargs):
        super(ResettableValue, self).__init__(name, initial, **kwargs)
        self.initial = initial

    def processes(self):
        p = super(Property, self).processes()
        p['set'] = self.setdata
        return p

    def setdata(self, socket, data):
        if data['value'] != self.initial:
            print 'resettable value', self.name, 'invalid set'
        else:
            self.set(data['value'])

    def type(self):
        return 'ResettableValue'
    

class RangeProperty(Property):
    def __init__(self, name, initial, min_value, max_value, **kwargs):
        super(RangeProperty, self).__init__(name, initial, **kwargs)
        self.min_value = min_value
        self.max_value = max_value

    def get_request(self):
        # faster than json, saving digits in transmission also
        return '{"' + self.name + ('": {"value": %.4f}}' % self.value)
        
    def set(self, value):
        if value >= self.min_value and value <= self.max_value:
            super(RangeProperty, self).set(value)
            return True
        return False

    def type(self):
        return {'type' : 'RangeProperty', 'min' : self.min_value, 'max' : self.max_value}

class HeadingProperty(RangeProperty):
    def __init__(self, name, initial):
        super(HeadingProperty, self).__init__(name, initial, 0, 360)

    def set(self, value):
        while value < 0:
            value += 360
        while value >= 360:
            value -= 360
        super(HeadingProperty, self).set(value)

class EnumProperty(Property):
    def __init__(self, name, initial, choices, **kwargs):
        super(EnumProperty, self).__init__(name, initial, **kwargs)
        self.choices = choices

    def set(self, value):
        for choice in self.choices:
            if choice == value:
                super(EnumProperty, self).set(value)
                return True
        return False

    def type(self):
        return {'type' : 'EnumProperty', 'choices' : self.choices}

class BooleanProperty(Property):
    def __init__(self, name, initial, **kwargs):
        super(BooleanProperty, self).__init__(name, initial, **kwargs)

    def set(self, value):
        super(BooleanProperty, self).set(not not value)

    def get_request(self):
        try: # faster
            strvalue = 'true' if self.value else 'false'
            request = '{"' + self.name + '": {"value": ' + strvalue + '}}'
        except:
            request = json.dumps({self.name : {'value' : self.value}})
        return request

    def type(self):
        return 'BooleanProperty'
