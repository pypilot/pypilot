#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# a value visible to external clients

import json

class Value(object):
    def __init__(self, name, initial):
        self.name = name
        self.value = initial
        self.watchers = []

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

    def set(self, value):
        self.value = value
        if not self.watchers:
            return
        request = self.get_request()
        for socket in self.watchers:
            socket.send(request + '\n')

    def watch(self, socket, data):
        value = True
        if 'value' in data:
            value = str(data['value'])
            if value == 'False':
                value = False
            elif value != 'True':
                print "watch value invalid", value
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
    def __init__(self, name, timestampholder, initial=False):
        super(SensorValue, self).__init__(name, initial)
        self.timestampholder = timestampholder

    def get_request(self):
        try:
            if type(self.value) == type([]):
                strvalue = '['
                first = True
                for value in self.value:
                    if not first:
                        strvalue += ','
                    first = False
                    strvalue += '%.4f' % value
                strvalue += ']'
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
    def __init__(self, name, initial):
        super(Property, self).__init__(name, initial)

    def processes(self):
        p = super(Property, self).processes()
        p['set'] = self.setdata
        return p

    def type(self):
        return 'Property'
            
class RangeProperty(Property):
    def __init__(self, name, initial, min_value, max_value):
        super(RangeProperty, self).__init__(name, initial)
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
    def __init__(self, name, initial, choices):
        super(EnumProperty, self).__init__(name, initial)
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
    def __init__(self, name, initial):
        super(BooleanProperty, self).__init__(name, initial)

    def set(self, value):
        super(BooleanProperty, self).set(not not value)

    def get_request(self):
        try: # faster than json, saving digits
            request = '{"' + self.name + ('": {"value": %.4f}}' % self.value)
        except:
            request = json.dumps({self.name : {'value' : self.value}})
        return request

    def type(self):
        return 'BooleanProperty'
