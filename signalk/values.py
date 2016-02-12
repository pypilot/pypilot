#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import os, time, json

class Value(object):
    def __init__(self, name, initial, **kwargs):
        self.name = name
        self.timestamp = False
        self.watchers = []
        self.persistent = False
        self.set(initial)
        self.client_can_set = False

        # value is stored to config file
        if 'persistent' in kwargs and kwargs['persistent']:
            self.persistent = True

    def type(self):
        return 'Value'

    def update(self, value):
        if self.value != value:
            self.set(value)

    def get_signalk(self):
        if type(self.value) == type(''):
            return '{"' + self.name + '": {"value": "' + self.value + '"}}'
        return '{"' + self.name + '": {"value": ' + str(self.value) + '}}'

    def set(self, value):
        self.value = value
        self.send()

    def send(self):
        if self.watchers:
            request = self.get_signalk() + '\n'
            for socket in self.watchers:
                socket.send(request)

class JSONValue(Value):
    def __init__(self, name, initial, **kwargs):
      super(JSONValue, self).__init__(name, initial, **kwargs)

    def get_signalk(self):
        return '{"' + self.name + '": {"value": ' + json.dumps(self.value) + '}}'


def round_value(value):
  if type(value) == type([]):
    ret = '['
    if len(value):
      ret += round_value(value[0])
      for item in value[1:]:
        ret += ', ' + round_value(item)
    return ret + ']'
  else:
    return '%.3f' % value

class RoundedValue(Value):
    def __init__(self, name, initial, **kwargs):
      super(RoundedValue, self).__init__(name, initial, **kwargs)
      
    def get_signalk(self):
      return '{"' + self.name + '": {"value": ' + round_value(self.value) + '}}'

class StringValue(Value):
    def __init__(self, name, initial):
        super(StringValue, self).__init__(name, initial)

    def get_signalk(self):
        if type(self.value) == type(False):
            strvalue = 'true' if self.value else 'false'
        else:
            strvalue = '"' + self.value + '"'
        return '{"' + self.name + '": {"value": ' + strvalue + '}}'

class SensorValue(Value): # same as Value with added timestamp
    def __init__(self, name, timestamp, initial=False, **kwargs):
        super(SensorValue, self).__init__(name, initial, **kwargs)
        if type(timestamp) != type('') and \
           (type(timestamp) != type([]) or len(timestamp) != 2 or type(timestamp[1]) != type('')):
            print 'invalid timstamp', timestamp, 'for sensorvalue', name
        self.timestamp = timestamp
        self.directional = 'directional' in kwargs and kwargs['directional']

    def type(self):
        if self.directional:
            return {'type': 'SensorValue', 'directional': True}
        return 'SensorValue'

    def get_signalk(self):
        value = self.value
        if type(value) == type(tuple()):
            value = list(value)
        return '{"' + self.name + '": {"value": ' + round_value(value) + ', "timestamp": %.3f }}' % self.timestamp[0]
    
# a value that may be modified by external clients
class Property(Value):
    def __init__(self, name, initial, **kwargs):
        super(Property, self).__init__(name, initial, **kwargs)
        self.client_can_set = True

    def type(self):
        return 'Property'

class ResettableValue(Property):
    def __init__(self, name, initial, **kwargs):
        self.initial = initial
        super(ResettableValue, self).__init__(name, initial, **kwargs)

    def type(self):
        return 'ResettableValue'

    def set(self, value):
        if not value:
            value = self.initial
        super(ResettableValue, self).set(value)
    

class RangeProperty(Property):
    def __init__(self, name, initial, min_value, max_value, **kwargs):
        self.min_value = min_value
        self.max_value = max_value
        if initial < min_value or initial > max_value:
            raise 'invalid initial value for range property', name, initial
        super(RangeProperty, self).__init__(name, initial, **kwargs)

    def type(self):
        return {'type' : 'RangeProperty', 'min' : self.min_value, 'max' : self.max_value}

    def get_signalk(self):
        return '{"' + self.name + ('": {"value": %.4f}}' % self.value)
        
    def set(self, value):
        if value >= self.min_value and value <= self.max_value:
            super(RangeProperty, self).set(value)

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
        self.choices = choices
        super(EnumProperty, self).__init__(name, initial, **kwargs)

    def type(self):
        return {'type' : 'EnumProperty', 'choices' : self.choices}

    def get_signalk(self):
        return '{"' + self.name + '": {"value": "' + self.value + '"}}'

    def set(self, value):
        for choice in self.choices:
            if choice == value:
                super(EnumProperty, self).set(value)
                return

class BooleanValue(Value):
    def __init__(self, name, initial, **kwargs):
        super(BooleanValue, self).__init__(name, initial, **kwargs)

    def get_signalk(self):
        strvalue = 'true' if self.value else 'false'
        return '{"' + self.name + '": {"value": ' + strvalue + '}}'

class BooleanProperty(Property):
    def __init__(self, name, initial, **kwargs):
        super(BooleanProperty, self).__init__(name, initial, **kwargs)

    def type(self):
        return 'BooleanProperty'

    def get_signalk(self):
        strvalue = 'true' if self.value else 'false'
        return '{"' + self.name + '": {"value": ' + strvalue + '}}'

    def set(self, value):
        super(BooleanProperty, self).set(not not value)
