#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import os, time, math
import pyjson

class Value(object):
    def __init__(self, name, initial, **kwargs):
        self.name = name
        self.watch = False
        self.set(initial)

        self.info = {'type': 'Value'}
        # if persistent argument make the server store/load this value regularly
        if 'persistent' in kwargs and kwargs['persistent']:
            self.info['persistent'] = True

    def update(self, value):
        if self.value != value:
            self.set(value)

    def get_msg(self):
        if isinstance(self.value, str):
            return '"' + self.value + '"'
        return str(self.value)

    def set(self, value):
        self.value = value
        if self.watch:
            if self.watch.period == 0: # and False:   # disable immediate
                self.client.send(self.name+'='+self.get_msg()+'\n')

            elif self.pwatch:
                t0 = time.monotonic()
                if t0 >= self.watch.time:
                    self.watch.time = t0 # watch already expired, increment time
                self.client.values.insert_watch(self.watch)
                self.pwatch = False

class JSONValue(Value):
    def __init__(self, name, initial, **kwargs):
      super(JSONValue, self).__init__(name, initial, **kwargs)

    def get_msg(self):
        return pyjson.dumps(self.value)

def round_value(value, fmt):
    if type(value) == type([]):
        ret = '['
        if len(value):
            ret += round_value(value[0], fmt)
            for item in value[1:]:
                ret += ', ' + round_value(item, fmt)
        return ret + ']'
    elif type(value) == type(False):
        if value:
            return 'true'
        return 'false'
    try:
        if math.isnan(value):
            return '"nan"'
        return fmt % value
    except Exception as e:
        return str(e)

class RoundedValue(Value):
    def __init__(self, name, initial, **kwargs):
        super(RoundedValue, self).__init__(name, initial, **kwargs)
      
    def get_msg(self):
        return round_value(self.value, '%.3f')

class StringValue(Value):
    def __init__(self, name, initial, **kwargs):
        super(StringValue, self).__init__(name, initial, **kwargs)

    def get_msg(self):
        if type(self.value) == type(False):
            strvalue = 'true' if self.value else 'false'
        else:
            strvalue = '"' + self.value + '"'
        return strvalue

class SensorValue(Value):
    def __init__(self, name, initial=False, fmt='%.3f', **kwargs):
        super(SensorValue, self).__init__(name, initial, **kwargs)
        self.directional = 'directional' in kwargs and kwargs['directional']
        self.fmt = fmt # round to 3 places unless overrideen

        self.info['type'] = 'SensorValue'
        if self.directional:
            self.info['directional'] = True

    def get_msg(self):
        value = self.value
        if type(value) == type(tuple()):
            value = list(value)
        return round_value(value, self.fmt)

# a value that may be modified by external clients
class Property(Value):
    def __init__(self, name, initial, **kwargs):
        super(Property, self).__init__(name, initial, **kwargs)
        self.info['writable'] = True

class ResettableValue(Property):
    def __init__(self, name, initial, **kwargs):
        self.initial = initial
        super(ResettableValue, self).__init__(name, initial, **kwargs)

    def type(self):
        return {'type': 'ResettableValue'}

    def set(self, value):
        if not value:
            value = self.initial # override value
        super(ResettableValue, self).set(value)

class RangeProperty(Property):
    def __init__(self, name, initial, min_value, max_value, **kwargs):
        self.min_value = min_value
        self.max_value = max_value
        if initial < min_value or initial > max_value:
            print('invalid initial value for range property', name, initial)
        super(RangeProperty, self).__init__(name, initial, **kwargs)

        self.info['type'] = 'RangeProperty'
        self.info['min'] = self.min_value
        self.info['max'] = self.max_value

    def get_msg(self):
        return '%.4f' % self.value
        
    def set(self, value):
        try:
            value = float(value) # try to convert to number
        except:
            return # ignore invalid value
        if value >= self.min_value and value <= self.max_value:
            super(RangeProperty, self).set(value)

    def set_max(self, max_value):
        if self.value > max_value:
            self.value = max_value
        self.max_value = max_value

# a range property that is persistent and specifies the units
class RangeSetting(RangeProperty):
    def __init__(self, name, initial, min_value, max_value, units):
        self.units = units
        super(RangeSetting, self).__init__(name, initial, min_value, max_value, persistent=True)

        self.info['type'] = 'RangeSetting'
        self.info['units'] = self.units

class EnumProperty(Property):
    def __init__(self, name, initial, choices, **kwargs):
        self.choices = choices
        super(EnumProperty, self).__init__(name, initial, **kwargs)
        self.info['type'] = 'EnumProperty'
        self.info['choices'] = self.choices
        
    def set(self, value):
        for choice in self.choices:
            try: # accept floating point equivilent, 10.0 is 10
                if float(choice) != float(value):
                    continue
            except:
                if str(choice) != str(value):
                    continue
            super(EnumProperty, self).set(value)
            return
        print('invalid set', self.name, '=', value)

class BooleanValue(Value):
    def __init__(self, name, initial, **kwargs):
        super(BooleanValue, self).__init__(name, initial, **kwargs)

    def get_msg(self):
        return 'true' if self.value else 'false'

class BooleanProperty(BooleanValue):
    def __init__(self, name, initial, **kwargs):
        super(BooleanProperty, self).__init__(name, initial, **kwargs)
        self.info['writable'] = True
        self.info['type'] = 'BooleanProperty'

    def set(self, value):
        super(BooleanProperty, self).set(not not value)
