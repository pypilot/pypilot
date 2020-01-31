# import all scripts in this directory

from __future__ import print_function
default = []

import os

import importlib

for module in os.listdir(os.path.dirname(__file__)):
    if module == '__init__.py' or module[-3:] != '.py' or module.startswith('.'):
        continue
    try:
        mod = importlib.import_module('pilots.'+module[:-3])
    except Exception as e1:
        try:
            mod = importlib.import_module(module[:-3])
        except Exception as e2:
            print('ERROR loading', module, e1, ', ', e2)
            continue
    try:
        if not mod.pilot.disabled:
            default.append(mod.pilot)
    except Exception:
        print('pilot not defined in', module)
