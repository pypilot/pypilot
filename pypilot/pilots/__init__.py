# import all scripts in this directory

from __future__ import print_function
default = []

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import importlib

for module in os.listdir(os.path.dirname(__file__)):
    print(module)
    if module == '__init__.py' or module[-3:] != '.py' or module.startswith('.'):
        continue
    if module == 'pilot.py':
        continue

    try:
        mod = importlib.import_module('pilots.'+module[:-3])
    except Exception as e1:
        try:
            mod = importlib.import_module(module[:-3])
        except Exception as e2:
            print(_('ERROR loading'), module, e1, ' ', e2)
            continue
    try:
        if mod.disabled:
            continue
    except Exception as e:
        #print(e)
        pass

    try:
        default.append(mod.pilot)
    except:
        pass
