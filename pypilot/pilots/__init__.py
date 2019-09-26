# import all scripts in this directory

from __future__ import print_function
default = []

import os
for module in os.listdir(os.path.dirname(__file__)):
    if module == '__init__.py' or module[-3:] != '.py' or module.startswith('.'):
        continue
    try:
        __import__('pilots.'+module[:-3], locals(), globals())
    except Exception as e:
        print('ERROR loading', module, e)
    del module

default += [simple.SimplePilot, basic.BasicPilot, learning.LearningPilot, wind.WindPilot, absolute.AbsolutePilot]
