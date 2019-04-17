# import all scripts in this directory

default = []

import os
for module in os.listdir(os.path.dirname(__file__)):
    if module == '__init__.py' or module[-3:] != '.py' or module.startswith('.'):
        continue
    try:
        __import__(module[:-3], locals(), globals())
    except Exception as e:
        print 'ERROR loading', module, e
    del module

default += [simple.SimplePilot, basic.BasicPilot, learning.LearningPilot, wind.WindPilot]
