#!/usr/bin/env python
#
#   Copyright (C) 2021 1Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  


# This script checks for dependencies of pypilot, and installs the informing user

import os, sys

class dep(object):
    def __init__(self, name):
        self.name = name

class py_dep(dep):
    def __init__(self, name, pip_only=False):
        self.pip_only = pip_only
        super(py_dep, self).__init__(name)

    def test(self):
        if self.pip_only:
            return False

        remap = {'pil': 'PIL',
                 'gevent-websocket': 'geventwebsocket',
                 'flask-socketio': 'flask_socketio',
                 'flask-babel': 'flask_babel',
                 'python-socketio': 'socketio',
                 'opengl': 'OpenGL'}

        name = remap[self.name] if self.name in remap else self.name
        try:
            __import__(name)
        except Exception as e:
            print('failed to import', self.name, ' ')
            return False
        return True

    def install(self):
        # first try via apt (faster and resumable)
        apt_name = 'python3-' + self.name

        if self.pip_only: # force pip install
            os.system('sudo apt remove ' + apt_name)
            ret = True
        else:
            ret = os.system('sudo apt install -y ' + apt_name)

        if ret:
            print('failed to install via apt, trying with pip', self.name)
            if self.name == 'pil':
                name = 'pillow'
            elif self.name == 'flask-socketio':
                name = 'flask-socketio==5'
            else:
                name = self.name
            ret = os.system('python3 -m pip install ' + name)

            if ret:
                print('failed to install dependency')
                return False
        return True

class sys_dep(dep):
    def __init__(self, name):
        super(sys_dep, self).__init__(name)

    def test(self):
        self.test = lambda : True # return true next time
        return False

    def install(self):
        ret = os.system('sudo apt install -y ' + self.name)
        if ret:
            return False
        return True

class rpi_dep(sys_dep):
    def __init__(self, name):
        super(rpi_dep, self).__init__(name)

    def test(self):
        try:
            f = open('/sys/firmware/devicetree/base/model')
            pi = 'Raspberry Pi' in f.readline()
            f.close()
            if pi:
                return super(rpi_dep, self).test()
        except:
            pass
        return True

class RTIMULIB2_dep(dep):
    def __init__(self):
        super(RTIMULIB2_dep, self).__init__('RTIMULIB2')

    def test(self):
        try:
            import RTIMU
        except Exception as e:
            print('failed to import', self.name)
            return False

        from importlib_metadata import version
        v = version('RTIMULIb').split('.')
        n = (int(v[0])*1000 + int(v[1]))*1000 + int(v[2])
        if n < 8000001:
            return False

        return True

    def install(self):
        return os.system('. scripts/install_rtimulib.sh') == 0

class data_dep(dep):
    def __init__(self):
        super(data_dep, self).__init__('data')

    def test(self):
        return os.path.exists('ui/compass.png')

    def install(self):
        return os.system('. scripts/install_data.sh') == 0

class subsystem(object):
    def __init__(self, name, info, deps):
        self.name = name
        self.info = info
        self.deps = deps
        self.summary = 'SUCCESS'

    def install(self):
        allok = True
        for dep in self.deps:
            sys.stdout.write('checking for ' + dep.name + '...')
            if dep.test():
                print('done')
            else:
                print(dep.name, 'not found')
                if not dep.install():
                    print('failed to install', dep.name)
                    self.summary = 'failed to install ' + dep.name
                elif dep.test():
                    print('install dependency', dep.name, 'success')
                else:
                    print('dependency failed to install', dep.name)
                    self.summary = 'failed to detect after installing ' + dep.name

    def result(self):
        return self.name + ' ' + self.info + ':' + self.summary

subsystems = []
def ss(*cargs):
    subsystems.append(subsystem(*cargs))
    
# autopilot dependencies (required): RTIMULIB2 python3-serial libpython3-dev python3-numpy python3-scipy swig
#                        (recommended): python3-ujson python3-pyudev python3-zeroconf

ss('autopilot', 'core autopilot or imu-only mode',
   [RTIMULIB2_dep(), py_dep('serial'), py_dep('numpy'), py_dep('scipy'), sys_dep('libpython3-dev'), sys_dep('swig')])

# dependencies not required but reduce cpu usage considerably
ss('optimize', '(recommended) core autopilot operations',
   [py_dep('ujson'), py_dep('pyudev')])

# signalk dependencies: python3-zerconf python3-requests python3-websocket
ss('signalk', 'communicate with signalk-node-server distributed with openploter',
   [py_dep('zeroconf'), py_dep('requests'), py_dep('websocket')])
        
# hat dependencies: python3-pil (or pillow)
ss('hat', 'SPI lcd keypad, and remote control interface',
   [py_dep('pil'), rpi_dep('wiringpi')])

# web dependencies: python3-flask python3-gevent-websocket
ss('web', 'web browser control',
   [py_dep('flask'), py_dep('gevent-websocket'), py_dep('python-socketio'), py_dep('flask-socketio', pip_only = True), py_dep('flask-babel')])

# client dependencies (viewers control applications): python3-wxgtk4.0
# optional: python3-opengl python3-pyglet pywavefront
ss('python_gui', 'python scripts for control and configuration',
   [sys_dep('python3-wxgtk4.0'), py_dep('opengl'), py_dep('pyglet'), py_dep('pywavefront')])

ss('data', 'data files used by various pypilot components',
   [data_dep()])


if os.path.basename(os.path.abspath(os.curdir)) != 'pypilot':
    print('please run this script from the pypilot directory')
    exit(1)

for s in subsystems:
    s.install()
print('')
print('')
print('summary of pypilot dependencies')
for s in subsystems:
    r = s.result()
    print(r)
print('')
print('')

f = open('deps', 'w')
for s in subsystems:
    r = s.result()
    f.write(r + '\n')
f.close()
