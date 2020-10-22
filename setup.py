#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import sys

if sys.version_info[0] < 3:
    print('pypilot requires python version 3.  python version is', sys.version)
    exit(1)

try:
    from setuptools import setup, Extension
    
except ImportError:
    from distutils.core import setup, Extension

linebuffer_module = Extension('pypilot/linebuffer/_linebuffer',
                        sources=['pypilot/linebuffer/linebuffer.cpp', 'pypilot/linebuffer/linebuffer.i'],
                        extra_compile_args=['-Wno-unused-result'],
                        swig_opts=['-c++']
)

arduino_servo_module = Extension('pypilot/arduino_servo/_arduino_servo',
                        sources=['pypilot/arduino_servo/arduino_servo.cpp', 'pypilot/arduino_servo/arduino_servo_eeprom.cpp', 'pypilot/arduino_servo/arduino_servo.i'],
                        extra_compile_args=['-Wno-unused-result'],
                        swig_opts=['-c++']
)

ugfx_defs = ['-DWIRINGPI']
try:
    import RPi.GPIO
    ugfx_libraries=['wiringPi']
except:
    try:
        import OPi.GPIO
        ugfx_libraries=['wiringPi']
    except:
        print('no RPi.GPIO library for ugfx')
        ugfx_libraries=[]
        ugfx_defs = []

ugfx_module = Extension('pypilot/hat/ugfx/_ugfx',
                        sources=['hat/ugfx/ugfx.cpp',
                                 'hat/ugfx/ugfx.i'],
                        extra_compile_args=['-Wno-unused-result'] + ugfx_defs,
                        libraries=ugfx_libraries,
                        swig_opts=['-c++'] + ugfx_defs
)

import os, os.path
locale_files = []
for walk in os.walk('hat/locale'):
    path, dirs, files = walk
    path = path[len('hat/'):]
    for file in files:
        if file[len(file)-3:] == '.mo':
            locale_files.append(os.path.join(path, file))

from pypilot import version

packages = ['pypilot', 'pypilot/pilots', 'pypilot/arduino_servo', 'ui', 'hat', 'web', 'pypilot/linebuffer', 'hat/ugfx']
try:
    from setuptools import find_packages
    packages = find_packages()
except:
    pass

# ensure all packages are under pypilot
package_dirs = {}
for package in list(packages):
    if not package.startswith('pypilot'):
        packages.remove(package)
        packages.append('pypilot.'+package)
        package_dirs['pypilot.'+package] = package.replace('.', '/')

setup (name = 'pypilot',
       version = version.strversion,
       description = 'pypilot sailboat autopilot',
       license = 'GPLv3',
       author="Sean D'Epagnier",
       url='http://pypilot.org/',
       packages=packages,
       package_dir=package_dirs,
       ext_modules = [arduino_servo_module, linebuffer_module, ugfx_module],
       package_data={'pypilot.hat': ['font.ttf', 'static/*', 'templates/*'] + locale_files,
                     'pypilot.ui': ['*.png', '*.mtl', '*.obj'],
                     'pypilot.web': ['static/*', 'templates/*']},
       entry_points={
           'console_scripts': [
               'pypilot=pypilot.autopilot:main',
               'pypilot_boatimu=pypilot.boatimu:main',
               'pypilot_servo=pypilot.servo:main',
               'pypilot_web=pypilot.web.web:main',
               'pypilot_hat=pypilot.hat.hat:main',
               'pypilot_control=pypilot.ui.autopilot_control:main',
               'pypilot_calibration=pypilot.ui.autopilot_calibration:main',
               'pypilot_client=pypilot.client:main',
               'pypilot_scope=pypilot.ui.scope_wx:main',
               'pypilot_client_wx=pypilot.ui.client_wx:main'
               ]
        }
       )
