#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

try:
    from setuptools import setup, Extension
except ImportError:
    from distutils.core import setup, Extension

linebuffer_module = Extension('_linebuffer',
                        sources=['signalk/linebuffer/linebuffer.cpp', 'signalk/linebuffer/linebuffer.i'],
                        extra_compile_args=['-Wno-unused-result'],
                        swig_opts=['-c++']
)

arduino_servo_module = Extension('_arduino_servo',
                        sources=['pypilot/arduino_servo/arduino_servo.cpp', 'pypilot/arduino_servo/arduino_servo.i'],
                        extra_compile_args=['-Wno-unused-result'],
                        swig_opts=['-c++']
)



try:
    import RPi.GPIO
    ugfx_libraries=['wiringPi']
except:
    try:
        import OPi.GPIO
        ugfx_libraries=['wiringPi']
    except:
        print 'no wiring library for ugfx'
        ugfx_libraries=[]

ugfx_module = Extension('_ugfx',
                        sources=['lcd/ugfx/ugfx.cpp',
                                 'lcd/ugfx/ugfx.i'],
                        extra_compile_args=['-Wno-unused-result'],
                        libraries=ugfx_libraries,
                        swig_opts=['-c++']
)

import os, os.path
locale_files = []
for walk in os.walk('lcd/locale'):
    path, dirs, files = walk
    path = path[len('lcd/'):]
    for file in files:
        if file[len(file)-3:] == '.mo':
            locale_files.append(os.path.join(path, file))


            
setup (name = 'pypilot',
       version = '0.1',
       description = 'pypilot sailboat autopilot',
       license = 'GPLv3',
       author="Sean D'Epagnier",
       url='http://pypilot.org/',
       packages=['pypilot', 'pypilot/arduino_servo', 'ui', 'lcd', 'webapp', 'signalk', 'signalk/linebuffer', 'lcd/ugfx'],
       ext_modules = [arduino_servo_module, linebuffer_module, ugfx_module],
#       py_modules = ['pypilot/arduino_servo', 'signalk/linebuffer/linebuffer'],
       package_data={'lcd': ['font.ttf'] + locale_files,
                     'ui': ['*.png', '*.mtl', '*.obj'],
                     'webapp': ['static/*', 'templates/*']},
#       requires=['flask', 'gevent'], # webapp
#       dependency_links	= ['https://github.com/adafruit/Adafruit_Nokia_LCD/tarball/master#egg=Adafruit-Nokia-LCD-0.1.0'],
#       install_requires	= ['Adafruit-Nokia-LCD>=0.1.0'],
       entry_points={
           'console_scripts': [
               'pypilot=pypilot.basic_autopilot:main',
               'pypilot_boatimu=pypilot.boatimu:main',
               'pypilot_servo=pypilot.servo:main',
               'pypilot_webapp=webapp.webapp:main',
               'pypilot_lcdclient=lcd.client:main',
               'pypilot_control=ui.autopilot_control:main',
               'pypilot_calibration=ui.autopilot_calibration:main',
               'signalk_client=signalk.client:main',
               'signalk_scope=signalk.scope:main',
               'signalk_client_wx=signalk.client_wx:main',
               'signalk_scope_wx=signalk.scope_wx:main',
               ]
        }
       )
