#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

try:
    from setuptools import setup, Extension
except ImportError:
    from distutils.core import setup, Extension
import os, os.path

locale_files = []
for walk in os.walk('pypilot_lcd/locale'):
    path, dirs, files = walk
    path = path[len('pypilot_lcd/'):]
    for file in files:
        if file[len(file)-3:] == '.mo':
            locale_files.append(os.path.join(path, file))

setup (name = 'pypilot',
       version = '0.1',
       description = 'pypilot sailboat autopilot',
       license = 'GPLv3',
       author="Sean D'Epagnier",
       url='http://pypilot.org/',
       packages=['pypilot', 'pypilot_ui', 'pypilot_lcd', 'pypilot_webapp', 'signalk'],
       package_data={'pypilot_lcd': ['font.ttf'] + locale_files,
                     'pypilot_webapp': ['static/*', 'templates/*']},
#       requires=['flask', 'gevent'], # webapp
       entry_points={
           'console_scripts': [
               'pypilot=pypilot.simple_autopilot:main',
               'pypilot_nmea_bridge=pypilot.nmea_bridge:main',
               'pypilot_webapp=pypilot_webapp.pypilot_webapp:main',
               'pypilot_lcdclient=pypilot_lcd.client:main',
               'pypilot_control=pypilot_ui.autopilot_control:main',
               'signalk_client=signalk.client:main',
               'signalk_scope=signalk.scope:main',
               'signalk_client_wx=signalk.client_wx:main',
               'signalk_scope_wx=signalk.scope_wx:main',
               ]
        }
       )
