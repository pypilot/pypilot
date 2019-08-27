#!/usr/bin/env python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

"""
setup.py file for ugfx
"""

from distutils.core import setup, Extension

libraries=[]
try:
    import RPi.GPIO
    libraries=['wiringPi']
except:
    pass
try:
    import OPi.GPIO
    libraries=['wiringPi']
except:
    pass
print('libraries', libraries)

ugfx_module = Extension('_ugfx',
                        sources=['ugfx_wrap.cxx', 'ugfx.cpp'],
                        extra_compile_args=['-Wno-unused-result'],
                        libraries=libraries
)

setup (name = 'ugfx',
       version = '0.1',
       description = """micro graphics library for linux framebuffer""",
       ext_modules = [ugfx_module],
       py_modules = ['ugfx']
)
