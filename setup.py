#!/usr/bin/env python
#
#   Copyright (C) 2021 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

# This setup.py only handles C/SWIG extension builds.
# All metadata, dependencies, and entry points are in pyproject.toml.

from setuptools import Extension, setup
from setuptools.command.install import install

# Workaround: ensure SWIG-generated Python wrappers exist before
# setuptools scans for packages.
class build_ext_first(install):
    def run(self):
        self.run_command("build_ext")
        return install.run(self)


linebuffer_module = Extension(
    "pypilot/linebuffer/_linebuffer",
    sources=[
        "pypilot/linebuffer/linebuffer.cpp",
        "pypilot/linebuffer/linebuffer.i",
    ],
    extra_compile_args=["-Wno-unused-result"],
    swig_opts=["-c++"],
)

arduino_servo_module = Extension(
    "pypilot/arduino_servo/_arduino_servo",
    sources=[
        "pypilot/arduino_servo/arduino_servo.cpp",
        "pypilot/arduino_servo/arduino_servo_eeprom.cpp",
        "pypilot/arduino_servo/arduino_servo.i",
    ],
    extra_compile_args=["-Wno-unused-result"],
    swig_opts=["-c++"],
)

ret = os.system('pkg-config --cflags libgpiod')
if ret == 0:
    print('detected libgpiod')
    ugfx_libraries=['gpiod']
    ugfx_defs = ['-DGPIOD']
else:
    print('no gpiod library for ugfx')
    ugfx_libraries=[]
    ugfx_defs = []

ugfx_module = Extension(
    "pypilot/hat/ugfx/_ugfx",
    sources=["hat/ugfx/ugfx.cpp", "hat/ugfx/ugfx.i"],
    extra_compile_args=["-Wno-unused-result"] + ugfx_defs,
    libraries=ugfx_libraries,
    swig_opts=["-c++"] + ugfx_defs,
)

spireader_module = Extension('pypilot/hat/spireader/_spireader',
                             sources=['hat/spireader/spireader.cpp',
                                      'hat/spireader/spireader.i'],
                             extra_compile_args=['-Wno-unused-result'],
                             libraries=ugfx_libraries,
                             swig_opts=['-c++'])

ext_modules = [arduino_servo_module, linebuffer_module, ugfx_module]
if spireader_module:
    ext_modules.append(spireader_module)

setup(
    ext_modules=ext_modules,
    cmdclass={"install": build_ext_first},
)
