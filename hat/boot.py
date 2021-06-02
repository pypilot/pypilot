#!/usr/bin/env python3
#
#   Copyright (C) 2021 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from config_esp32 import config

try:
    import lcd_esp32
except Exception as e:
    print('exception', e)
    print('reboot')
    import machine
    machine.reset()



