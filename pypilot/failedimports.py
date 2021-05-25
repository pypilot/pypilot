#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from __future__ import print_function
from gettext import gettext as _

print(_('pypilot failed to import required modules.  Did you forget to run sudo python3 setup.py install?'))

import sys
if sys.version_info[0] < 3:
    print('pypilot requires python version 3.  python version is', sys.version)
    print('I will now attempt to re-run the command using python 3')
    cmd = 'python3 '
    for arg in sys.argv:
        cmd += arg + ' '
    import os
    os.system(cmd)
exit(1)
