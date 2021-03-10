#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

print('pypilot failed to import required modules.  Did you forget to run sudo python3 setup.py install?')

import sys
if sys.version_info[0] < 3:
    print ('pypilot requires python version 3.  python version is', sys.version)
exit(1)
