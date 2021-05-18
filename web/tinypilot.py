#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# determine if we are on tinypilot if piCore is in uname -r

import tempfile, subprocess, os
temp = tempfile.mkstemp()
p=subprocess.Popen(['uname', '-r'], stdout=temp[0], close_fds=True)
p.wait()
f = os.fdopen(temp[0], 'r')
f.seek(0)
kernel_release = f.readline().rstrip()
f.close()

tinypilot = 'piCore' in kernel_release
# javascript uses lowercase bool, easier to use int
tinypilot = 1 if tinypilot else 0
