#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

# setup.py declares python_requires='>=3.6', so if this module is
# reached we are already on Python 3. No need for print_function
# futures or a re-exec to python3.
print(_('pypilot failed to import required modules.  Did you forget to run sudo python3 setup.py install?'))
exit(1)
