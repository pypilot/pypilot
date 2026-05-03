#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.


print(_('pypilot failed to import required modules.  Did you forget to run sudo python3 setup.py install?'))

# pyproject.toml declares requires-python>=3.9, so reaching this module
# guarantees we are already on Python 3. The previous py2 re-exec branch
# is unreachable and has been removed.
exit(1)
