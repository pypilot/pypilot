#!/usr/bin/env python
#
#   Copyright (C) 2026 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

import sys

e = sys.stdout.encoding.lower()
if e.startswith('utf'):
    import gettext
    import os
    print('loading translations: terminal encoding:', e)
    locale_d = os.path.abspath(os.path.dirname(__file__)) + '/locale'
    gettext.translation('pypilot', locale_d, fallback=True).install()
else:
    # no translations
    print('no translations, encoding', e)
    import builtins
    builtins.__dict__['_'] = lambda x : x
