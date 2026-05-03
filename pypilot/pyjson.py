#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

try:
    import gettext_loader
except ImportError:
    from . import gettext_loader

# ujson is declared in pyproject.toml under the `optimize` extra. When it
# isn't installed, the stdlib json module is the documented fallback, so
# the warning print on ImportError has been dropped.
try:
    import ujson as _json
except ImportError:
    import json as _json

loads, dumps = _json.loads, _json.dumps
