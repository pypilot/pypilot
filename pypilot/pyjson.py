#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

# ujson is declared in setup.py as an optional `performance` extra; when it
# is not installed the stdlib `json` module is the intended fallback. Don't
# warn, since the fallback is a supported configuration.
try:
    import ujson as _json
except ImportError:
    import json as _json

loads, dumps = _json.loads, _json.dumps
