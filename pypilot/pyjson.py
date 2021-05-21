#!/usr/bin/env python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

try:
    import ujson
    loads, dumps = ujson.loads, ujson.dumps
except Exception as e:
    from gettext import gettext as _
    print(_('WARNING: python ujson library failed, parsing will consume more cpu'), e)
    import json
    loads, dumps = json.loads, json.dumps
