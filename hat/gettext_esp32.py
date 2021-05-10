#!/usr/bin/env python
#
#   Copyright (C) 2021 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

class tobject(object):
    def __init__(self, path):
        self.d = {}
        if not path:
            return
        f = open(path)
        id = None
        while True:
            l = f.readline()
            if not l:
                break
            if l.startswith('msgid "'):
                id = l[7:-2]
            elif l.startswith('msgstr "') and id:
                str = l[8:-2]
                self.d[id] = str

    def gettext(self, id):
        if id in self.d:
            return self.d[id]
        return id

def translation(name, path, languages, fallback):
    if languages[0] == 'en':
        return tobject(None)
    return tobject(path + '/' +languages[0] + '/LC_MESSAGES/' + name + '.po')
