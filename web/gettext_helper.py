#!/usr/bin/env python
#
#   Copyright (C) 2026 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

# Lightweight gettext wrapper that replaces flask_babel for pypilot's web UIs.

import gettext as _gettext
import os

from flask import has_request_context, request

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_LOCALE_DIR = os.path.join(_THIS_DIR, 'translations')
_DOMAIN = 'messages'

try:
    LANGUAGES = sorted(os.listdir(_LOCALE_DIR))
except OSError:
    LANGUAGES = []

_translations = {}
for _lang in LANGUAGES:
    try:
        _translations[_lang] = _gettext.translation(
            _DOMAIN, _LOCALE_DIR, languages=[_lang], fallback=False)
    except OSError:
        # .mo not compiled for this language; fall through to NullTranslations
        pass

_null = _gettext.NullTranslations()


def _load_pot_static_strings():
    # Collect msgids that appear under "#: static" reference blocks in
    # pypilot_web.pot, mirroring the loop that lived inline in web.py.
    # Used by the index template to feed static strings to the JS layer.
    out = []
    path = os.path.join(_THIS_DIR, 'pypilot_web.pot')
    try:
        with open(path) as f:
            in_static = False
            for line in f:
                if line.startswith('#: static'):
                    in_static = True
                elif not line.strip():
                    in_static = False
                elif in_static and line.startswith('msgid "'):
                    s = line.strip()[7:-1]
                    if s:
                        out.append(s)
    except OSError:
        pass
    return out


translations = _load_pot_static_strings()


def _pick_translation(config):
    lang = None
    if config and config.get('language') and config['language'] != 'default':
        lang = config['language']
    elif has_request_context():
        lang = request.accept_languages.best_match(LANGUAGES)
    return _translations.get(lang, _null)


def load(app, config=None):
    # Install a request-scoped `_` translator on the Flask app and return
    # it so the caller can also bind it at module scope (web.py uses
    # bare `_("...")` calls in route handlers).
    def _(s):
        return _pick_translation(config).gettext(s)
    app.jinja_env.globals.update(_=_)
    return _
