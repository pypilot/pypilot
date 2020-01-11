#!/bin/bash

function translate() {
    cp pypilot_lcdclient.pot $language/LC_MESSAGES/pypilot_lcdclient.po
    python apertium-po.py $language/LC_MESSAGES/pypilot_lcdclient.po $mode
    /usr/bin/msgfmt --check -o $language/LC_MESSAGES/pypilot_lcdclient.mo $language/LC_MESSAGES/pypilot_lcdclient.po
}

language=es
mode=en-es
translate

language=es
mode=en-es/es-fr
translate
