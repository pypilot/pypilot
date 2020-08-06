#!/bin/bash

function translate() {
    cp pypilot_hat.pot $language/LC_MESSAGES/pypilot_hat.po
    ./apertium-po.py $language/LC_MESSAGES/pypilot_hat.po $mode
    /usr/bin/msgfmt --check -o $language/LC_MESSAGES/pypilot_hat.mo $language/LC_MESSAGES/pypilot_hat.po
}

language=es
mode=en-es
translate

language=fr
mode=en-es/es-fr
translate
