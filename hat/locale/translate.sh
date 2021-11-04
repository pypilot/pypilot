#!/bin/bash

function translate() {
    if [ ! -f "$1/LC_MESSAGES/pypilot_hat.mo" ]; then
        mkdir -p $1/LC_MESSAGES
        cp -n pypilot_hat.pot $1/LC_MESSAGES/pypilot_hat.po
        msgmerge -N -U $1/LC_MESSAGES/pypilot_hat.po pypilot_hat.pot
        ./trans-po.py $1/LC_MESSAGES/pypilot_hat.po $1
        /usr/bin/msgfmt --check -o $1/LC_MESSAGES/pypilot_hat.mo $1/LC_MESSAGES/pypilot_hat.po
    fi
}

translate ca # Catalan
translate da # Danish
translate de # German
translate el # Greek
# translate eo # Esperanto
translate es # Spanish
translate fi # Finnish
translate fr # French
translate it # Italian
translate nl # Dutch
translate no # Norwegian
translate pl # Polish
translate pt # Portuguese
translate ru # Russian
translate sv # Swedish
