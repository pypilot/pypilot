#!/bin/sh

translate() {
    if [ ! -f "$1/LC_MESSAGES/pypilot_hat.po" ]; then
        mkdir -p $1/LC_MESSAGES
        cp pypilot_hat.pot $1/LC_MESSAGES/pypilot_hat.po
    fi
    msgmerge -N -U $1/LC_MESSAGES/pypilot_hat.po pypilot_hat.pot
    ./trans-po.py $1/LC_MESSAGES/pypilot_hat.po $1
    msgfmt --check -o $1/LC_MESSAGES/pypilot_hat.mo $1/LC_MESSAGES/pypilot_hat.po
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
