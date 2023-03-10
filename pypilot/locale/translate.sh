#!/bin/sh

# python3 setup.py extract_messages -o pypilot/locale/pypilot.pot

function translate() {
    mkdir -p $1/LC_MESSAGES
    if [ ! -e $1/LC_MESSAGES/pypilot.po ]; then
        cp pypilot.pot $1/LC_MESSAGES/pypilot.po
    fi
    msgmerge -N -U $1/LC_MESSAGES/pypilot.po pypilot.pot
    ./trans-po.py $1/LC_MESSAGES/pypilot.po $1
    msgfmt --check -o $1/LC_MESSAGES/pypilot.mo $1/LC_MESSAGES/pypilot.po
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
