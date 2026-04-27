#!/bin/sh

# python3 setup.py extract_messages -o pypilot/locale/pypilot.pot

python -c 'import polib' 2> /dev/null
NO_POLIB=$?

if [ ! -z $NO_POLIB ]; then
    echo 'cannot translate without polib'
fi

translate() {
    if [ ! -f "$1/LC_MESSAGES/pypilot.po" ]; then
        mkdir -p $1/LC_MESSAGES
        cp pypilot.pot $1/LC_MESSAGES/pypilot.po
    fi
    msgmerge -N -U $1/LC_MESSAGES/pypilot.po pypilot.pot
    if [ -z $NO_POLIB ]; then
        ./trans-po.py $1/LC_MESSAGES/pypilot.po $1
    fi
    msgfmt --check -o $1/LC_MESSAGES/pypilot.mo $1/LC_MESSAGES/pypilot.po
}

translate ca # Catalan
translate da # Danish
translate de # German
translate el # Greek
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
