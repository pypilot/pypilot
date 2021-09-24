pybabel extract -F babel.cfg -o pypilot_web.pot .

function translate() {
    pybabel update -i pypilot_web.pot -d translations -l $1
    ../hat/locale/trans-po.py translations/$1/LC_MESSAGES/messages.po $1
}

translate ca # Catalan
translate da # Danish
translate de # German
translate el # Greek
## translate eo # Esperanto
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


pybabel compile -d translations
