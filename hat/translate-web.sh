pybabel extract -F babel.cfg -o pypilot_hat_web.pot .

function translate() {
    mkdir -p translations/$1/LC_MESSAGES
    touch translations/$1/LC_MESSAGES/messages.po
    pybabel update -i pypilot_hat_web.pot -d translations -l $1
    locale/trans-po.py translations/$1/LC_MESSAGES/messages.po $1
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
