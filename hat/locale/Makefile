translations: pypilot_lcdclient.pot
	./translate.sh

pypilot_lcdclient.pot: ../client.py
	/usr/bin/xgettext --from-code=iso-8859-1 --force-po --package-name=pypilot_lcdclient --package-version="" --output=$@ --keyword=_ --width=80 --files-from=POTFILES.in
	sed s/charset=CHARSET/charset=UTF-8/1 -i $@
