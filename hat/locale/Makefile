translations: pypilot_hat.pot
	./translate.sh

pypilot_hat.pot: ../page.py ../menu.py
	/usr/bin/xgettext --from-code=iso-8859-1 --force-po --package-name=pypilot_hat --package-version="" --output=$@ --keyword=_ --width=80 --files-from=POTFILES.in
	sed s/charset=CHARSET/charset=UTF-8/1 -i $@

clean:
	find | grep mo$ | xargs rm
