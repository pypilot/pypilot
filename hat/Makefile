MPY_CROSS = /home/sean/MicroPython_ESP32_psRAM_LoBo/MicroPython_BUILD/components/micropython/mpy-cross/mpy-cross
PORT=/dev/ttyUSB0

%.mpy : %.py
	$(MPY_CROSS) $<

SRCS = font.py gpio_esp32.py lcd.py menu.py page.py upy_client.py wifi_esp32.py lcd_esp32.py config_esp32.py
ALL = $(SRCS:.py=.mpy)

all: $(ALL)

upload: $(ALL)
	for f in $(ALL); do echo $$f;  ampy -p $(PORT) put $$f;	done
	ampy -p $(PORT) put boot.py

fonts:
	for f in `ls ~/.pypilot/ugfxfonts`; do echo $$f; ampy -p $(PORT) put ~/.pypilot/ugfxfonts/$$f flash/ugfxfonts; done

clean:
	rm -rf *mpy
