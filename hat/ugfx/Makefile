all: splash

install: splash
	strip splash
	sudo cp splash /opt/splash

.PHONY: pypilot_version.h

pypilot_version.h:
	python3 generate_version.py

splash: splash.cpp ugfx.cpp pypilot_version.h
	g++ -O0 -o splash splash.cpp ugfx.cpp -g -lpthread -lwiringPi -DWIRINGPI
