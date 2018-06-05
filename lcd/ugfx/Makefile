all: splash

install: splash
	sudo cp splash /usr/local/bin/pypilot_splash
	strip /usr/local/bin/pypilot_splash

splash: splash.cpp ugfx.cpp
	g++ -Os -o splash splash.cpp ugfx.cpp -g -lwiringPi -lpthread

