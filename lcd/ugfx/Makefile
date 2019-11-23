all: splash

install: splash
	sudo cp splash /usr/local/bin/pypilot_splash
	strip /usr/local/bin/pypilot_splash

splash: splash.cpp ugfx.cpp
	g++ -O0 -o splash splash.cpp ugfx.cpp -g -lpthread -lwiringPi -DWIRINGPI

