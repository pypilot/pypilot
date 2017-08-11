all: ugfx.py splash
	python setup.py build

install: ugfx.py ugfx.cpp ugfx_wrap.cxx
	rm -rf build dist
	sudo python setup.py install

splash: splash.cpp ugfx.cpp
	g++ -Os -o splash splash.cpp ugfx.cpp -O0 -g -lwiringPi -lpthread

ugfx.py ugfx_wrap.cxx: ugfx.i
	swig -c++ -python ugfx.i
