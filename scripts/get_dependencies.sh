#!/bin/bash

sudo apt install -y python-gps python-serial libpython-dev python-numpy python-scipy swig python-pil python-flask python-gevent-websocket python-opengl python-pip

sudo pip install setuptools
sudo pip install ujson
sudo pip install pyudev
sudo pip install pyglet
sudo pip install pywavefront

cd ~
git clone https://github.com/seandepagnier/RTIMULib2
cd RTIMULib2/Linux/python
python setup.py build
sudo python setup.py install
