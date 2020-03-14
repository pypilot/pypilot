#!/bin/sh

# RTIMULIB2 must be the git version from https://github.com/seandepagnier/RTIMULib2

git clone --depth=1 https://github.com/seandepagnier/RTIMULib2
cd RTIMULib2/Linux/python/
sudo python3 setup.py install
