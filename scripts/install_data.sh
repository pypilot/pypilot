#!/bin/sh

# put pypilot data into current directory assuming it is pypilot directory

result=${PWD##*/}
if [ $result != 'pypilot' ]
then
   echo "Please run this script from the pypilot directory"
   exit 1
fi

CURDIR=`pwd`
cd /tmp
git clone --depth 1 https://github.com/pypilot/pypilot_data
mv -rv pypilot_data/* $CURDIR
rm -rf pypilot_data
cd $CURDIR
