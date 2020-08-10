#!/bin/bash

# gpio13 is write protect of eeprom
echo 13 > /sys/class/gpio/export || echo ok
echo out > /sys/class/gpio/gpio13/direction
dd if=/dev/zero ibs=1k count=4 of=blank.eep
sudo eepflash.sh -w -f=blank.eep -t=24c32
eepmake eeprom_settings.txt pypilot.eep -c pypilot_hat.conf
sudo eepflash.sh -w -f=pypilot.eep -t=24c32
echo in > /sys/class/gpio/gpio13/direction
