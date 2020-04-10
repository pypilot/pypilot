#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time

try:
    import RPi.GPIO as GPIO
    print('have gpio for raspberry pi')

except ImportError:
    try:
        import OPi.GPIO as GPIO
        orangepi = True
        print('have gpio for orange pi')
    except:
        print('No gpio available')
        GPIO = None

class buzzer(object):
    def __init__(self, config):
        self.alarmcount = 0
        self.lastalarmtime = 0
        self.needbeep = 0
        self.pwm = False
        try:
            self.config = config['buzzer']
            pin = self.config['pin']
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(pin, GPIO.OUT)
            self.pwm = GPIO.PWM(pin, 800)
        except Exception as e:
            print('failed to configure buzzer', e)

    def poll(self):
        if not self.pwm:
            return
        if self.alarmcount:
            if self.alarmcount % 2: # odd
                self.pwm.ChangeFrequency(1200)
                self.pwm.start(50)
            else:
                self.pwm.stop()
            self.alarmcount-=1
        elif self.needbeep:
            self.pwm.ChangeFrequency(600)
            self.pwm.start(50)
            self.needbeep -= 1
        else:
            self.pwm.stop()
            
    def beep(self):
        self.needbeep = 1

    def alarm(self):
        t = time.monotonic()
        if t - self.lastalarmtime < 3:
            return
        self.lastalarmtime = t
        if not self.alarmcount:
            self.alarmcount = 7

def main():
    c = {'buzzer': {'pin':12}}
    b = buzzer(c)

    b.beep()
    x = 0
    while True:
        b.poll()
        time.sleep(.1)
        if x > 10:
            b.alarm()
        x+=1

if __name__ == '__main__':
    main()
