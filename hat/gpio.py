#!/usr/bin/env python
#
#   Copyright (C) 2019 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

orangepi = False
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

class gpio(object):
    def __init__(self):
        self.keystate = {}
        self.events = {}

        if orangepi:
            self.pins = [11, 16, 13, 15, 12]
        else:
            self.pins = [17, 23, 27, 22, 18, 5, 6]

        if not GPIO:
            return
        
        if orangepi:
            for pin in self.pins:
                cmd = 'gpio -1 mode ' + str(pin) + ' up'
                os.system(cmd)
            GPIO.setmode(GPIO.BOARD)
        else:
            GPIO.setmode(GPIO.BCM)

        for pin in self.pins:
            try:
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                pass
            except RuntimeError:
                os.system("sudo chown tc /dev/gpiomem")
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                    
            def cbr(channel):
                # more accurate timings?
                pass

            try:
                GPIO.add_event_detect(pin, GPIO.BOTH, callback=cbr, bouncetime=20)
            except Exception as e:
                print('WARNING', e)        

    def poll(self):                
        for pini in range(len(self.pins)):
            pin = self.pins[pini]
            value = True

            if False:
                f = open('/sys/class/gpio/gpio%d/value' % pin)
                a = f.readline()
                value = bool(int(a))
            else:
                if GPIO:
                     value = GPIO.input(pin)

            if not value and self.keypad[pini] > 0:
                self.keypad[pini] += 1
                self.events['gpio%d'%pin] = (True, self.keypad[pini])

            if pini in self.keystate and self.keystate[pini] != value:
                if value:
                    self.keypadup[pini] = True
                else:
                    self.keypad[pini] = 1
                self.events['gpio%d'%pin] = [not value, self.keypad[pini]]

            self.keystate[pini] = value

    def read(self):
        if not self.events:
            return False

        key = list(self.events)[0]
        return [key] + self.events[key]

