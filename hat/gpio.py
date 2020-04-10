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
        self.events = []

        if orangepi:
            self.pins = [11, 16, 13, 15, 12]
        else:
            self.pins = [17, 23, 27, 22, 18, 5, 6, 26]

        for pin in self.pins:
            self.keystate[pin] = 0

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
                print('failed to open /dev/gpiomem, no permission')
                # if failed, attempt to give current user privilege if no sudo pw
                user = os.getenv('USER')
                os.system('sudo chown ' + user + '/dev/gpiomem')
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                    
            def cbr(pin):
                self.evalkey(pin, GPIO.input(pin))

            try:
                GPIO.add_event_detect(pin, GPIO.BOTH, callback=cbr, bouncetime=20)
            except Exception as e:
                print('WARNING', e)        

    def poll(self):                
        for pin in self.pins:
            value = True

            if False:
                f = open('/sys/class/gpio/gpio%d/value' % pin)
                a = f.readline()
                value = bool(int(a))
            else:
                if GPIO:
                    value = GPIO.input(pin)

            self.evalkey(pin, value)

    def evalkey(self, pin, value):
        if value:
            if self.keystate[pin]:
                self.keystate[pin] = 0
            else:
                return
        else:
            self.keystate[pin] += 1
        self.events.append(['gpio%d'%pin, self.keystate[pin]])
        
