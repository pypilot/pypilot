#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from machine import Pin
#22, 17, 2, 33, 
keypad_pin_numbers = [21, 0, 35,39, 12 , 32, 2]

def make_pin(pin):
    if pin >= 34:
        return Pin(pin, Pin.IN)
    return Pin(pin, Pin.IN, Pin.PULL_UP)

keypad_pins = list(map(make_pin, keypad_pin_numbers))

#nudge = [(make_pin(0), -1), (make_pin(35), 1)]
nudge = [(make_pin(12), -1), (make_pin(13), 1)]
def poll(lcd):
    anykey = False
    for i in range(len(keypad_pins)):
        up = keypad_pins[i].value()
        lcd.keypadup[i] = up and lcd.keypad[i]
        if not up:
            lcd.keypad[i] += 1
            anykey = True
        else:
            lcd.keypad[i] = 0

    # force motor movement from these keys
    for pin, d in nudge:
        if not pin.value():
            lcd.client.set('servo.command', d)
            anykey = True
    return anykey
