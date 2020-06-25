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
keypad_pin_numbers = [21, 0, 35, 12, 13, 32, 2, 33, 17, 22]

noisr = False
def make_pin(pin, i, lcd):
    global noisr
    if pin >= 34:
        pin = Pin(pin, Pin.IN)
    else:
        pin = Pin(pin, Pin.IN, Pin.PULL_UP)

    def cbr(pin):
        handle_pin(pin, i, lcd)

    if not noisr:
        try:
            Pin.irq(handler = cbr, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        except:
            print('no Pin.irq!! keypresses will lag')
            noisr = True

    return pin

def handle_pin(pin, i, lcd):
    up = pin()
    if up:
        if lcd.keypad[i]:
            lcd.keypadup[i] = True
        lcd.keypad[i] = False
    else:
        if lcd.keypad[i] != 1:
            lcd.keypad[i] += 1
            
keypad_pins = []
def init(lcd):
    global keypad_pins
    for i in range(len(keypad_pin_numbers)):
        keypad_pins.append(make_pin(keypad_pin_numbers[i], i, lcd))

def poll(lcd):
    for i in range(len(keypad_pins)):
        handle_pin(keypad_pins[i], i, lcd)
