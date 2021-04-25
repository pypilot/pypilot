#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from machine import Pin

keypad_pin_numbers = [33, 25, 12, 13, 26, 15, 2, 27, 0, 35]

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
            pin.irq(handler = cbr, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        except:
            print('no Pin.irq!! keypresses will lag')
            noisr = True
    return pin

def handle_pin(pin, i, lcd):
    key = lcd.keypad[i]
    v = not pin()
    
    if v:
        lcd.keypress = True
    key.update(v)
            
keypad_pins = []
def init(lcd):
    global keypad_pins
    for i in range(len(keypad_pin_numbers)):
        keypad_pins.append(make_pin(keypad_pin_numbers[i], i, lcd))

def poll(lcd):
    if noisr:
        for i in range(len(keypad_pins)):
            handle_pin(keypad_pins[i], i, lcd)
