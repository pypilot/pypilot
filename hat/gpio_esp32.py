#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from machine import Pin
import gc
if gc.mem_free() > 1e6:  # larger ttgo display
    keypad_pin_numbers = [34, 36, 26, 33, 0, 35, 39, -1, 37, 38]
    keypad_pullup = [37, 38, 39]
    power_down_pin_number = None
else:
    keypad_pin_numbers = [33, 25, 12, 13, 27, 15, 32, -1, 0, 35]
    keypad_pullup = [0, 35]
    power_down_pin_number = 26


noisr = False
def make_pin(pin, i, lcd):
    global noisr
    if pin in keypad_pullup:
        pin = Pin(pin, Pin.IN, Pin.PULL_UP)
    else:
        pin = Pin(pin, Pin.IN, Pin.PULL_DOWN)

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
    v = pin()
    if keypad_pin_numbers[i] in keypad_pullup:
        v = not v
    if not v:
        lcd.keypress = True
    print('update key', i, v)
    key.update(v)
            
keypad_pins = []
keypad_pins_wake = []

def powerdown():
    if power_down_pin_number:
        Pin(power_down_pin_number, Pin.OUT).off()

def init(lcd):
    global keypad_pins
    if power_down_pin_number:
        Pin(power_down_pin_number, Pin.IN, Pin.PULL_UP)
    
    for i in range(len(keypad_pin_numbers)):
        pini = keypad_pin_numbers[i]
        if pini >= 0:
            pin = make_pin(pini, i, lcd)
            keypad_pins.append(pin)
            if not i in keypad_pullup:
                keypad_pins_wake.append(pin)

def poll(lcd):
    if noisr:
        for i in range(len(keypad_pins)):
            handle_pin(keypad_pins[i], i, lcd)
