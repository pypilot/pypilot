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
    keypad_pin_numbers = [0, 19, 25, 32, 34, 33, 35, 39, 37, 38]
else:
    keypad_pin_numbers = [33, 25, 12, 13, 27, 15, 32, -1, 0, 35]

power_down_pin = 26

noisr = False
def make_pin(pin, i, lcd):
    global noisr
    if pin >= 34 or pin == 0:
        pin = Pin(pin, Pin.IN)
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
    if i == 0 or i >= 35:
        v = not v
    if not v:
        lcd.keypress = True
    key.update(v)
            
keypad_pins = []
power_down_pin = False

def powerdown():
    power_down_pin.mode(Pin.OUT)
    power_down_pin.low()

def init(lcd):
    global keypad_pins
    global power_down_pin
    power_down_pin = Pin(power_down_pin, Pin.IN, Pin.PULL_UP)
    
    for i in range(len(keypad_pin_numbers)):
        pini = keypad_pin_numbers[i]
        if pini >= 0:
            keypad_pins.append(make_pin(pini, i, lcd))

def poll(lcd):
    if noisr:
        for i in range(len(keypad_pins)):
            handle_pin(keypad_pins[i], i, lcd)
