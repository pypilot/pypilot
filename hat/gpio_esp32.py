#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  
from machine import Pin, TouchPad
import gc
import time
from time import sleep
if gc.mem_free() > 1e6:  # larger ttgo display
    keypad_pin_numbers = [34, 36, 26, 33, 0, 35, 39, -1, 37, 38] #AUTO, MENU, SMALL_PORT, SMALL_STARBOARD, SELECT, BIG_PORT, BIG_STARBOARD, TACK, NUDGE_PORT, NUDGE_STARBOARD
    keypad_pullup = [37, 38, 39]                                 #add the pins that have pullups(LOW activ) ALL other will have pulldowns(HIGH active)
    keypad_touch = []                                            #add the pins that are esp32 touchpins and are used without pushbuttons
    power_down_pin_number = None
else:
    keypad_pin_numbers = [33, 25, 12, 13, 27, 15, 32, -1, 0, 35] #AUTO, MENU, SMALL_PORT, SMALL_STARBOARD, SELECT, BIG_PORT, BIG_STARBOARD, TACK, NUDGE_PORT, NUDGE_STARBOARD
    keypad_pullup = [0, 35]                                     #add the pins that have pullups(LOW activ) ALL other will have pulldowns(HIGH active)
    keypad_touch = []                                      #add the pins that are esp32 touchpins and are used without pushbuttons
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
            pin.irq(handler=cbr, trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING)
        except:
            print('no Pin.irq!! keypresses will lag')
            noisr = True
    return pin

def handle_pin(pin, i, lcd):
    
    if pin == -5 :
        key = lcd.keypad[i]
        lcd.keypress = True
        v = 1
        key.update(v)
    elif pin == -10 :
        #You need to toogle for the button to be activated
        key = lcd.keypad[i]
        lcd.keypress = True
        v = 0
        #print('update key touch', i, v)
        key.update(v)
    else:
        key = lcd.keypad[i]
        v = pin()
        if keypad_pin_numbers[i] in keypad_pullup:
            v = not v
        if not v:
            lcd.keypress = True
            #print('update key', i, v)
        key.update(v)
                
        
        
keypad_pins = []
keypad_pins_wake = []
index_pins_for_touch = []
Threshold = []
Threshold_ratio = []

def powerdown():
    if power_down_pin_number:
        Pin(power_down_pin_number, Pin.OUT).off()

def init(lcd):
    global keypad_pins
    global Threshold
    global Threshold_ratio
    
    if power_down_pin_number:
        Pin(power_down_pin_number, Pin.IN, Pin.PULL_UP)
    
    for i in range(len(keypad_pin_numbers)):
        pini = keypad_pin_numbers[i]
        if pini >= 0:
            if not pini in keypad_touch:
                pin = make_pin(pini, i, lcd)
                keypad_pins.append(pin)
                if not pini in keypad_pullup:
                    keypad_pins_wake.append(pin)
            else:
                index_pins_for_touch.append(i)        #get the index of the touchpins
    
    #calculate and store the threshold ratio for each touch pin. It takes 1.2s for each pin but it will wait for tinypilot
    #to boot because the are powered at the same time so there is enough time
    for i in range(len(index_pins_for_touch)):
        touch = TouchPad(Pin(keypad_pin_numbers[index_pins_for_touch[i]]))
        for x in range(12):
            Threshold.append(touch.read())
            sleep(.1)
        Threshold_ratio.append(sum(Threshold) // len(Threshold))
        Threshold = []
  
    
def poll(lcd):
    if noisr:
        for i in range(len(keypad_pins)):
            handle_pin(keypad_pins[i], i, lcd)
            
    #Currently there is no support for interrupts from touchpads in micropython(as i know) so it is a bit laggy but saves
    #pcb space and complexibility
    for i in range(len(index_pins_for_touch)):
        touch = TouchPad(Pin(keypad_pin_numbers[index_pins_for_touch[i]]))
        ratio = touch.read()  / Threshold_ratio[i]
        if .10 < ratio < .95:
            #print(touch.read())
            #print(ratio)
            handle_pin(-5, index_pins_for_touch[i], lcd)
            while .10 < ratio < .95:
                ratio = touch.read()  / Threshold_ratio[i]
                sleep(0.1)
            handle_pin(-10, index_pins_for_touch[i], lcd)
            
    
    
    
