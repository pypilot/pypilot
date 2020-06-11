#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import wifi_esp32
import display

# initialize tft display
tft = display.TFT()
tft.tft_setspeed(4000000)
tft.init(tft.ST7789,bgr=False,rot=tft.PORTRAIT, miso=17,backl_pin=4,backl_on=1, mosi=19, clk=18, cs=5, dc=16, color_bits=tft.COLOR_BITS16, splash=False)
tft.setwin(52,40,240, 320)
#tft.set_bg(0xff00)
#tft.clear()

import page
print('page')
from lcd import LCD
print('lcd')
import gpio_esp32
import time
print ('loaded')

lcd = LCD(False)
period = .1
sleeptime = time.time()

import machine
#machine.freq(80000000)

vbatt = machine.ADC(34)
vbatt.atten(3) # only one that works!!

while True:
    v = vbatt.read()
    lcd.battery_voltage = v/4095.0 * 2.0 * 3.3 * 1.1
    #print('batt', lcd.battery_voltage)
    t0 = time.time()
    lcd.poll()
    t1 = time.time()
    if gpio_esp32.poll(lcd):
        sleeptime = time.time()
        tft.backlight(True)
    t2 = time.time()
        
    wifi_esp32.poll(lcd.client)
    t3 = time.time()
    
    #print('dt', t1-t0, t2-t1, t3-t2)
    dt = t3-t0
    if dt < period:
        time.sleep(period - dt)
    if time.time() - sleeptime > 40:
        tft.backlight(False)
        #esp.sleep_type(esp.SLEEP_MODEM) # SLEEP_LIGHT
        time.sleep(.1)
