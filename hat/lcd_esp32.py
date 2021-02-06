#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

import time
t0= time.time()
import wifi_esp32

def gettime():
    return time.ticks_us()/1e6

'''

import display

# initialize tft display
tft = display.TFT()
tft.tft_setspeed(4000000)
tft.init(tft.ST7789,bgr=False,rot=tft.PORTRAIT, miso=17,backl_pin=4,backl_on=1, mosi=19, clk=18, cs=5, dc=16, color_bits=tft.COLOR_BITS16, splash=False)
tft.setwin(52,40,240, 320)
#tft.set_bg(0xff00)
#tft.clear()
'''

t1= gettime()

import page
t2= gettime()
from lcd import LCD
t3= gettime()
import gpio_esp32
t4= gettime()

lcd = LCD(False)
period = .1
sleeptime = gettime()

import machine, micropython
#machine.freq(80000000)

vbatt = machine.ADC(machine.Pin(34))
vbatt.atten(3) # only one that works!!

gpio_esp32.init(lcd)

t5= gettime()
print ('loaded', t5-t0, ':',t1-t0, t2-t1, t3-t2, t4-t3, t5-t4)

sleepmode = 0
while True:
    import gc
    gc.collect()
    #micropython.mem_info()

    v = vbatt.read() * 0.0017728937728937728
    if not lcd.battery_voltage:
        lcd.battery_voltage = v
    lp = .02
    lcd.battery_voltage = (1-lp)*lcd.battery_voltage + lp*v

    gpio_esp32.poll(lcd)
    if any(list(map(lambda key : key.count, lcd.keypad))):
        sleeptime = gettime()
        if sleepmode:
            #tft.backlight(True)
            pass
        if sleepmode > 1:
            machine.freq(240000000)
        sleepmode = 0
    
    t0 = gettime()
#    try:
    if 1:
        lcd.poll()
#    except Exception as e:
#        print('lcd poll failed', e)
    t1 = gettime()
    gpio_esp32.poll(lcd)
    t2 = gettime()
        
    if 0 and gettime() - sleeptime > 10:
        #print('sleep blank screen')
        #tft.backlight(False)
        #esp.sleep_type(esp.SLEEP_MODEM) # SLEEP_LIGHT
        sleepmode = 1

    if 0 and gettime() - sleeptime > 20:
        if wifi_esp32.station.isconnected():
            print('sleep wifi off')
            pass
        wifi_esp32.station.active(False)
        machine.freq(80000000)
        sleepmode = 2

    #if wifi_esp32.station.isconnected():
    wifi_esp32.poll(lcd.client)

    if 0 and gettime() - sleeptime > 30:
        print('sleep power down')
        machine.deepsleep()

    t3 = gettime()
    dt = t3-t0
    s = period - dt
    if s <= .01:
        s = .01
    #print('sleep ', dt, t1-t0, t2-t1, t3-t2, s*100/(t3-t0+s), '%')

    time.sleep(s)
