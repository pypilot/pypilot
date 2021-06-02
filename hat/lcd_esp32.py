#!/usr/bin/env python
#
#   Copyright (C) 2021 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

# TODO:  fix loading slow,  make more responsive,  make power down work


import time
import wifi_esp32

# running 60-90mA
#idletimeout = 10      # 15mA
#sleeptimeout = 10     # 700uA
#powerofftimeout = 10  # 6.5uA
idletimeout = 120      # 15mA  (wakes up faster)
sleeptimeout = 600     # 700uA
powerofftimeout = 3600  # 6.5uA (can only wake from auto key)

def gettime():
    return time.ticks_ms()/1e3
t0= gettime()

t1= gettime()
import page
t2= gettime()
from lcd import LCD
t3= gettime()
import gpio_esp32
t4= gettime()

lcd = LCD(False)
period = .25
sleeptime = gettime()

import machine, micropython
rtc = machine.RTC()
rtc_memory = rtc.memory().decode()

if rtc_memory == 'deepsleep':
    sleeptime -= idletimeout/2
    #lcd.screen.backlight = False;

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
    if lcd.keypress:
        lcd.keypress = False
        rtc_memory = 'keypress'
        rtc.memory('keypress')
        sleeptime = gettime()
        if sleepmode:
            machine.freq(240000000)
            wifi_esp32.enable()
            for k in lcd.keypad:
                k.up = False
                k.count = 0
                k.down = 0
            lcd.client.host = lcd.host
            lcd.client.disconnect()
            lcd.poll()
            lcd.screen.backlight = True;
        sleepmode = 0
    
    t0 = gettime()
    lcd.poll()
            
    t1 = gettime()
    gpio_esp32.poll(lcd)
    t2 = gettime()

    sleepdt = gettime() - sleeptime
    if sleepdt < 0: # work around ticks wrapping
        sleeptime = 0

    if sleepmode == 0:
        if sleepdt > idletimeout and lcd.battery_voltage < 4.2:
            print('sleep blank screen')
            lcd.screen.backlight = False;
            print('sleep wifi off')
            wifi_esp32.disable()
            lcd.client.host = False
            machine.freq(20000000)
            sleepmode = 1
    elif sleepmode == 1:
        if sleepdt > idletimeout + sleeptimeout:
            if rtc_memory == 'deepsleep':
                print('sleep power down')
                rtc.memory('powerdown')
                gpio_esp32.powerdown()
                # if we get here, give up
                rtc_memory = 'powerdown'
            else:
                print('sleep deep sleep')
                wake = gpio_esp32.keypad_pins[:7]
                import esp32
                esp32.wake_on_ext1(pins = tuple(wake), level= esp32.WAKEUP_ANY_HIGH)
                rtc.memory('deepsleep')
                machine.deepsleep(powerofftimeout * 1000)

    #if wifi_esp32.station.isconnected():
    wifi_esp32.poll(lcd.client)
    
    t3 = gettime()
    dt = t3-t0
    if dt < 0:
        s = .1;
    else:
        if sleepmode:
            s = 1 - dt
        else:
            s = period - dt
        if s <= .01:
            s = .01
        elif s > 1:
            s = 1
        
    time.sleep(s)
