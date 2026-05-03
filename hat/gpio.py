#!/usr/bin/env python
#
#   Copyright (C) 2026 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

import os
import time

raspberrypi = False
orangepi = False

try:
    with open('/sys/firmware/devicetree/base/model') as m:
        if 'raspberry pi' in m.read().lower():
            while True:
                try:
                    f = open('/dev/gpiomem', 'w')
                    f.close()
                    break
                except Exception as e:
                    print('waiting for gpiomem...', e)
                time.sleep(1)
            import lgpio
            print('have gpio for raspberry pi')
            raspberrypi = True
except Exception:
    pass

class gpio(object):
    def __init__(self):
        self.keystate = {}
        self.events = []

        from pypilot.nonblockingpipe import NonBlockingPipe
        self.pipe = NonBlockingPipe(str(self), True)

        if orangepi:
            self.pins = [11, 16, 13, 15, 12]
        else:
            self.pins = [17, 23, 27, 22, 18, 5, 6, 26]

        self.lastkeystate = {}
        for p in self.pins:
            self.lastkeystate[p] = False

        self.keystate = 0
        self.keypin = False
        self.lastkeyevent = 0

        if not lgpio:
            return

        self.gpio_handle = lgpio.gpiochip_open(0)

        for pin in self.pins:
            while True:
                try:
                    lgpio.gpio_claim_alert(self.gpio_handle, pin, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)
                    lgpio.gpio_set_debounce_micros(self.gpio_handle, pin, 20000)
                    break
                except RuntimeError:
                    print('failed to open /dev/gpiomem, no permission')
                    # if failed, attempt to give current user privilege if no sudo pw
                    user = os.getenv('USER')
                    os.system('sudo chown ' + user + ' /dev/gpiomem')
                    time.sleep(0.01)
                    
            def cbr(chip, pin, level, timestamp):
                #print("got cbr", chip, pin, level, timestamp)
                #value = GPIO.input(pin)
                #time.sleep(.02)  # workaround buggy gpio
                #self.lastkeystate[pin] = not value
                #self.evalkeys()
                # break from poll by sending to a pipe
                self.pipe[0].send(pin)

            while True:
                try:
                    #GPIO.add_event_detect(pin, GPIO.BOTH, callback=cbr, bouncetime=50)
                    lgpio.callback(self.gpio_handle, pin, lgpio.BOTH_EDGES, cbr)
                    break
                except Exception as e:
                    print('WARNING', e, 'on pin ', pin)

                if not raspberrypi:
                    break
                print('retrying to setup gpio with edge detect.')
                time.sleep(3)

    def poll(self):
        if not lgpio:
            return []

        while self.pipe[1].recv():
            pass

        for p in self.pins:
            value = lgpio.gpio_read(self.gpio_handle, p)
            self.lastkeystate[p] = not value

        self.evalkeys()

        events = self.events
        self.events = []
        return events

    def evalkeys(self):
        pin = ''
        for p in self.pins:
            if self.lastkeystate[p]:
                if pin:
                    pin += '_'
                pin += '%02d' % p

        # once all keys are released reset state
        if self.keypin is False:
            if not pin:
                self.keypin = ''
            return

        if pin == self.keypin: # key repeated limit rate
            if time.monotonic() - self.lastkeyevent < .03:
                return
            self.keystate += 1
        elif len(self.keypin) <= len(pin):  # new key
            self.keypin = pin
            self.keystate = 1
        elif self.keypin:     # any key released, send release code
            self.events.append(('gpio' + self.keypin, 0))
            self.keypin = False  # require all keys to release before new code
            return

        if self.keypin:
            self.lastkeyevent = time.monotonic()
            self.events.append(('gpio' + self.keypin, self.keystate))


def main():
    gp = gpio()
    while True:
        events = gp.poll()
        if events:
            print('events', events)
        time.sleep(.1)

if __name__ == '__main__':
    main()
