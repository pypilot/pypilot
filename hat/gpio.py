#!/usr/bin/env python
#
#   Copyright (C) 2026 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import os, time, signal

#import threading
#import gpiod
gpiod = False

class gpio(object):
    def __init__(self):
        self.keystate = {}
        self.events = []

        from pypilot.nonblockingpipe import NonBlockingPipe
        self.pipe = NonBlockingPipe(str(self), True)
        
        #if orangepi:
        #    self.pins = [11, 16, 13, 15, 12]
        self.pins = [17, 23, 27, 22, 18, 5, 6, 26]

        self.lastkeystate = {}
        for p in self.pins:
            self.lastkeystate[p] = False

        self.keystate = 0
        self.keypin = False
        self.lastkeyevent = 0

        if not gpiod:
            return

        config = {}
        for pin in self.pins:
            config[pin] = gpiod.LineSettings(direction=gpiod.line.Direction.INPUT, edge_detection=gpiod.line.Edge.BOTH, bias=gpiod.line.Bias.PULL_UP)
        self.request = gpiod.request_lines("/dev/gpiochip0", consumer="keys", config=config)

        self.thread_running = True
        self.thread = threading.Thread(target=self.thread_main, daemon=True)
        self.thread.start()

    def thread_main(self):
        while self.thread_running:
            # This blocks waiting for GPIO edge events.
            if not self.request.wait_edge_events(timeout=1.0):
                continue

            for ev in self.request.read_edge_events():
                gpio = ev.line_offset
                #value = self.request.get_value(gpio)
                #state = 1 if value == Value.ACTIVE else 0
                time.sleep(.03)
                self.pipe[0].send(gpio) # wake up poll for gpio

    def poll(self):
        if not gpiod:
            return []

        while True:
            pin = self.pipe[1].recv()
            if not pin:
                break

            value = self.request.get_value(pin)
            self.lastkeystate[pin] = not value

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
