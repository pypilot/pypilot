#!/usr/local/bin/python
#
#   Copyright (C) 2016 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# automatically probe serial ports for gps informing gpsd

import time, sys, socket, os

class GpsProbe():
    def __init__(self):
        # probe last known gps device first,
        # this could be saved between sessions
        self.lastgpsdevice = ''

    def connect(self):
        while True: # connection to gpsd loop
            try:
                import gps
                self.gpsd = gps.gps(mode=gps.WATCH_ENABLE) #starting the stream of info
                self.gpsd.next() # flush initial message
                self.gpsd.activated = False
                return

            except:
                time.sleep(3)

    def probe(self):
        if not os.system('timeout -s KILL 5 gpsctl 2> /dev/null'):
            return True

        # try to probe all possible usb devices
        devicesp = ['/dev/gps', '/dev/ttyUSB', '/dev/ttyAMA', '/dev/ttyS']
        devices = [self.lastgpsdevice]
        for devicep in devicesp:
            for i in range(4):
                devices.append(devicep + '%d' % i)

        for device in devices:
            if not os.path.exists(device):
                continue
            if not os.system('timeout -s KILL 5 gpsctl -f ' + device + ' 2> /dev/null'):
                os.environ['GPSD_SOCKET'] = '/tmp/gpsd.sock'
                os.system('gpsdctl add ' + device)
                print('GPS found: ' + device)
                self.lastgpsdevice = device
                return True
            sys.stdout.flush()

        return False

    def verify(self):
        while True:
            try:
                result = self.gpsd.next()
                if 'devices' in result:
                    activated = len(result['devices']) > 0
                    if activated != self.gpsd.activated:
                        print('GPS ' + ('' if activated else 'de') + 'activated')
                    self.gpsd.activated = activated
                    break
                    
            except StopIteration:
                print('GPS lost gpsd')
                return

        if not self.gpsd.activated:
            if self.probe():
                self.gpsd.activated = True
                print('GPS probe success')

        activated = self.gpsd.activated
        del self.gpsd
        if activated:
            # once activated gpsd normally can find the device
            # if it comes and goes.. but keep trying in case gpsd restarts
            time.sleep(60)
        else:
            time.sleep(15)


def main():
    gpsprobe = GpsProbe()
    try:
        while True:
            gpsprobe.connect()
            gpsprobe.verify()

    except KeyboardInterrupt:
        print('Keyboard interrupt, gpsprobe exit')


if __name__ == "__main__":
    main()
