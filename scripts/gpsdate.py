#!/usr/local/bin/python
#
#   Copyright (C) 2017 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# automatically set system clock to gps time if available

import os, sys, time
import gps

while True:
    while True:
        try:
            gpsd = gps.gps(host='piCore', mode=gps.WATCH_ENABLE) #starting the stream of info
            break
        except:
            time.sleep(3)

    while True:
        try:
            gpsd.next()
        except KeyboardInterrupt:
            exit(1)
        except:
            break

        if len(gpsd.utc):
            date, t = gpsd.utc[:-5].split('T')
            print 'Setting date to gps time', date, t
            sys.stdout.flush()
            os.system('date -u -s "' + date + ' ' + t + '"')
            time.sleep(3*24*60*60) # sync again in 3 days
