#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import network    
station = network.WLAN(network.STA_IF)  # client, not AP
from page import gettime
import time
connected = False, gettime()
enabled = True

address = ''
def connect():
    print('wifi timeout, reconnecting wifi', gettime() - connected[1])

    import config_esp32
    global address

    config = config_esp32.read_config()
    essid = config['essid']
    psk = str(config['psk'])
    address = config['address']

    print('wifi connecting to', essid)
    station.active(False)
    station.active(True) # enable wifi
    if psk:
        print('connect to', essid, psk)
        station.connect(essid, psk)
    else:
        station.connect(essid)

connect()

def enable():
    global enabled
    if not connected[0]:
        connect()
    enabled = True

def disable():
    global enabled
    station.active(False)
    enabled = False
    

def poll(client):
    global connected
    isconnected = station.isconnected()
    if connected[0] == isconnected: # no change
        if enabled and not isconnected and gettime() - connected[1] > 8:
            connected = isconnected, gettime()
            connect()
        return isconnected
    connected = isconnected, gettime()
    if isconnected:
        if address:
            host = address
        else:
            addrs = station.ifconfig()
            print('wifi connection success', addrs)
            host = addrs[3]

        if client.host != host:
            print('wifi connecting to pypilot at', host)
            client.disconnect()
        client.host = host
    else: # disconnected
        print('wifi disconnected')
        client.disconnect()
    return connected
