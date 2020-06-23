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

address = ''
def connect():
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

import time
connected = False, time.time()
connect()

def poll(client):
    global connected
    isconnected = station.isconnected()
    if connected[0] == isconnected: # no change
        if not isconnected and time.time() - connected[1] > 8:
            print('wifi timeout, reconnecting wifi', time.time() - connected[1])
            connect()
        connected = isconnected, time.time()
        return isconnected
    connected = isconnected, time.time()
    if connected:
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
