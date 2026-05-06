#!/usr/bin/env python
#
#   Copyright (C) 2022 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

# make it not blocking with the main server

import fcntl
import os
import socket
import struct
import threading
import time

DEFAULT_PORT = 23322
from version import strversion

retries = 0

# creating a socket is somehow slow...
zerosocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def get_local_addresses():
    global retries
    addresses = []
    if retries:
        try:
            from netifaces import ifaddresses, interfaces
            for interface in interfaces():
                addrs = ifaddresses(interface)
                for i in addrs:
                    if 'addr' in i:
                        addresses.append(i['addr'])
            return addresses
        except Exception:
            #print('zeroconf service fallback to socket address')
            retries -= 1

    interfaces = os.listdir('/sys/class/net')
    for interface in interfaces:
        try:
            addresses.append(socket.inet_ntoa(fcntl.ioctl(
                zerosocket.fileno(),
                0x8915,  # SIOCGIFADDR
                struct.pack('256s', bytes(interface[:15], 'utf-8'))
            )[20:24]))
        except OSError: # no address for this interface
            pass

    return addresses

class zeroconf(threading.Thread):
    #def __del__(self):
    #self.close()

    def run(self):
        # wait until zeroconf is available (tinypilot booting)
        while True:
            try:
                # register zeroconf service
                from zeroconf import IPVersion, ServiceInfo, Zeroconf

            except ImportError:
                time.sleep(10)
                continue # try again
            break

        addresses = []
        zeroconf = None
        info = None

        #ip_version = IPVersion.All
        #ip_version = IPVersion.V6Only
        ip_version = IPVersion.V4Only

        while True:
            t=time.time()
            i = get_local_addresses()
            #print("t", time.time()-t)
            if i != addresses:
                print('zeroconf addresses', i, len(i))
                # close addresses
                if zeroconf:
                    if info:
                        zeroconf.unregister_service(info)
                    zeroconf.close()

                addresses = i

                # register addresses
                #print('zeroconf registering address', address)
                info = ServiceInfo(
                    "_pypilot._tcp.local.",
                    "pypilot._pypilot._tcp.local.",
                    addresses=[socket.inet_aton(a) for a in addresses],
                    port=DEFAULT_PORT,
                    properties={'version': strversion})

                zeroconf = Zeroconf(ip_version=ip_version)#, interfaces=[address])
                try:
                    zeroconf.register_service(info)
                except Exception as e:
                    print('zeroconf exception type:', type(e).__name__)
                    print('zeroconf exception repr:', repr(e))
                    import traceback
                    traceback.print_exc()
                    print('info/addresses:', info, addresses)
                    addresses = []
                    break # maybe try again?

            time.sleep(60)


if __name__ == '__main__':
    zc = zeroconf()
    zc.run()
