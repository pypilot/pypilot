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

def _interface_is_up(name):
    # skip loopback and down interfaces: Zeroconf registration on these fails,
    # leaks instances every 60s, and eventually breaks the network stack (#272)
    try:
        with open('/sys/class/net/' + name + '/operstate') as f:
            return f.read().strip() == 'up'
    except OSError:
        return False

def get_local_addresses():
    global retries
    addresses = []
    if retries:
        try:
            from netifaces import ifaddresses, interfaces
            for interface in interfaces():
                if not _interface_is_up(interface):
                    continue
                addrs = ifaddresses(interface)
                for i in addrs:
                    if 'addr' in i:
                        addresses.append((interface, i['addr']))
            return addresses
        except Exception:
            #print('zeroconf service fallback to socket address')
            retries -= 1

    interfaces = [i for i in os.listdir('/sys/class/net') if _interface_is_up(i)]
    for interface in interfaces:
        try:
            addresses.append((interface, socket.inet_ntoa(fcntl.ioctl(
                zerosocket.fileno(),
                0x8915,  # SIOCGIFADDR
                struct.pack('256s', bytes(interface[:15], 'utf-8'))
            )[20:24])))
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
        zeroconf = {}
        info = None
        event = threading.Event()

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
                    for i in zeroconf:
                        zeroconf[i].unregister_all_services()
                        zeroconf[i].close()
                    zeroconf = {}

                addresses = i

                for address in addresses:
                    # register addresses
                    #print('zeroconf registering address', address)
                    info = ServiceInfo(
                        '_pypilot._tcp.local.',
                        #"pypilot._pypilot._tcp.local.",
                        f'pypilot-{address[0]}._pypilot._tcp.local.',
                        addresses=[socket.inet_aton(address[1])],
                        port=DEFAULT_PORT,
                        properties={'version': strversion},
                        server=f'pypilot-{address[0]}.local.',
                    )

                    print(f'zeroconf[{address[0]}] = Zeroconf(ip_version={ip_version}, interfaces=[{address[1]}])')
                    zeroconf[address[0]] = Zeroconf(ip_version=ip_version, interfaces=[address[1]])
                    try:
                        zeroconf[address[0]].register_service(info)
                    except Exception as e:
                        print('zeroconf exception type:', type(e).__name__)
                        print('zeroconf exception repr:', repr(e))
                        import traceback
                        traceback.print_exc()
                        print('info/addresses:', info, addresses)
                        addresses = []
                        break # maybe try again?

            event.wait(timeout=60)  # dont do sleep here, it blocks the zeroconf threads, it is better to use threading event wait

if __name__ == '__main__':
    zc = zeroconf()
    zc.run()
