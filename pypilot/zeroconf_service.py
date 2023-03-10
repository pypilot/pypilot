#!/usr/bin/env python
#
#   Copyright (C) 2022 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# make it not blocking with the main server

import socket, struct, fcntl, time, os
import threading

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
            from netifaces import interfaces, ifaddresses
            for interface in interfaces():
                addrs = ifaddresses(interface)
                for i in addrs:
                    if 'addr' in i:
                        addresses.append(i['addr'])
            return addresses
        except Exception as e:
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
        except: # no address for this interface
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
                
            except:
                time.sleep(10)
                continue # try again
            break

        addresses = []
        zeroconf = {}
        info = {}

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
                for address in zeroconf:
                    zeroconf[address].unregister_service(info[address])
                    zeroconf[address].close()
                zeroconf = {}
                info = {}

                addresses = i

                # register addresses
                for address in addresses:
                    #print('zeroconf registering address', address)
                    info[address] = ServiceInfo(
                        "_pypilot._tcp.local.",
                        "pypilot._pypilot._tcp.local.",
                        addresses=[socket.inet_aton(address)],
                        port=DEFAULT_PORT,
                        properties={'version': strversion})
        
                    zeroconf[address] = Zeroconf(ip_version=ip_version, interfaces=[address])
                    try:
                        zeroconf[address].register_service(info[address])
                    except Exception as e:
                        print('zeroconf exception:', e)
                        
            time.sleep(60)


if __name__ == '__main__':
    zc = zeroconf()
    zc.run()
