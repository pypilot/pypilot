#!/usr/bin/env python
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import sys, os, time
import json

pypilot_dir = os.getenv('HOME') + '/.pypilot/'

def enumerate_devices(name):
    devices = []
    by_id = '/dev/serial/by-id'
    if False: #rpi3
        devicesp = ['/dev/ttyS']
    else:
        devicesp = ['/dev/ttyAMA']
    if os.path.exists(by_id):
        for device_path in os.listdir(by_id):
            devices.append(os.path.join(by_id, device_path))
    else:
        devicesp = ['/dev/ttyUSB'] + devicesp

    for devicep in devicesp:
        for i in range(4):
            device_path = devicep + '%d' % i
            devices.append(device_path)

    existing_devices = []
    for device in devices:
        if os.path.exists(device):
            existing_devices.append(device)
            
    return existing_devices

class SerialProbe:
    def __init__(self):
        self.gpsdevices = []
        self.lastworkingdevices = {}
        self.probes = {}
        self.devices = []

    def lastworkingdevice(self, name):
        if name in self.lastworkingdevices:
            return self.lastworkingdevices[name]

        filename = pypilot_dir + name + 'device'
        try:
            file = open(filename, 'r')
            lastdevice = json.loads(file.readline().rstrip())
            file.close()
        
            # ensure lastdevice defines path and baud here
            path, bauds = lastdevice['path'], lastdevice['bauds']
        except:
            lastdevice = False

        #    if os.path.exists(filename):
        #        os.unlink(filename)
        self.lastworkingdevices[name] = lastdevice
        return lastdevice

    def probe(self, name, bauds):
        t0 = time.time()
        if not name in self.probes:
            self.probes[name] = {'time': 0, 'devices' : 'none', 'device': False}
        probe = self.probes[name]

        if time.time() - probe['time'] < 3:
            return False

        probe['time'] = time.time()

        device = probe['device']
        if device:
            device['bauds'] = device['bauds'][1:]
            if device['bauds']:
                return device['path'], device['bauds'][0]

        if not self.devices:
            self.devices = enumerate_devices(name)

        if probe['devices'] == 'none':
            probe['devices'] = []
            lastdevice = self.lastworkingdevice(name)
            if lastdevice:
                probe['devices'].append(lastdevice)
            for device in self.devices:
                probe['devices'].append({'path': device, 'bauds': bauds})

        probe['device'] = False
        if not probe['devices']:
            return False
        
        probe_device = probe['devices'][0]
        probe['devices'] = probe['devices'][1:]
        # do not probe another probe's device
        for n in self.probes:
            if self.probes[n]['device'] and \
               self.probes[n]['device']['path'] == probe_device['path']:
                probe['devices'].append(probe_device)
                return False
        probe['device'] = probe_device

        for gpsdevice in self.gpsdevices:
            # device used by gpsd
            try:
                if os.path.samefile(gpsdevice, probe['device']['path']):
                    return False
            except:
                pass
                    
        serial_device = probe['device']['path'], probe['device']['bauds'][0]
        try:
            import serial
            serial.Serial(*serial_device)
        except serial.serialutil.SerialException as err:
            if err.args[0] == 16: # device busy, retry later
                print 'busy, try again later', probe['device'], name
                probe['devices'].append(probe['device'])
            elif err.args[0] == 6: # No such device or address, don't try again
                pass
            else:
                print 'serial exception', device, name, err
                # don't try again if ttyS port?
            probe['device'] = False
        except IOError:
            print 'io error', serial_device
            probe['device'] = False

        if not probe['device']:
            return False
    
        print name, 'probe...', serial_device
        return serial_device

    def probe_success(self, name):
        probe = self.probes[name]
        if 'success' in probe:
            return # already success
        filename = pypilot_dir + name + 'device'
        device = probe['device']
        probe['success'] = True
        #print 'serialprobe: record', filename, device
        try:
            file = open(filename, 'w')
            file.write(json.dumps(device) + '\n')
            file.close()

        except:
            print 'serialprobe failed to record device', name
