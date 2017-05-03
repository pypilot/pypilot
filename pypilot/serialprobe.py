#!/usr/bin/env python
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import sys, os, time
import json

pypilot_dir = os.getenv('HOME') + '/.pypilot/'

def enumerate_devices(name, bauds):
    devices = []
    by_id = '/dev/serial/by-id'
    if False: #rpi3
        devicesp = ['/dev/ttyS']
    else:
        devicesp = ['/dev/ttyAMA']
    if os.path.exists(by_id):
        for device_path in os.listdir(by_id):
            for baud in bauds:
                devices.append({'path': os.path.join(by_id, device_path), 'baud': baud})
    else:
        devicesp = ['/dev/ttyUSB'] + devicesp

    for devicep in devicesp:
        for i in range(4):
            device_path = devicep + '%d' % i
            for baud in bauds:
                devices.append({'path': device_path, 'baud' : baud})
    return devices

class SerialProbe:
    def __init__(self):
        self.gpsdevices = []
        self.lastworkingdevices = {}
        self.probes = {}

    def lastworkingdevice(self, name):
        if name in self.lastworkingdevices:
            return self.lastworkingdevices[name]

        filename = pypilot_dir + name + 'device'
        try:
            file = open(filename, 'r')
            lastdevice = json.loads(file.readline().rstrip())
            file.close()
        
            # ensure lastdevice defines path and baud here
            path = lastdevice['path']
            baud = lastdevice['baud']
        except:
            lastdevice = False

        #    if os.path.exists(filename):
        #        os.unlink(filename)
        self.lastworkingdevices[name] = lastdevice
        return lastdevice

    def probe(self, name, bauds):
        t0 = time.time()
        if not name in self.probes:
            self.probes[name] = {'probed': [], 'time': 0}
            self.probes[name]['devices'] = False

        if time.time() - self.probes[name]['time'] < 3:
            return False

        probe = self.probes[name]
        if not probe['devices']:
            probe['devices'] = enumerate_devices(name, bauds)
            lastdevice = self.lastworkingdevice(name)
            if lastdevice:
                probe['devices'].insert(0, lastdevice)

        devices = probe['devices']
        probe['time'] = time.time()
        probe['device_probe'] = False # currently probing
        probe['device'] = False # successful

        probe_device = False
        for device in devices:
            exists = os.path.exists(device['path'])
            if not exists:
                for device2 in list(devices):
                    if device2['path'] == device['path']:
                        devices.remove(device2)

                return False

            ok = True
        
            # do not probe another probe's device
            for n in self.probes:
                # device is being probed
                if self.probes[n]['device_probe'] and \
                   self.probes[n]['device_probe']['path'] == device['path']:
                    ok = False
                    break

                # device in use
                #print 'device', probes[n]['device'], device
                if self.probes[n]['device'] and \
                   self.probes[n]['device']['path'] == device['path']:
                    ok = False
                    break

            for d in probe['probed']:
                # device already probed before
                if d == device:
                    ok = False
                    break

            for gpsdevice in self.gpsdevices:
                # device used by gpsd
                try:
                    if not os.path.samefile(gpsdevice, device['path']):
                        continue
                except:
                    pass
                ok = False
                break
                    
            if not ok:
                continue


            import serial
            serial_device = (device['path'], device['baud'])
            failed = False
            try:
                serial.Serial(*serial_device)
                probe_device = device
                break

            except serial.serialutil.SerialException as err:
                if err.args[0] == 16: # device busy, retry later
                    #print 'busy, try again later', device, name
                    pass
                elif err.args[0] == 6: # No such device or address, don't try again
                    for device2 in list(devices):
                        if device2['path'] == device['path']:
                            devices.remove(device2)
                    return False
                else:
                    print 'serial exception', device, name, err
                    # don't try again if ttyS port?

            except IOError:
                print 'io error', serial_device

        if not probe_device:
            #print 'no device for', name
            return False
    
        probe['probed'].append(probe_device)
        probe['device_probe'] = probe_device

        print name, 'probe...', probe_device
        return serial_device

    def probe_success(self, name):
        if self.probes[name]['device']:
            return # already success
        filename = pypilot_dir + name + 'device'
        probe = self.probes[name]
        self.probes[name]['device'] = probe['device_probe']
        probe['device_probe'] = False
        device = probe['device']
        probe['probed'].remove(device) # allowed to probe again if unplugged
        #print 'serialprobe: record', filename, device
        try:
            file = open(filename, 'w')
            file.write(json.dumps(device) + '\n')
            file.close()

        except:
            print 'serialprobe failed to record device', name
