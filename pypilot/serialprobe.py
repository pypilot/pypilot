#!/usr/bin/env python
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import sys, os, time
import json

import autopilot

def lastworkingdevice(device, name):
    filename = autopilot.pypilot_dir + name + 'device'
    try:
        file = open(filename, 'r')
        lastdevice = json.loads(file.readline().rstrip())
        file.close()
    except:
        lastdevice = False
        
    if device:
        try:
            file = open(filename, 'w')
            file.write(json.dumps(device) + '\n')
            file.close()

        except:
            print 'serialprobe failed to record device', device
    else:
        if os.path.exists(filename):
            os.unlink(filename)
    return lastdevice


probes = {}

def probe(name, probe_device, bauds):
    global probes

    if not name in probes:
        devices = [] # last working device
        lastdevice = lastworkingdevice(False, name)
        if lastdevice:
            devices.append(lastdevice)

        by_id = '/dev/serial/by-id'
        if os.path.exists(by_id):
            for device_path in os.listdir(by_id):
                for baud in bauds:
                    devices.append((os.path.join(by_id, device_path), baud))
        else:
            devicesp = ['/dev/ttyUSB', '/dev/ttyAMA', '/dev/ttyS']
            for devicep in devicesp:
                for i in range(4):
                    device_path = devicep + '%d' % i
                    if os.path.exists(device_path):
                        for baud in bauds:
                            devices.append((device_path, baud))

        probe = {'devices': devices, 'time': 0}
        probes[name] = probe

    if time.time() - probes[name]['time'] < 3:
        return

    probes[name]['time'] = time.time()
    probes[name]['device_path'] = ''

    devices = probes[name]['devices']
    if len(devices) == 1:
        del probes[name]
    else:
        probes[name]['devices'] = devices[1:]

    device_path = devices[0]

    for n in probes:
        if probes[n]['device_path'] == device_path[0]:
            return False

    sys.stdout.write(name + 'probe... ' + str(device_path))

    try:
        device = probe_device(device_path)
        sys.stdout.flush()
        print ' ok'
        #del probes[name]
        lastworkingdevice(device_path, name)
        time.sleep(.1);
        probes[name]['device_path'] = device_path[0]
        return device
    except:
        print ' failed'

