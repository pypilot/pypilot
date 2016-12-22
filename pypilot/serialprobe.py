#!/usr/bin/env python
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import sys, os, time
import json

import autopilot

gpsdevices = []

def lastworkingdevice(name):
    filename = autopilot.pypilot_dir + name + 'device'
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

    return lastdevice

probes = {}

def probe(name, bauds):
    global probes

    devices = []
    lastdevice = lastworkingdevice(name)
    if lastdevice:
        if os.path.exists(lastdevice['path']):
            devices.append(lastdevice)

    by_id = '/dev/serial/by-id'
    devicesp = ['/dev/ttyAMA', '/dev/ttyS']
    if os.path.exists(by_id):
        for device_path in os.listdir(by_id):
            for baud in bauds:
                devices.append({'path': os.path.join(by_id, device_path), 'baud': baud})
    else:
        print 'warning: /dev/serial/by-id not found'
        print ' this may delay initialization'
        devicesp = ['/dev/ttyUSB'] + devicesp

    for devicep in devicesp:
        for i in range(4):
            device_path = devicep + '%d' % i
            if os.path.exists(device_path):
                for baud in bauds:
                    devices.append({'path': device_path, 'baud' : baud})

    if not name in probes:
        probes[name] = {'probed': [], 'time': 0}

    if time.time() - probes[name]['time'] < 3:
        return False

    probe = probes[name]
    probe['time'] = time.time()
    probe['device_probe'] = False # currently probing
    probe['device'] = False # successful

    if len(devices) == 0:
        return False

    probe_device = False
    #print 'name', name, devices
    for device in devices:
        # do not probe another probe's device
        ok = True
        for n in probes:
            # device is being probed
            if probes[n]['device_probe'] and probes[n]['device_probe']['path'] == device['path']:
                ok = False
                break

            # device in use
            #print 'device', probes[n]['device'], device
            if probes[n]['device'] and probes[n]['device']['path'] == device['path']:
                ok = False
                break

        for d in probe['probed']:
            # device already probed before
            if d == device:
                ok = False
                break

        for gpsdevice in gpsdevices:
            # device used by gpsd
            if os.path.samefile(gpsdevice, device['path']):
                ok = False
                break
            
        if ok:
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
                    probe['probed'].append(device)
                else:
                    print 'serial exception', device, name, err
                    # don't try again if ttyS port

            except IOError:
                print 'io error', serial_device

        
    if not probe_device:
        return False
    
    probe['probed'].append(probe_device)
    probe['device_probe'] = probe_device

    print name, 'probe...', probe_device
    return serial_device

def probe_success(name):
    if probes[name]['device']:
        return # already success
    filename = autopilot.pypilot_dir + name + 'device'
    probe = probes[name]
    probes[name]['device'] = probe['device_probe']
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

def gpsdevices(devices):
    global gpsdevices
    gpsdevices = devices
