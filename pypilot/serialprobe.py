#!/usr/bin/env python
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import sys, os, time
import pyjson

pypilot_dir = os.getenv('HOME') + '/.pypilot/'

def debug(*args):
    #print(*args)
    pass

def read_config(filename, fail):
    devices = []
    if os.path.exists(pypilot_dir + filename):
        try:
            f = open(pypilot_dir + filename, 'r')
            while True:
                device = f.readline()
                if not device:
                    break
                devices.append(device.strip())
            f.close()
            return devices
        except Exception as e:
            print('error reading', pypilot_dir + filename)
    return fail

blacklist_serial_ports = 'init'
def read_blacklist():
    global blacklist_serial_ports
    if blacklist_serial_ports == 'init':
        blacklist_serial_ports = read_config('blacklist_serial_ports', [])
    return blacklist_serial_ports
    
allowed_serial_ports = 'init'
def read_allowed():
    global allowed_serial_ports
    if allowed_serial_ports == 'init':
        allowed_serial_ports = read_config('serial_ports', 'any')
    return allowed_serial_ports

probes = {}
def new_probe(name):
    global probes
    probes[name] = {'time': 0, 'device': False, 'probelast': True, 'lastdevice': False, 'lastworking': False}

def read_last_working_devices():
    global probes
    for filename in os.listdir(pypilot_dir):
        if filename.endswith('device'):
            name = filename[:-6]
            if name:
                try:
                    file = open(pypilot_dir + filename, 'r')
                    lastdevice = pyjson.loads(file.readline().rstrip())
                    file.close()
                    # ensure lastdevice defines path and baud here
                    if not name in probes:
                        new_probe(name)
                    probes[name]['lastworking'] = lastdevice[0], lastdevice[1]
                except:
                    pass

def scan_devices():
    devices = {}
    devicesp = ['ttyAMA']

    by_id = '/dev/serial/by-id'
    by_path = '/dev/serial/by-path'
    by = by_id
    if os.path.exists(by_id):
        paths = os.listdir(by_id)
        debug('serialprobe scan by-id', paths)

        if os.path.exists(by_path):
            by_path_paths = os.listdir(by_path)
            if len(by_path_paths) > len(paths):
                # if more devices by path than id, then use the paths
                # this allows identical devices and remembers which port
                # they are plugged into to speed up future probing
                print('serial probe found more devices by path')
                paths = by_path_paths
                by = by_path
        
        for device_path in paths:
            full_path = os.path.join(by, device_path)
            realpath = os.path.realpath(full_path)
            devices[full_path] = {'realpath': realpath}
    else: # do not have by-id and by-path support
        devicesp = ['ttyUSB', 'ttyACM'] + devicesp

    # devicesp are devices that need number after, enumerate them
    devgpsdevices = []
    for dev in os.listdir('/dev'):
        devicesd = []
        if dev.startswith('gps'):
            path = '/dev/'+dev
            realpath = os.path.realpath(path)
            devgpsdevices.append(realpath)
            
        for p in devicesp:
            if dev.startswith(p):
                path = '/dev/'+dev
                realpath = os.path.realpath(path)
                for device in devices:
                    if device[1] == realpath:
                        break
                else:
                    devices[path] = {'realpath': realpath}

    for device in list(devices):
        if devices[device]['realpath'] in devgpsdevices:
            print('serialprobe removing gps device', device)
            del devices[device]
                    
    blacklist_serial_ports = read_blacklist()
    for path in blacklist_serial_ports:
        realpath = os.path.realpath(path)
        for device in list(devices):
            if devices[device]['realpath'] == realpath:
                del devices[device]
    
    allowed_serial_ports = read_allowed()
    if allowed_serial_ports == 'any':
        return devices
    
    allowed_devices = {}
    for path in allowed_serial_ports:
        for device in devices:
            realpath = devices[device]['realpath']
            if os.path.realpath(path) == realpath:
                allowed_devices[device] = devices[device]

    # add any unique serial ports not scanned
    # but listed in serial_ports file to end of list
    for path in allowed_serial_ports:
        realpath = os.path.realpath(path)
        for device in allowed_devices:
            if devices[device]['realpath'] == realpath:
                break
        else:
            allowed_devices[path]['realpath'] = realpath
    
    return allowed_devices

devices = {}
gpsdevices = []
enumstate = 'init'

def enumerate_devices():
    global devices
    global enumstate

    t0 = time.monotonic()
    if enumstate == 'init':
        enumstate = {'monitor': False, 'starttime': t0, 'scantime': 0, 'retries': 0, 'pyudevwarning': False}
        devices = {}
        read_last_working_devices()

    if enumstate['monitor']:
        # only scan devices if they change
        ret = enumstate['monitor'].poll(0)
        if ret:
            enumstate['scantime'] = t0
            enumstate['retries'] = 5 # up to 5 retries
            while ret:
                debug('serialprobe pyudev monitor', ret)
                ret = enumstate['monitor'].poll(0) # flush events
        if enumstate['retries'] == 0 or t0 < enumstate['scantime']:
            return False

    else:
        # delay monitor slightly to ensure startup speed
        if t0 > enumstate['starttime'] + 5:
            # try to start pyudev
            import signal
            # need to temporary disable sigchld while loading pyudev
            cursigchld_handler = signal.getsignal(signal.SIGCHLD)
            signal.signal(signal.SIGCHLD, signal.SIG_IGN)
            try:
                import pyudev
                context = pyudev.Context()
                enumstate['monitor'] = pyudev.Monitor.from_netlink(context)
                enumstate['monitor'].filter_by(subsystem='usb')
            except Exception as e:
                # try pyudev/scanning again in 10 seconds if it is delayed loading
                enumstate['starttime'] = time.monotonic() + 10
                if not enumstate['pyudevwarning']:
                    print('no pyudev module! will scan usb devices often!', e)
                    enumstate['pyudevwarning'] = True

            signal.signal(signal.SIGCHLD, cursigchld_handler)

        if t0 < enumstate['scantime']:
            return False
        enumstate['scantime'] = t0 + 20 # scan every 20 seconds

    scanned_devices = scan_devices()
    if enumstate['monitor']:
        prev_devices = {}
        for name in devices:
            prev_devices[name] = {'realpath': devices[name]['realpath']}
        if prev_devices == scanned_devices: # compare path and realpath
            if enumstate['retries'] > 0:
                enumstate['scantime'] += 2 #try again in 2 seconds until we get a change
                enumstate['retries'] -= 1
            return False
        elif enumstate['monitor']:
            debug('serialprobe pyudev found it', devices, scanned_devices)
            enumstate['retries'] = 0
    
    debug('serialprobe scan', scanned_devices)
    # remove devices not scanned
    for device in list(devices):
        if not device in scanned_devices:
            del devices[device]

    # add new devices and set the time the device was added
    for device in scanned_devices:
        if not device in devices:
            devices[device] = scanned_devices[device]
            devices[device]['time'] = t0
    return True

# called to find a new serial device and baud to try to use
def relinquish(name):
    if name in probes:
        probes[name]['device'] = False


def probe(name, bauds, timeout=5):    
    global devices
    global probes
    
    t0 = time.monotonic()

    if enumerate_devices():
        # relinquish any probes that are assigned to devices that no longer exist
        for n, probe in probes.items():
            device = probe['device']
            if device and not device in devices:
                probe['device'] = False
    
    if not name in probes:
        new_probe(name)
    probe = probes[name]

    # current device probed, try next baud rate
    if probe['device']:
        probe['bauds'] = probe['bauds'][1:]
        if probe['bauds']:  # there are more bauds to try
            return probe['device'], probe['bauds'][0]
        probe['device'] = False # relinquish device
    
    if t0 - probe['time'] < timeout:
        # prevent probing too often
        probe['device'] = False
        return False
    debug('serialprobe PROBE', name, probe)

    # try the last working device every other probe
    device_list = list(devices)
    if probe['probelast']:
        probe['time'] = t0
        probe['probelast'] = False # next time probe new devices
        if not probe['lastworking']:
            debug('serialprobe no last working, abort', name)
            return False
        last_device, last_baud = probe['lastworking']
        bauds = [last_baud]
        for index in range(len(device_list)):
            device = device_list[index]
            if device == last_device:
                break
        else: # last device not found
            debug('serialprobe last not found', device_list)
            return False
    else:
        probe['probelast'] = True # next time try last working device if this fails

        # find next device index to probe
        for index in range(len(device_list)):
            device = device_list[index]
            if device == probe['lastdevice']:
                index += 1
                break
        else:
            index = 0

    # do not probe another probe's device
    while index < len(device_list):
        device = device_list[index]
        for p in probes:
            probe_device = probes[p]['device']
            if probe_device == device:
                if not probe['probelast']:
                    debug('serialprobe ret1')
                    return False
                index += 1
                break
        else:
            # do not probe another's last working device if it's less than 10 seconds
            for last_name in probes:
                lastworking = probes[last_name]['lastworking']
                if not lastworking:
                    continue
                last_device, baud = lastworking
                if last_name != name and last_device == device:
                    if t0 - devices[device]['time'] < 10:
                        if not probe['probelast']:
                            debug('serialprobe ret2')
                            return False
                        index += 1
                        break
            else:
                break
    else:
        debug('serialprobe ret3', name)
        probe['lastdevice'] = False
        return False

    if probe['probelast']:
        probe['lastdevice'] = device

    if device == '/dev/ttyAMA0' and name != 'servo':
        debug('serial probe abort', name, 'reserved for servo')
        return False # only let servo have AMA0

    if devices[device]['realpath'] in gpsdevices:
        debug('serial probe abort', name, 'device', device, 'is a gps device')
        return False
    
    probe['device'] = device
    probe['bauds'] = bauds
    
    debug('serial probing', name, device, bauds[0])
    return device, bauds[0]

# reserve gpsd devices against probing
def gpsddevices(devices):
    global gpsdevices
    gpsdevices = []
    for device in devices:
        realpath = os.path.realpath(device)
        gpsdevices.append(realpath)

# called to record the working serial device
def success(name, device):
    global probes
    filename = pypilot_dir + name + 'device'
    print('serialprobe success:', filename, device)
    probes[name]['lastworking'] = device
    try:
        file = open(filename, 'w')
        file.write(pyjson.dumps(device) + '\n')
        file.close()

    except:
        print('serialprobe failed to record device', name)

if __name__ == '__main__':
    print('testing serial probe')
    while True:
        t0 = time.monotonic()
        device = probe('test', [9600], timeout=2)
        if device:
            print('return', device, time.monotonic() - t0)
        time.sleep(1)
