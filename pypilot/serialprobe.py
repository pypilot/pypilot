#!/usr/bin/env python
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

from __future__ import print_function
import sys, os, time
import pyjson

pypilot_dir = os.getenv('HOME') + '/.pypilot/'

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
            print('error reading', pypilot_dir + 'serial_ports')
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

def scan_devices():
    devices = []
    #rpi3, orange pi have ttyS, othes have ttyAMA
    devicesp = ['ttyAMA', 'ttyS']
        
    by_id = '/dev/serial/by-id'
    if os.path.exists(by_id):
        for device_path in os.listdir(by_id):
            devices.append(os.path.join(by_id, device_path))

        # identical devices might exist, so also add by path
        by_path = '/dev/serial/by-path'
        if os.path.exists(by_path):
            for device_path in os.listdir(by_path):
                full_path = os.path.join(by_path, device_path)
                realpath = os.path.realpath(full_path)
            # make sure we don't already have it "by-id" 
            have = False
            for path in devices:
                if os.path.realpath(path) == realpath:
                    have = True
            if not have:
                devices.append(full_path)
    else: # do not have by-id and by-path support
        devicesp = ['ttyUSB', 'ttyACM'] + devicesp

    # devicesp are devices that need number after, enumerate them
    for dev in os.listdir('/dev'):
        devicesd = []
        for p in devicesp:
            if dev.startswith(p):
                devicesd.append('/dev/'+dev)
        devices = devicesd + devices

    blacklist_serial_ports = read_blacklist()
    for device in blacklist_serial_ports:
        realpath = os.path.realpath(device)
        for d in devices:
            if os.path.realpath(d.strip()) == realpath:
                devices.remove(d)
    
    allowed_serial_ports = read_allowed()
    if allowed_serial_ports == 'any':
        return devices
    
    allowed_devices = []
    for device in allowed_serial_ports:
        realpath = os.path.realpath(device)
        for d in devices:
            if realpath == os.path.realpath(d):
                allowed_devices.append(d)

    # add any unique serial ports not scanned
    # but listed in serial_ports file to end of list
    for device in allowed_serial_ports:
        realpath = os.path.realpath(device)
        have = False
        for d in allowed_devices:
            if realpath == os.path.realpath(d):
                have = True
        if not have:
            allowed_devices.append(device)
    
    return allowed_devices

devices = 'init'
pyudev = 'init'
monitor = False
starttime = False
pyudevwarning = False

def enumerate_devices():
    global pyudev
    global devices
    global monitor
    global starttime
    if devices == 'init':
        starttime = time.monotonic()
        devices = scan_devices()
        
    if monitor:
        import signal
        t1 = time.monotonic()
        if monitor.poll(0):
            while monitor.poll(0): # flush events
                pass
            devices = scan_devices()
    else:
        # delay monitor slightly to ensure startup speed
        if time.monotonic() > starttime and pyudev == 'init':
            import signal
            try:
                import signal
                # need to temporary disable sigchld while loading pyudev
                cursigchld_handler = signal.getsignal(signal.SIGCHLD)
                signal.signal(signal.SIGCHLD, 0)
                import pyudev
                context = pyudev.Context()
                signal.signal(signal.SIGCHLD, cursigchld_handler)
                monitor = pyudev.Monitor.from_netlink(context)
                monitor.filter_by(subsystem='usb')
            except Exception as e:
                global pyudevwarning
                if not pyudevwarning:
                    print('no pyudev module! will scan usb devices every probe!', e)
                    pyudevwarning = True
                # try pyudev again in 20 seconds if it is delayed loading
                starttime = time.monotonic() + 20
                #pyudev = False
        devices = scan_devices()
    return devices

# reads the file recording the last working
# serial device and baud rate for that use
lastworkingdevices = {}
def lastworkingdevice(name):
    global lastworkingdevices
    if name in lastworkingdevices:
        return lastworkingdevices[name]

    filename = pypilot_dir + name + 'device'
    try:
        file = open(filename, 'r')
        lastdevice = pyjson.loads(file.readline().rstrip())
        file.close()

        # ensure lastdevice defines path and baud here
        lastdevice = lastdevice[0], [lastdevice[1]]
    except:
        lastdevice = False

    lastworkingdevices[name] = lastdevice
    return lastdevice

# called to find a new serial device and baud to try to use
probes = {}
def probe(name, bauds, timeout=5):
    global devices
    global probes
    
    t0 = time.monotonic()
    if not name in probes:
        probes[name] = {'time': 0, 'device': False, 'probe last': True}
    probe = probes[name]

    # prevent probing too often
    if t0 - probe['time'] < timeout:
        probe['device'] = False
        return False
    probe['time'] = t0

    # current device being probed or used, try next baud rate
    if probe['device']:
        probe['bauds'] = probe['bauds'][1:]
        if probe['bauds']:  # there are more bauds to try
            return probe['device'], probe['bauds'][0]

    # try the last working device every other probe
    if probe['probe last']:
        probe['probe last'] = False # next time probe new devices
        last = lastworkingdevice(name)
        if last:
            if 'USB' in last[0] or 'ACM' in last[0]:
                if not os.path.exists(last[0]):
                    lastworkingdevices[name] = False
                    return False
            probe['device'], probe['bauds'] = last
            return probe['device'], probe['bauds'][0]
    probe['probe last'] = True # next time try last working device if this fails

    # find a new device
    #t1 = time.monotonic()
    devices = enumerate_devices()

    #print('enumtime', time.monotonic() - t1, devices)

    # find next device index to probe
    try:
        index = devices.index(probe['lastdevice']) + 1
    except:
        index = 0

    # do not probe another probe's device
    pi = 0
    plist = list(probes)
    while pi < len(plist) and index < len(devices):
        real_path = os.path.realpath(devices[index])
        probe_path = probes[plist[pi]]['device']
        if probe_path and os.path.realpath(probe_path) == real_path:
            index += 1
            pi = 0
        else:
            pi += 1

    # if no more devices, return false to reset, and allow other probes
    if index >= len(devices):
        probe['lastdevice'] = False
        return False

    device = devices[index]
    #print('device', device, devices)
    serial_device = device, bauds[0]

    try:
        try:
            import serial
            serial.Serial
        except Exception as e:
            print('No serial.Serial available')
            print('pip3 uninstall serial')
            print('pip3 install pyserial')
            exit(1)
        serial.Serial(*serial_device)
    except serial.serialutil.SerialException as err:
        arg = err.args[0]
        if type(arg) == type('') and 'Errno ' in  arg:
            arg = int(arg[arg.index('Errno ')+6: arg.index(']')])
        if arg == 16: # device busy, retry later
            print('busy, try again later', serial_device, probe['device'], name)
        elif arg == 6: # No such device or address, don't try again
            devices.remove(device)
        elif arg == 5: # input output error (unusable)
            devices.remove(device)
        elif arg == 2: # No such file or directory
            devices.remove(device)
        else:
            devices.remove(device)
            print('serial exception', serial_device, name, err)
            # don't try again if ttyS port?
            #if device.startswith('/dev/ttyS'):
            #    devices.remove(device)
        serial_device = False
    except IOError:
        print('io error', serial_device)
        devices.remove(device)
        serial_device = False

    probe['device'] = device
    probe['lastdevice'] = probe['device']
    probe['bauds'] = bauds
    #print('probe', name, serial_device)
    return serial_device

# allow reserving gps devices against probing
def reserve(device):
    devices = enumerate_devices()
    #print('prevent serial probing', device)
    i = 0
    while 'reserved%d' % i in probes:
        i+=1
    for dev in devices:
        if os.path.realpath(dev) == os.path.realpath(device):
            probes['reserved%d' % i] = {'device': device}
            break

# called to record the working serial device
def success(name, device):
    global probes
    filename = pypilot_dir + name + 'device'
    print('serialprobe success:', filename, device)
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
