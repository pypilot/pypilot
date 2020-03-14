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
    probes[name] = {'time': time.monotonic(), 'device': False, 'probelast': True, 'lastdevice': False, 'lastworking': False}

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
                    probes[name]['lastworking'] = lastdevice[0], [lastdevice[1]]
                except:
                    pass

def scan_devices():
    devices = {}
    devicesp = ['ttyAMA']

    by_id = '/dev/serial/by-id'
    if os.path.exists(by_id):
        paths = os.listdir(by_id)
        debug('serialprobe scan by-id', paths)

        by_path = '/dev/serial/by-path'
        if os.path.exists(by_path):
            by_path = os.listdir(by_path)
            if len(by_path) > len(paths):
                # if more devices by path than id, then use the paths
                # this allows identical devices and remembers which port
                # they are plugged into to speed up future probing
                print('serial probe found more devices by path')
                paths = by_path
        
        for device_path in paths:
            full_path = os.path.join(by_id, device_path)
            realpath = os.path.realpath(full_path)
            devices[full_path] = {'realpath': realpath}
    else: # do not have by-id and by-path support
        devicesp = ['ttyUSB', 'ttyACM'] + devicesp

    # devicesp are devices that need number after, enumerate them
    for dev in os.listdir('/dev'):
        devicesd = []
        for p in devicesp:
            if dev.startswith(p):
                path = '/dev/'+dev
                realpath = os.path.realpath(path)
                for device in devices:
                    if device[1] == realpath:
                        break
                else:
                    devices[path] = {'realpath': realpath}

    blacklist_serial_ports = read_blacklist()
    for path in blacklist_serial_ports:
        realpath = os.path.realpath(path)
        for device in devices:
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
enumstate = 'init'

def enumerate_devices():
    global devices
    global enumstate

    t0 = time.monotonic()
    if enumstate == 'init':
        enumstate = {'monitor': False, 'starttime': t0, 'retries': 0, 'pyudevwarning': False}
        devices = {}
        read_last_working_devices()
        return False

    if enumstate['monitor']:
        # only scan devices if they change
        ret = enumstate['monitor'].poll(0)
        if ret:
            enumstate['starttime'] = t0
            enumstate['retries'] = 5 # up to 5 retries
            while ret:
                #print('serialprobe pyudev monitor', ret)
                ret = enumstate['monitor'].poll(0) # flush events
        if enumstate['retries'] == 0 or t0 < enumstate['starttime']:
            return False

    else:
        # delay monitor slightly to ensure startup speed
        if t0 < enumstate['starttime']:
            return False

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
            if not enumstate['pyudevwarning']:
                print('no pyudev module! will scan usb devices every probe!', e)
                enumstate['pyudevwarning'] = True
            # try pyudev/scanning again in 20 seconds if it is delayed loading
            enumstate['starttime'] = time.monotonic() + 20

        signal.signal(signal.SIGCHLD, cursigchld_handler)

    scanned_devices = scan_devices()
    prev_devices = {}
    for name in devices:
        prev_devices[name] = {'realpath': devices[name]['realpath']}
    if prev_devices == scanned_devices: # compare path and realpath
        if enumstate['monitor']:
            if enumstate['retries'] > 0:
                enumstate['starttime'] += 2 #try again in 2 seconds until we get a change
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
    debug('serial probing', name, devices, probes)
    
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

    # if the last working device is recently plugged in last 5 seconds
    # probe it exclusively
    if probe['lastworking'] and probe['lastworking'][0] in devices and \
       t0 - devices[probe['lastworking'][0]]['time'] < 5:
        probe['probelast'] = True
        debug('serial probe skip timeout', name, t0 - devices[probe['lastworking'][0]]['time'], devices, probes)
    elif t0 - probe['time'] < timeout:
        # prevent probing too often
        probe['device'] = False
        return False
    probe['time'] = t0

    # current device probed, try next baud rate
    if probe['device']:
        probe['bauds'] = probe['bauds'][1:]
        if probe['bauds']:  # there are more bauds to try
            return probe['device'], probe['bauds'][0]
        probe['device'] = False # relinquish device

    # try the last working device every other probe
    device_list = list(devices)
    if probe['probelast']:
        probe['probelast'] = False # next time probe new devices
        if not probe['lastworking']:
            return False
        last_device, last_baud = probe['lastworking']
        for index in range(len(device_list)):
            device = device_list[index]
            if device == last_device:
                break
        else: # last device not found
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
            if probe_device == device or probe_device == devices[device]['realpath']:
                probe['probelast'] = True # next time probe last
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
                        probe['probelast'] = True # next time probe last
                        index += 1
                        break
            else:
                break
    else:
        probe['lastdevice'] = False
        return False

    serial_device = device, bauds[0]

    try:
        import serial
        serial.Serial
    except Exception as e:
        print('No serial.Serial available')
        print('pip3 uninstall serial')
        print('pip3 install pyserial')
        exit(1)

    try:
        debug('serialprobe open', name, serial_device)
        dev = serial.Serial(*serial_device)
        dev.close() # opened without exception, now close it
    except serial.serialutil.SerialException as err:
        arg = err.args[0]
        if type(arg) == type('') and 'Errno ' in  arg:
            arg = int(arg[arg.index('Errno ')+6: arg.index(']')])
        if arg == 16: # device busy, retry later
            print('serialprobe busy, try again later', name, err)
        elif arg == 6: # No such device or address, don't try again
            del devices[device]
        elif arg == 5: # input output error (unusable)
            del devices[device]
        elif arg == 2: # No such file or directory
            del devices[device]
        else:
            del devices[device]
            print('serialprobe exception', serial_device, name, err)
            # don't try again if ttyS port?
            #if device.startswith('/dev/ttyS'):
            #    devices.remove(device)
        serial_device = False
    except IOError:
        print('io error', serial_device)
        del devices[device]
        serial_device = False

    probe['device'] = device
    if probe['probelast']:
        probe['lastdevice'] = device
    probe['bauds'] = bauds
    return serial_device

# reserve gpsd devices against probing
def reserve(device):
    i = 0
    global probes
    while 'reserved%d' % i in probes:
        i+=1
    realpath = os.path.realpath(device)
    for device_path in devices:
        if devices[device_path]['realpath'] == realpath:
            probes['reserved%d' % i] = {'device': device_path, 'lastworking': False}
            print('serial probe reserve', device, device_path, realpath)
            return
    print('serial probe reserve not found!!', device)

def unreserve(device):
    print('serial probe unreserve', device)
    global probes
    realpath = os.path.realpath(device)
    for device_path in devices:
        if devices[device_path]['realpath'] == realpath:
            for probe in probes:
                if probe.startswith('reserved'):
                    if probes[probe]['device'] == device_path:
                        del probes[probe]
                        return
    print('error: failed to unreserve!!!!!!!', device, realpath)
    
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
