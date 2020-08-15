#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

filename = 'config'

def config(key, value):
    config = read_config()
    config[key] = value
    write_config(config)

def read_config():
    config = {}
    failed = False
    try:
        f = open(filename)
        while True:
            line = f.readline().rstrip()
            try:
                if not line:
                    break
                key, value = line.split('=', 1)
                v = value.strip()
                config[key.strip()] = v
            except Exception as e:
                print('failed parsing line', line, e)
                failed = True
                break
        f.close()
    except Exception as e:
        print('failed to load config', e)
        failed = True
    if failed:
        config = {'essid': 'pypilot', 'psk': '', 'address': ''}
        write_config(config);

    return config

def write_config(config):
    f = open(filename, 'w')
    for key in config:
        f.write(key)
        f.write(' = ')
        f.write(str(config[key]))
        f.write('\n')
    f.close()
