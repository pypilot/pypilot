#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

# determine if we are on tinypilot if piCore is in uname -r
import tempfile, subprocess, os
temp = tempfile.mkstemp()
p=subprocess.Popen(['uname', '-r'], stdout=temp[0], close_fds=True)
p.wait()
f = os.fdopen(temp[0], 'r')
f.seek(0)
kernel_release = f.readline().rstrip()
f.close()

tinypilot = 'piCore' in kernel_release

# javascript uses lowercase bool, easier to use int
tinypilot = 1 if tinypilot else 0


# for tinypilot provide wifi config
def wifi(app):
    if not tinypilot:
        return
    
    @app.route('/wifi', methods=['GET', 'POST'])
    def wifi():
        networking = '/home/tc/.pypilot/networking.txt'

        wifi = {'mode': 'Master', 'ssid': 'pypilot', 'psk': '', 'client_ssid': 'pypilot', 'client_psk': ''}
        try:
            f = open(networking, 'r')
            while True:
                l = f.readline()
                if not l:
                    break
                try:
                    name, value = l.split('=')
                    wifi[name] = value.rstrip()
                except Exception as e:
                    print('failed to parse line in networking.txt', l)
            f.close()
        except:
            pass

        if request.method == 'POST':
            try:
                for name in request.form:
                    cname = name
                    if name != 'mode' and request.form['mode'] == 'Managed':
                        cname = 'client_' + name
                    wifi[cname] = str(request.form[name])

                f = open(networking, 'w')
                for name in wifi:
                    f.write(name+'='+wifi[name]+'\n')
                f.close()

                os.system('/opt/networking.sh')
            except Exception as e:
                print('exception!', e)

        return render_template('wifi.html', async_mode=socketio.async_mode, wifi=Markup(wifi))
