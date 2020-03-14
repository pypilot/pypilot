#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time, socket, multiprocessing
import pyjson
from client import pypilotClient
from sensors import source_priority

# provide bi-directional translation of these keys
signalk_table = {'wind': {'environment.wind.speedApparent': 'speed',
                          'environment.wind.angleApparent': 'angle'},
                 'gps': {'navigation.courseOverGroundTrue': 'track',
                         'navigation.speedOverGround': 'speed'},
                 'rudder': {'steering.rudderAngle': 'angle'},
                 'apb': {'steering.autopilot.target.headingTrue': 'track'},
                 'imu': {'navigation.headingMagnetic': 'heading_lowpass',
                         'navigation.attitude': {'pitch': 'pitch', 'roll': 'roll', 'yaw': 'heading_lowpass'}}}

class signalk(object):
    def __init__(self, sensors=False):
        self.sensors = sensors
        if not sensors: # standalone
            self.client = pypilotClient()
            self.multiprocessing = False
        else:
            server = sensors.client.server
            self.multiprocessing = server.multiprocessing
            self.client = pypilotClient(server)

        self.initialized = False
        if self.multiprocessing:
            import multiprocessing
            self.process = multiprocessing.Process(target=self.process, daemon=True)
            self.process.start()
        else:
            self.process = False

    def setup(self):
        self.last_values = {}

        self.sensor_priority = {}
        for sensor in signalk_table:
            self.sensor_priority[sensor] = source_priority['none']
        self.sensor_priority['imu'] = 0 # override to never read imu from signalk
        
        self.period = 0.5
        self.signalk_host_port = False
        self.signalk_ws_url = False
        self.ws = False
        self.detect_signalk()
        self.initialized = True

    def detect_signalk(self):
        try:
            from zeroconf import ServiceBrowser, ServiceStateChange, Zeroconf
        except Exception as e:
            print('signalk: failed to import zeroconf, autodetection not possible using', self.signalk_ws_url)
            return

        def on_service_change(zeroconf, service_type, name, state_change):
            if state_change is not ServiceStateChange.Added:
                return
            info = zeroconf.get_service_info(service_type, name)
            if not info:
                return
            properties = {}
            for name, value in info.properties.items():
                properties[name.decode()] = value.decode()
            if properties['swname'] == 'signalk-server':
                self.signalk_host_port = socket.inet_ntoa(info.addresses[0]) + ':' + str(info.port)
                print('signalk server found', self.signalk_host_port)
        
        self.zeroconf = Zeroconf()
        self.browser = ServiceBrowser(self.zeroconf, "_http._tcp.local.", handlers=[on_service_change])

    def probe_signalk(self):
        print('signalk probe...', self.signalk_host_port)
        try:
            import requests
        except Exception as e:
            print('signalk could not import requests, install with pip3 install requests')
            return
        try:
            r = requests.get('http://' + self.signalk_host_port + '/signalk')
            contents = pyjson.loads(r.content)
            self.signalk_ws_url = contents['endpoints']['v1']['signalk-ws']# + '?subscribe=none'
        except Exception as e:
            print('failed to retrieve/parse data from', self.signalk_host_port, e)
            return
        print('signalk found', self.signalk_ws_url)

    def connect_signalk(self):
        try:
            from websocket import create_connection
        except Exception as e:
            print('signalk cannot create connection: try pip3 install websocket-client', e)
            return

        try:
            self.ws = create_connection(self.signalk_ws_url)
            self.ws.settimeout(0) # nonblocking
        except Exception as e:
            print('failed to connect signalk', e)

    def send_signalk(self, msg):
        print('would send signalk server', msg)
            
    def process(self):
        while True:
            self.poll(1)

    def poll(self, timeout=0):
        t0 = time.monotonic()
        if not self.initialized:
            self.setup()
            return

        if not self.signalk_host_port:
            time.sleep(timeout)
            return # waiting for signalk to detect

        if not self.signalk_ws_url:
            #zeroconf.close()  # takes a long time
            self.probe_signalk()
            time.sleep(max(timeout - (time.monotonic()-t0), 0))
            return
        
        if not self.ws:
            self.connect_signalk()
            time.sleep(max(timeout - (time.monotonic()-t0), 0))
            return # don't bother watching anything

        # at this point we have a connection
        # setup pypilot watches
        watches = ['imu.heading_lowpass', 'imu.roll', 'imu.pitch']
        for watch in watches:
            self.client.watch(watch, self.period)

            self.setup()
            if not self.ws:
                return
            
        self.client.poll(timeout)

        # read all messages from pypilot
        updated = {}
        while True:
            msg = self.client.receive_single()
            if not msg:
                break
            name, value = msg
            self.last_values[name] = value

            for sensor in signalk_table:
                signalk_sensor = signalk_table[sensor]
                for signalk_path, pypilot_path in signalk_sensor.items():
                    if name != pypilot_path:
                        continue
                    if type(pypilot_path) == type({}):
                        for signalk_key in pypilot_path:
                            updated[signalk_path] = True
                    else:
                        updated[signalk_path] = True

        # update sources
        for sensor in signalk_table:
            source_name = sensor + '.source'
            if source_name in self.last_values:
                self.update_sensor_source(sensor, self.last_values[source_name])
        
        # see if we can produce any signalk output from these
        updates = []
        for sensor in signalk_table:
            for signalk_path, pypilot_path in signalk_table[sensor].items():
                if not signalk_path in updated:
                    continue
                if type(pypilot_path) == type({}): # single path translates to multiple pypilot
                    v = {}
                    for signalk_key, pypilot_key in pypilot_path.items():
                        key = sensor+'.'+pypilot_key
                        if not key in self.last_values:
                            break
                        v[signalk_key] = self.last_values[key]
                    else:
                        updates.append({'path': signalk_path, 'value': v})
                else:
                    key = sensor+'.'+pypilot_path
                    if key in self.last_values:
                        updates.append({'path': signalk_path, 'value': self.last_values[key]})

        if updates:
            # send signalk updates
            msg = {"updates":[{"$source":"pypilot","values":updates}]}
            try:
                self.ws.send(pyjson.dumps(msg)+'\n')
            except Exception as e:
                print('signalk failed to send', e)
                self.ws.close()
                self.ws = False
                return

        signalk_values = {}
        while self.ws:
            try:
                self.receive_signalk(self.ws.recv(), signalk_values)
            except Exception as e:
                print('signalk failed to receive', e)
                self.ws.close()
                self.ws = False
                return

        for sensor, sensor_table in signalk_table.items():
            for source, values in signalk_values.items():
                data = {}
                for signalk_path, signalk_value in values.items():
                    if signalk_path in sensor_table:
                        pypilot_path = sensor_table[signalk_path]
                        data[pypilot_path] = signalk_value
                if data:
                    data['device'] = source
                    if self.sensors:
                        self.sensors.write(sensor, data, 'signalk')
                    else:
                        print('signalk received', sensor, data)
            
    def receive_signalk(self, msg, signalk_values):
        data = pyjson.loads(msg)
        if 'updates' in data:
            updates = data['updates']
            for update in updates:
                source = 'unknown'
                if 'source' in update:
                    source = update['source']['talker']
                elif '$source' in update:
                    source = update['$source']
                if 'timestamp' in update:
                    timestamp = update['timestamp']
                values = update['values']
                for value in values:
                    if not source in signalk_values:
                        signalk_values[source] = {}
                    signalk_values[source][value['path']] = value['value']
                    
    def update_sensor_source(self, sensor, source):
        priority = source_priority[source]
        self.sensor_priority[sensor] = priority
        watch = priority < source_priority['signalk'] # translate from pypilot -> signalk
        if watch:
            watch = self.period
        subscribe = priority > source_priority['signalk'] # translate signalk -> pypilot
        signalk_sensor = signalk_sensors[sensor]
        for signalk_path, pypilot_path in signalk_table.items():
            self.client.watch(pypilot_path, watch)
            self.subscribe_signalk(signalk_path, subscribe)

    def subscribe_signalk(self, sensor, value):
        signalk_sensor = signalk_sensors[sensor]
        for signalk_path, pypilot_path in signalk_sensor.items():
            subscriptions.append({'path': signalk_path, 'minPeriod': self.period*1000, 'format': 'delta', 'policy': 'instant'})
        subscription = {'context': 'vessels.self'}
        if value:
            subscription['subscribe'] = subscriptions
        else:
            subscription['unsubscribe'] = subscriptions
        self.ws.send(pyjson.dumps(subscription+'\n'))
            

def main():
    sk = signalk()
    while True:
        sk.poll(1)
            
if __name__ == '__main__':
    main()
