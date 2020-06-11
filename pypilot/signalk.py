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

dependencies = True
try:
    from zeroconf import ServiceBrowser, ServiceStateChange, Zeroconf
except Exception as e:
    print('signalk: failed to import zeroconf, autodetection not possible')
    print('try pip3 install zeroconf')
    dependencies = False

try:
    import requests
except Exception as e:
    print('signalk could not import requests')
    print('try pip3 install requests')
    dependencies = False
    
    
class signalk(object):
    def __init__(self, sensors=False):
        self.sensors = sensors
        if not sensors: # standalone
            self.client = pypilotClient()
            self.multiprocessing = False
        else:
            server = sensors.client.server
            #self.multiprocessing = server.multiprocessing
            self.multiprocessing = False
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
        self.signalk_msgs = {}
        self.signalk_msgs_skip = {}

        self.sensor_priority = {}
        for sensor in signalk_table:
            self.sensor_priority[sensor] = source_priority['none']
        self.sensor_priority['imu'] = 0 # override to never read imu from signalk
        
        self.period = 0.5
        self.signalk_host_port = False
        self.signalk_ws_url = False
        self.ws = False

        if not dependencies:
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

        self.signalk_host_port = 'localhost:3000'
        self.initialized = True

    def probe_signalk(self):
        print('signalk probe...', self.signalk_host_port)
        try:
            r = requests.get('http://' + self.signalk_host_port + '/signalk')
            contents = pyjson.loads(r.content)
            self.signalk_ws_url = contents['endpoints']['v1']['signalk-ws'] + '?subscribe=none'
        except Exception as e:
            print('failed to retrieve/parse data from', self.signalk_host_port, e)
            return
        print('signalk found', self.signalk_ws_url)

    def connect_signalk(self):
        try:
            from websocket import create_connection
        except Exception as e:
            print('signalk cannot create connection:', e)
            print('try pip3 install websocket-client')
            return

        self.subscriptions = [] # track signalk subscriptions
        self.signalk_values = {}
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
        self.client.poll(timeout)
        if not self.initialized:
            self.setup()
            return

        if not self.signalk_host_port:
            time.sleep(timeout)
            return # waiting for signalk to detect

        if not self.signalk_ws_url:
            #zeroconf.close()  # takes a long time
            self.probe_signalk()
            return

        if not self.ws:
            self.connect_signalk()
            if not self.ws:
                return
            print('connected to signalk server')
            # setup pypilot watches
            watches = ['imu.heading_lowpass', 'imu.roll', 'imu.pitch', 'timestamp']
            for watch in watches:
                self.client.watch(watch, self.period)
            for sensor in signalk_table:
                self.client.watch(sensor+'.source')
            return

        # at this point we have a connection
        # read all messages from pypilot
        while True:
            msg = self.client.receive_single()
            if not msg:
                break
            name, value = msg
            #print('msg', msg)
            if name == 'timestamp':
                self.send_signalk()
                self.last_values = {} # reset last values
                self.signalk_msgs = {}
            self.last_values[name] = value

            if name.endswith('.source'):
                # update sources
                for sensor in signalk_table:
                    source_name = sensor + '.source'
                    if name == source_name:
                        self.update_sensor_source(sensor, value)

        self.send_signalk()

        while True:
            try:
                msg = self.ws.recv()
            except:
                break
            self.receive_signalk(msg)

        for sensor, sensor_table in signalk_table.items():
            for source, values in self.signalk_values.items():
                data = {}
                for signalk_path, pypilot_path in sensor_table.items():
                    if signalk_path in values:
                        data[pypilot_path] = values[signalk_path]
                    else:
                        break
                else:
                    for signalk_path in sensor_table:
                        del values[signalk_path]
                    # all needed sensor data is found 
                    data['device'] = source
                    #print('data', data, sensor)
                    if self.sensors:
                        self.sensors.write(sensor, data, 'signalk')
                    else:
                        print('signalk received', sensor, data)
                    break

    def send_signalk(self):
        # see if we can produce any signalk output from the data we have read
        updates = []
        for sensor in signalk_table:
            for signalk_path, pypilot_path in signalk_table[sensor].items():
                if signalk_path in self.signalk_msgs:
                    continue
                if type(pypilot_path) == type({}): # single path translates to multiple pypilot
                    v = {}
                    for signalk_key, pypilot_key in pypilot_path.items():
                        key = sensor+'.'+pypilot_key
                        if not key in self.last_values:
                            break
                        v[signalk_key] = self.last_values[key]                        
                    else:
                        updates.append({'paths': signalk_path, 'value': v})
                        self.signalk_msgs[signalk_path] = True                        
                else:
                    key = sensor+'.'+pypilot_path
                    if key in self.last_values:
                        updates.append({'path': signalk_path, 'value': self.last_values[key]})
                        self.signalk_msgs[signalk_path] = True                        

        if updates:
            # send signalk updates
            msg = {'updates':[{'$source':'pypilot','values':updates}]}
            #print('signalk updates', msg)
            try:
                self.ws.send(pyjson.dumps(msg)+'\n')
            except Exception as e:
                print('signalk failed to send', e)
                self.disconnect_signalk()

    def disconnect_signalk(self):
        self.ws.close()
        self.ws = False
        self.client.clear_watches() # don't need to receive pypilot data

    def receive_signalk(self, msg):
        try:
            data = pyjson.loads(msg)
        except:
            print('failed to parse signalk msg:', msg)
            return
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
                if not source in self.signalk_values:
                    self.signalk_values[source] = {}
                for value in update['values']:
                    path = value['path']
                    if path in self.signalk_msgs_skip:
                        self.signalk_values[source][path] = value['value']
                    else:
                        self.signalk_msgs_skip[path] = True
                    
    def update_sensor_source(self, sensor, source):
        priority = source_priority[source]
        self.sensor_priority[sensor] = priority
        watch = priority < source_priority['signalk'] # translate from pypilot -> signalk
        if watch:
            watch = self.period
        for signalk_path, pypilot_path in signalk_table[sensor].items():
            self.client.watch(pypilot_path, watch)
        if priority != source_priority['signalk']:
            self.subscribe_signalk(sensor, not watch)

    def subscribe_signalk(self, sensor, value):
        if not value:
            #signalk can't unsubscribe by path!?!?!
            subscription = {'context': '*', 'unsubscribe': [{'path': '*'}]}
            self.ws.send(pyjson.dumps(subscription)+'\n')
        
        signalk_sensor = signalk_table[sensor]
        if value:
            subscriptions = []
            for signalk_path in signalk_sensor:
                if signalk_path in self.signalk_msgs_skip:
                    del self.signalk_msgs_skip[signalk_path]
                subscriptions.append({'path': signalk_path, 'minPeriod': self.period*1000, 'format': 'delta', 'policy': 'instant'})
            self.subscriptions += subscriptions
        else:
            # remove this subscription and resend all subscriptions
            subscriptions = []
            for subscription in self.subscriptions:
                for signalk_path in signalk_sensor:
                    if subscription['path'] == signalk_path:
                        break
                else:
                    subscriptions.append(subscription)
            self.subscriptions = subscriptions
            self.signalk_msgs_skip = {}
            
        subscription = {'context': 'vessels.self'}
        subscription['subscribe'] = subscriptions
        self.ws.send(pyjson.dumps(subscription)+'\n')

def main():
    sk = signalk()
    while True:
        sk.poll(1)
            
if __name__ == '__main__':
    main()
