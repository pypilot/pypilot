#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.  

import time, socket, multiprocessing, os
from nonblockingpipe import NonBlockingPipe
import pyjson
from client import pypilotClient
from values import RangeProperty
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

token_path = os.getenv('HOME') + '/.pypilot/signalk-token'

class signalk(object):
    def __init__(self, sensors=False):
        self.sensors = sensors
        if not sensors: # only signalk process for testing
            self.client = pypilotClient()
            self.multiprocessing = False
        else:
            server = sensors.client.server
            self.multiprocessing = server.multiprocessing
            self.client = pypilotClient(server)

        self.initialized = False
        self.signalk_access_url = False
        self.last_access_request_time = 0

        print('hitehre')
        try:
            f = open(token_path)
            self.token = f.read()
            print('read token', self.token)
            f.close()
        except Exception as e:
            print('signalk failed to read token', token_path)
            self.token = False
        self.sensors_pipe, self.sensors_pipe_out = NonBlockingPipe('nmea pipe', self.multiprocessing)
        if self.multiprocessing:
            import multiprocessing
            self.process = multiprocessing.Process(target=self.process, daemon=True)
            self.process.start()
        else:
            self.process = False

    def setup(self):
        try:
            from zeroconf import ServiceBrowser, ServiceStateChange, Zeroconf
        except Exception as e:
            print('signalk: failed to import zeroconf, autodetection not possible')
            print('try pip3 install zeroconf or apt install python3-zeroconf')
            return
            
        self.last_values = {}
        self.signalk_msgs = {}
        self.signalk_msgs_skip = {}

        self.period = self.client.register(RangeProperty('signalk.period', .5, .1, 2, persistent=True))

        self.signalk_host_port = False
        self.signalk_ws_url = False
        self.ws = False
        
        class Listener:
            def __init__(self, signalk):
                self.signalk = signalk
                self.name_type = False
            
            def remove_service(self, zeroconf, type, name):
                print('zeroconf service %s removed', name, type)
                if self.name_type == (name, type):
                    self.signalk.signalk_host_port = False
                    print('signalk server lost')

            def add_service(self, zeroconf, type, name):
                print('zeroconf service add', name, type)
                self.name_type = name, type
                info = zeroconf.get_service_info(type, name)
                if not info:
                    return
                properties = {}
                for name, value in info.properties.items():
                    properties[name.decode()] = value.decode()
                if properties['swname'] == 'signalk-server':
                    try:
                        host_port = socket.inet_ntoa(info.addresses[0]) + ':' + str(info.port)
                    except Exception as e:
                        host_port = socket.inet_ntoa(info.address) + ':' + str(info.port)
                    self.signalk.signalk_host_port = host_port
                    print('signalk server found', host_port)

        zeroconf = Zeroconf()
        listener = Listener(self)
        browser = ServiceBrowser(zeroconf, "_http._tcp.local.", listener)
        #zeroconf.close()
        self.initialized = True
        

    def probe_signalk(self):
        print('signalk probe...', self.signalk_host_port)
        try:
            import requests
        except Exception as e:
            print('signalk could not import requests', e)
            print("try 'sudo apt install python3-requests' or 'pip3 install requests'")
            time.sleep(50)
            return

        try:
            r = requests.get('http://' + self.signalk_host_port + '/signalk')
            contents = pyjson.loads(r.content)
            self.signalk_ws_url = contents['endpoints']['v1']['signalk-ws'] + '?subscribe=none'
        except Exception as e:
            print('failed to retrieve/parse data from', self.signalk_host_port, e)
            time.sleep(5)
            return
        print('signalk found', self.signalk_ws_url)

    def request_access(self):
        import requests
        if self.signalk_access_url:
            dt = time.monotonic() - self.last_access_request_time            
            if dt < 10:
                return
            self.last_access_request_time = time.monotonic()
            try:
                r = requests.get(self.signalk_access_url)
                contents = pyjson.loads(r.content)
                print('signalk see if token is ready', self.signalk_access_url, contents)
                if contents['state'] == 'COMPLETED':
                    if 'accessRequest' in contents:
                        access = contents['accessRequest']
                        if access['permission'] == 'APPROVED':
                            self.token = access['token']
                            print('signalk received token', self.token)
                            try:
                                f = open(token_path, 'w')
                                f.write(self.token)
                                f.close()
                            except Exception as e:
                                print('signalk failed to store token', token_path)
                    else:
                        self.signalk_access_url = False
            except Exception as e:
                print('error requesting access', e)
                self.signalk_access_url = False
            return

        try:
            uid = random_uid()
            uid = "1234-45653343454";
            r = requests.post('http://' + self.signalk_host_port + '/signalk/v1/access/requests', data={"clientId":uid, "description": "pypilot"})
            
            contents = pyjson.loads(r.content)
            print('post', contents)
            if contents['statusCode'] == 202 or contents['statusCode'] == 400:
                self.signalk_access_url = 'http://' + self.signalk_host_port + contents['href']
                print('signalk request access url', self.signalk_access_url)
        except Exception as e:
            print('signalk error requesting access', e)
            self.signalk_ws_url = False
        
    def connect_signalk(self):
        try:
            from websocket import create_connection
        except Exception as e:
            print('signalk cannot create connection:', e)
            print('try pip3 install websocket-client or apt install python3-websocket')
            self.signalk_host_port = False
            return

        self.subscribed = {}
        for sensor in list(signalk_table):
            self.subscribed[sensor] = False
        self.subscriptions = [] # track signalk subscriptions
        self.signalk_values = {}
        try:
            self.ws = create_connection(self.signalk_ws_url, header={'Authorization': 'JWT ' + self.token})
            self.ws.settimeout(0) # nonblocking

            #self.ws.send(pyjson.dumps({"clientId": self.uid, "validate":{"token": self.token}})+'\n')
        except Exception as e:
            print('failed to connect signalk', e)
            self.token = False

    def process(self):
        time.sleep(6) # let other stuff load
        print('signalk process', os.getpid())
        self.process = False
        while True:
            time.sleep(.1)
            self.poll(1)

    def poll(self, timeout=0):
        if self.process:
            msg = self.sensors_pipe_out.recv()
            while msg:
                sensor, data = msg
                self.sensors.write(sensor, data, 'signalk')
                msg = self.sensors_pipe_out.recv()
            return

        t0 = time.monotonic()
        if not self.initialized:
            self.setup()
            return

        self.client.poll(timeout)
        if not self.signalk_host_port:
            return # waiting for signalk to detect

        t1 = time.monotonic()
        if not self.signalk_ws_url:
            self.probe_signalk()
            return

        t2 = time.monotonic()
        if not self.token:
            self.request_access()
            return
        t3 = time.monotonic()

        if not self.ws:
            self.connect_signalk()
            if not self.ws:
                return
            print('connected to signalk server')
            # setup pypilot watches
            watches = ['imu.heading_lowpass', 'imu.roll', 'imu.pitch', 'timestamp']
            for watch in watches:
                self.client.watch(watch, self.period.value)
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

        t4 = time.monotonic()

        while True:
            try:
                msg = self.ws.recv()
                print('sigk', msg)
            except:
                break
            self.receive_signalk(msg)

        t5 = time.monotonic()
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
                    print('data', data, sensor)
                    if self.sensors_pipe:
                        self.sensors_pipe.send([sensor, data])
                    else:
                        print('signalk received', sensor, data)
                    break
        #print('sigktimes', t1-t0, t2-t1, t3-t2, t4-t3, t5-t4)

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
                        updates.append({'path': signalk_path, 'value': v})
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
        sk_priority = source_priority['signalk']
        watch = priority < sk_priority # translate from pypilot -> signalk
        if watch:
            watch = self.period.value
        for signalk_path, pypilot_path in signalk_table[sensor].items():
            self.client.watch(pypilot_path, watch)
        self.subscribe_signalk(sensor, priority >= sk_priority)

    def subscribe_signalk(self, sensor, value):
        # prevent duplicating subscriptions
        if self.subscribed[sensor] == value:
            return
        self.subscribed[sensor] = value

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
                subscriptions.append({'path': signalk_path, 'minPeriod': self.period.value*1000, 'format': 'delta', 'policy': 'instant'})
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
        #print('signalk subscribe', subscription)
        self.ws.send(pyjson.dumps(subscription)+'\n')

def main():
    sk = signalk()
    while True:
        sk.poll(1)
            
if __name__ == '__main__':
    main()
