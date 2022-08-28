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
from values import Property, RangeProperty
from sensors import source_priority

signalk_priority = source_priority['signalk']
radians = 3.141592653589793/180
meters_s = 0.5144456333854638

# provide bi-directional translation of these keys
signalk_table = {'wind': {('environment.wind.speedApparent', meters_s): 'speed',
                          ('environment.wind.angleApparent', radians): 'direction'},
                 'gps': {('navigation.courseOverGroundTrue', radians): 'track',
                         ('navigation.speedOverGround', meters_s): 'speed',
                         ('navigation.position', 1): 'fix'},
                 'rudder': {('steering.rudderAngle', radians): 'angle'},
                 'apb': {('steering.autopilot.target.headingTrue', radians): 'track'},
                 'imu': {('navigation.headingMagnetic', radians): 'heading_lowpass',
                         ('navigation.attitude', radians): {'pitch': 'pitch', 'roll': 'roll', 'yaw': 'heading_lowpass'},
                         ('navigation.rateOfTurn', radians): 'headingrate_lowpass'},
                 'water': {('navigation.speedThroughWater', meters_s): 'speed',
                           ('navigation.leewayAngle', radians): 'leeway'}}


token_path = os.getenv('HOME') + '/.pypilot/signalk-token'

def debug(*args):
    #print(*args)
    pass

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
        self.missingzeroconfwarned = False
        self.signalk_access_url = False
        self.last_access_request_time = 0

        self.sensors_pipe, self.sensors_pipe_out = NonBlockingPipe('signalk pipe', self.multiprocessing)
        if self.multiprocessing:
            import multiprocessing
            self.process = multiprocessing.Process(target=self.process, daemon=True)
            self.process.start()
        else:
            self.process = False

    def setup(self):
        try:
            f = open(token_path)
            self.token = f.read()
            print('signalk' + _('read token'), self.token)
            f.close()
        except Exception as e:
            print('signalk ' + _('failed to read token'), token_path)
            self.token = False

        try:
            from zeroconf import ServiceBrowser, ServiceStateChange, Zeroconf
        except Exception as e:
            if not self.missingzeroconfwarned:
                print('signalk: ' + _('failed to') + ' import zeroconf, ' + _('autodetection not possible'))
                print(_('try') + ' pip3 install zeroconf' + _('or') + ' apt install python3-zeroconf')
                self.missingzeroconfwarned = True
            time.sleep(20)
            return
            
        self.last_values = {}
        self.last_sources = {}
        self.signalk_last_msg_time = {}

        # store certain values across parsing invocations to ensure
        # all of the keys are filled with the latest data
        self.last_values_keys = {}
        for sensor in signalk_table:
            for signalk_path_conversion, pypilot_path in signalk_table[sensor].items():
                signalk_path, signalk_conversion = signalk_path_conversion
                if type(pypilot_path) == type({}): # single path translates to multiple pypilot
                    self.last_values_keys[signalk_path] = {}

        self.period = self.client.register(RangeProperty('signalk.period', .5, .1, 2, persistent=True))
        self.uid = self.client.register(Property('signalk.uid', 'pypilot', persistent=True))

        self.signalk_host_port = False
        self.signalk_ws_url = False
        self.ws = False
        
        class Listener:
            def __init__(self, signalk):
                self.signalk = signalk
                self.name_type = False
            
            def remove_service(self, zeroconf, type, name):
                print('signalk zeroconf ' + _('service removed'), name, type)
                if self.name_type == (name, type):
                    self.signalk.signalk_host_port = False
                    self.signalk.disconnect_signalk()
                    print('signalk ' + _('server lost'))

            def update_service(self, zeroconf, type, name):
                self.add_service(zeroconf, type, name)

            def add_service(self, zeroconf, type, name):
                print('signalk zeroconf ' + _('service add'), name, type)
                self.name_type = name, type
                info = zeroconf.get_service_info(type, name)
                if not info:
                    return
                properties = {}
                for name, value in info.properties.items():
                    try:
                        properties[name.decode()] = value.decode()
                    except Exception as e:
                        print('signalk zeroconf exception', e, name, value)

                if 'swname' in properties and properties['swname'] == 'signalk-server':
                    try:
                        host_port = socket.inet_ntoa(info.addresses[0]) + ':' + str(info.port)
                    except Exception as e:
                        host_port = socket.inet_ntoa(info.address) + ':' + str(info.port)
                    self.signalk.signalk_host_port = host_port
                    print('signalk ' + _('server found'), host_port)

        zeroconf = Zeroconf()
        listener = Listener(self)
        browser = ServiceBrowser(zeroconf, "_http._tcp.local.", listener)
        #zeroconf.close()
        self.initialized = True

    def probe_signalk(self):
        print('signalk ' + _('probe') + '...', self.signalk_host_port)
        try:
            import requests
        except Exception as e:
            print('signalk ' + _('could not') + ' import requests', e)
            print(_('try') + " 'sudo apt install python3-requests' " + _('or') + " 'pip3 install requests'")
            time.sleep(50)
            return

        try:
            r = requests.get('http://' + self.signalk_host_port + '/signalk')
            contents = pyjson.loads(r.content)
            self.signalk_ws_url = contents['endpoints']['v1']['signalk-ws'] + '?subscribe=none'
        except Exception as e:
            print(_('failed to retrieve/parse data from'), self.signalk_host_port, e)
            time.sleep(5)
            self.signalk_host_port = False
            return
        print('signalk ' + _('found'), self.signalk_ws_url)

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
                print('signalk ' + _('see if token is ready'), self.signalk_access_url, contents)
                if contents['state'] == 'COMPLETED':
                    if 'accessRequest' in contents:
                        access = contents['accessRequest']
                        if access['permission'] == 'APPROVED':
                            self.token = access['token']
                            print('signalk ' + _('received token'), self.token)
                            try:
                                f = open(token_path, 'w')
                                f.write(self.token)
                                f.close()
                            except Exception as e:
                                print('signalk ' + _('failed to store token'), token_path)
                        # if permission == DENIED should we try other servers??
                    self.signalk_access_url = False
            except Exception as e:
                print('signalk ' + _('error requesting access'), e)
                self.signalk_access_url = False
            return

        try:
            def random_number_string(n):
                if n == 0:
                    return ''
                import random
                return str(int(random.random()*10)) + random_number_string(n-1)
            
            if self.uid.value == 'pypilot':
                self.uid.set('pypilot-' + random_number_string(11))
            r = requests.post('http://' + self.signalk_host_port + '/signalk/v1/access/requests', data={"clientId":self.uid.value, "description": "pypilot"})
            
            contents = pyjson.loads(r.content)
            print('signalk post', contents)
            if contents['statusCode'] == 202 or contents['statusCode'] == 400:
                self.signalk_access_url = 'http://' + self.signalk_host_port + contents['href']
                print('signalk ' + _('request access url'), self.signalk_access_url)
        except Exception as e:
            print('signalk ' + _('error requesting access'), e)
            self.signalk_ws_url = False
        
    def connect_signalk(self):
        try:
            from websocket import create_connection, WebSocketBadStatusException
        except Exception as e:
            print('signalk ' + _('cannot create connection:'), e)
            print(_('try') + ' pip3 install websocket-client ' + _('or') + ' apt install python3-websocket')
            self.signalk_host_port = False
            return

        self.subscribed = {}
        for sensor in list(signalk_table):
            self.subscribed[sensor] = False
        self.subscriptions = [] # track signalk subscriptions
        self.signalk_values = {}
        self.keep_token = False
        try:
            self.ws = create_connection(self.signalk_ws_url, header={'Authorization': 'JWT ' + self.token})
            self.ws.settimeout(0) # nonblocking
        except WebSocketBadStatusException:
            print('signalk ' + _('bad status, rejecting token'))
            self.token = False
            self.ws = False
        except ConnectionRefusedError:
            print('signalk ' + _('connection refused'))
            #self.signalk_host_port = False
            self.signalk_ws_url = False
            time.sleep(5)
        except Exception as e:
            print('signalk ' + _('failed to connect'), e)
            self.signalk_ws_url = False
            time.sleep(5)
            
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

        if not self.ws:
            self.connect_signalk()
            if not self.ws:
                return
            print('signalk ' + _('connected to'), self.signalk_ws_url)

            # setup pypilot watches
            watches = ['imu.heading_lowpass', 'imu.roll', 'imu.pitch', 'timestamp']
            watches += ['gps.filtered.output'] # for gps generation
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
            debug('signalk pypilot msg', msg)
            name, value = msg
            if name == 'timestamp':
                self.send_signalk()
                self.last_values = {}

            if name.endswith('.source'):
                # update sources
                for sensor in signalk_table:
                    source_name = sensor + '.source'
                    if name == source_name:
                        self.update_sensor_source(sensor, value)
                self.last_sources[name[:-7]] = value
            elif name == 'gps.filtered.output':
                self.client.watch('gps.filtered.fix', value)
            else:
                self.last_values[name] = value

        t3 = time.monotonic()
                
        t4 = time.monotonic()
        while True:
            try:
                msg = self.ws.recv()
            except Exception as e:
                break

            if not msg:
                print('signalk server closed connection')
                if not self.keep_token:
                    print('signalk invalidating token')
                    self.token = False
                self.disconnect_signalk()
                return

            try:
                self.receive_signalk(msg)
            except Exception as e:
                debug('failed to parse signalk', msg, e)
                return
            self.keep_token = True # do not throw away token if we got valid data

        t5 = time.monotonic()
        # convert received signalk values into sensor inputs if possible
        for sensor, sensor_table in signalk_table.items():
            for source, values in self.signalk_values.items():
                data = {}
                for signalk_path_conversion, pypilot_path in sensor_table.items():
                    signalk_path, signalk_conversion = signalk_path_conversion
                    if signalk_path in values:
                        try:
                            if not 'timestamp'in data and signalk_path in self.signalk_last_msg_time:
                                ts = time.strptime(self.signalk_last_msg_time[signalk_path], '%Y-%m-%dT%H:%M:%S.%f%z')
                                data['timestamp'] = time.mktime(ts)

                            value = values[signalk_path]
                            if type(pypilot_path) == type({}): # single path translates to multiple pypilot
                                for signalk_key, pypilot_key in pypilot_path.items():
                                    if not value[signalk_key] is None:
                                    data[pypilot_key] = value[signalk_key] / signalk_conversion
                            elif not value is None:
                                data[pypilot_path] = value / signalk_conversion
                        except Exception as e:
                            print(_('Exception converting signalk->pypilot'), e, self.signalk_values)
                            break
                    elif signalk_conversion != 1: # don't require fields with conversion of 1
                        break  # missing fields?  skip input this iteration
                else:
                    for signalk_path_conversion in sensor_table:
                        signalk_path, signalk_conversion = signalk_path_conversion
                        if signalk_path in values:
                            del values[signalk_path]
                    # all needed sensor data is found 
                    data['device'] = source + 'signalk'
                    if self.sensors_pipe:
                        self.sensors_pipe.send([sensor, data])
                    else:
                        debug('signalk ' + _('received'), sensor, data)
                    break
        #print('sigktimes', t1-t0, t2-t1, t3-t2, t4-t3, t5-t4)

    def send_signalk(self):
        # see if we can produce any signalk output from the data we have read
        updates = []
        for sensor in signalk_table:
            if sensor != 'imu' and (not sensor in self.last_sources or\
                                    source_priority[self.last_sources[sensor]]>=signalk_priority):
                #debug('signalk skip send from priority', sensor)
                continue

            if sensor == 'gps' and self.last_values['gps.filtered.output'] is True:
                continue
            
            for signalk_path_conversion, pypilot_path in signalk_table[sensor].items():
                signalk_path, signalk_conversion = signalk_path_conversion
                if type(pypilot_path) == type({}): # single path translates to multiple pypilot
                    keys = self.last_values_keys[signalk_path]
                    # store keys we need for this signalk path in dictionary
                    for signalk_key, pypilot_key in pypilot_path.items():
                        key = sensor+'.'+pypilot_key
                        if key in self.last_values:
                            keys[key] = self.last_values[key]

                    # see if we have the keys needed
                    v = {}
                    for signalk_key, pypilot_key in pypilot_path.items():
                        key = sensor+'.'+pypilot_key
                        if not key in keys:
                            break
                        v[signalk_key] = keys[key]*signalk_conversion
                    else:
                        updates.append({'path': signalk_path, 'value': v})
                        self.last_values_keys[signalk_path] = {}
                else:
                    key = sensor+'.'+pypilot_path
                    if key in self.last_values:
                        v = self.last_values[key]*signalk_conversion
                        updates.append({'path': signalk_path, 'value': v})

        # generate filtered gps output if enabled
        if self.last_values['gps.filtered.output'] is True and self.last_values['gps.filtered.fix']:
            fix = self.last_values['gps.filtered.fix']
            self.last_values['gps.filtered.fix'] = False
            try:
                for signalk_path_conversion, pypilot_path in signalk_table['gps'].items():
                    signalk_path, signalk_conversion = signalk_path_conversion
                    key = sensor+'.'+pypilot_path
                    if key in self.last_values:
                        v = fix[key]*signalk_conversion
                        updates.append({'path': signalk_path, 'value': v})
            except:
                pass

        if updates:
            # send signalk updates
            msg = {'updates':[{'$source':'pypilot','values':updates}]}
            debug('signalk updates', msg)
            try:
                self.ws.send(pyjson.dumps(msg)+'\n')
            except Exception as e:
                print('signalk ' + _('failed to send updates'), e)
                self.disconnect_signalk()

    def disconnect_signalk(self):
        if self.ws:
            self.ws.close()
        self.ws = False
        self.client.clear_watches() # don't need to receive pypilot data

    def receive_signalk(self, msg):
        try:
            data = pyjson.loads(msg)
        except:
            if msg:
                print('signalk ' + _('failed to parse msg:'), msg)
            return
        
        if 'updates' in data:
            updates = data['updates']
            for update in updates:
                source = 'unknown'
                if '$source' in update:
                    source = update['$source']
                elif 'source' in update:
                    if 'talker' in update['source']:
                        source = update['source']['talker']

                if 'timestamp' in update:
                    timestamp = update['timestamp']
                if not source in self.signalk_values:
                    self.signalk_values[source] = {}
                if 'values' in update:
                    values = update['values']
                elif 'meta' in update:
                    values = update['meta']
                else:
                    debug('signalk message update contains no values or meta', update)
                    continue

                for value in values:
                    path = value['path']
                    if path in self.signalk_last_msg_time:
                        if self.signalk_last_msg_time[path] == timestamp:
                            debug('signalk skip duplicate timestamp', source, path, timestamp)
                            continue
                        self.signalk_values[source][path] = value['value']
                    else:
                        debug('signalk skip initial message', source, path, timestamp)
                    self.signalk_last_msg_time[path] = timestamp
                    
    def update_sensor_source(self, sensor, source):
        priority = source_priority[source]
        watch = priority < signalk_priority # translate from pypilot -> signalk
        if watch:
            watch = self.period.value
        for signalk_path_conversion, pypilot_path in signalk_table[sensor].items():
            if type(pypilot_path) == type({}):
                for signalk_key, pypilot_key in pypilot_path.items():
                    pypilot_path = sensor + '.' + pypilot_key
                    if pypilot_path in self.last_values:
                        del self.last_values[pypilot_path]
                    self.client.watch(pypilot_path, watch)
            else:
                # remove any last values from this sensor
                pypilot_path = sensor + '.' + pypilot_path
                if pypilot_path in self.last_values:
                    del self.last_values[pypilot_path]
                self.client.watch(pypilot_path, watch)
        subscribe = priority >= signalk_priority

        # prevent duplicating subscriptions
        if self.subscribed[sensor] == subscribe:
            return
        self.subscribed[sensor] = subscribe

        if not subscribe:
            #signalk can't unsubscribe by path!?!?!
            subscription = {'context': '*', 'unsubscribe': [{'path': '*'}]}
            debug('signalk unsubscribe', subscription)
            try:
                self.ws.send(pyjson.dumps(subscription)+'\n')
            except Exception as e:
                print('signalk failed to send', e)
                self.disconnect_signalk()
                return
        
        signalk_sensor = signalk_table[sensor]
        if subscribe: # translate from signalk -> pypilot
            subscriptions = []
            for signalk_path_conversion in signalk_sensor:
                signalk_path, signalk_conversion = signalk_path_conversion
                if signalk_path in self.signalk_last_msg_time:
                    del self.signalk_last_msg_time[signalk_path]
                subscriptions.append({'path': signalk_path, 'minPeriod': self.period.value*1000, 'format': 'delta', 'policy': 'instant'})
            self.subscriptions += subscriptions
        else:
            # remove this subscription and resend all subscriptions
            debug('signalk remove subs', signalk_sensor, self.subscriptions)
            subscriptions = []
            for subscription in self.subscriptions:
                for signalk_path_conversion in signalk_sensor:
                    signalk_path, signalk_conversion = signalk_path_conversion
                    if subscription['path'] == signalk_path:
                        break
                else:
                    subscriptions.append(subscription)
            self.subscriptions = subscriptions
            self.signalk_last_msg_time = {}
            
        subscription = {'context': 'vessels.self'}
        subscription['subscribe'] = subscriptions
        debug('signalk subscribe', subscription)
        try:
            self.ws.send(pyjson.dumps(subscription)+'\n')
        except Exception as e:
            print('signalk failed to send subscription', e)
            self.disconnect_signalk()

def main():
    sk = signalk()
    while True:
        sk.poll(1)
            
if __name__ == '__main__':
    main()
