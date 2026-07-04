#!/usr/bin/env python
#
#   Copyright (C) 2020 Sean D'Epagnier
#
# This Program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.

import multiprocessing
import math
import os
import socket
import time

import pyjson
from client import pypilotClient
from values import Property, RangeProperty, BooleanProperty, EnumProperty
from nonblockingpipe import NonBlockingPipe
from sensors import source_priority
from values import Property, RangeProperty

requests = None
websocket = None
zeroconf = None

signalk_priority = source_priority['signalk']
radians = 3.141592653589793/180
meters_s = 0.5144456333854638

# provide bi-directional translation of these keys
signalk_table = {'wind': {('environment.wind.speedApparent', meters_s): 'speed',
                          ('environment.wind.angleApparent', radians): 'direction'},
                 'truewind': {('environment.wind.speedTrue', meters_s): 'speed',
                              ('environment.wind.angleTrue', radians): 'direction'},
                 'gps': {('navigation.courseOverGroundTrue', radians): 'track',
                         ('navigation.speedOverGround', meters_s): 'speed',
                         ('navigation.position', 1): {'latitude': 'lat',
                                                      'longitude': 'lon'}},
                 'rudder': {('steering.rudderAngle', -radians): 'angle'},
                 'apb': {('steering.autopilot.target.headingTrue', radians): 'track'},
                 'imu': {('navigation.headingMagnetic', radians): 'heading_lowpass',
                         ('navigation.attitude', radians): {'pitch': 'pitch', 'roll': 'roll', 'yaw': 'heading_lowpass'},
                         ('navigation.rateOfTurn', radians): 'headingrate_lowpass'},
                 'water': {('navigation.speedThroughWater', meters_s): 'speed',
                           ('navigation.leewayAngle', radians): 'leeway'}}

token_path = os.path.expanduser('~') + '/.pypilot/signalk-token'

# Timeout (seconds) for all HTTP requests against the SignalK server. Keep
# this small because the poll loop is otherwise frozen while it waits.
SIGNALK_HTTP_TIMEOUT = 5

# Exponential backoff bounds for reconnect attempts to the SignalK server.
SIGNALK_BACKOFF_MIN = 5
SIGNALK_BACKOFF_MAX = 60

# Match the GPS sensor's loss timeout when assembling updates or reading a cached fix.
GPS_TIMEOUT = 8

# Default SignalK HTTP/WS port. Used when signalk.host is a bare hostname.
SIGNALK_DEFAULT_PORT = 3000

# Subscription policy values permitted by the SignalK v1 spec.
SIGNALK_POLICIES = ('instant', 'fixed', 'ideal')

# SignalK paths that pypilot is willing to accept PUT requests for.
# Map: signalk_path -> (pypilot_name, conversion_factor)
# Incoming SI value is divided by conversion_factor to get pypilot units.
# Keep this small: every entry here is a control surface exposed over
# the network when signalk.put.enabled is True.
signalk_put_table = {
    'steering.autopilot.target.headingMagnetic': ('ap.heading_command', radians),
    'steering.autopilot.target.headingTrue': ('ap.heading_command', radians),
    'steering.autopilot.state': ('ap.mode', 1),
    'steering.autopilot.engaged': ('ap.enabled', 1),
}

def debug(*args):
    #print(*args)
    pass


def _parse_signalk_timestamp(ts):
    """Convert a SignalK RFC 3339 timestamp to a POSIX float.

    The SignalK v1 spec (and RFC 3339) allow either a 'Z' UTC suffix or a
    numeric offset like '-04:00'. Real-world servers ship both: Node
    signalk-server emits 'Z', NMEA 2000 bridges often emit '-04:00'.
    time.strptime with '%Y-%m-%dT%H:%M:%S.%fZ' rejects the offset form,
    which used to drown the log in conversion exceptions.

    datetime.fromisoformat accepts both in Python 3.11+. For older
    interpreters, normalize 'Z' to '+00:00' first.
    """
    import datetime
    if not isinstance(ts, str):
        raise ValueError('timestamp must be a string')
    # Python 3.10 and older: fromisoformat does not accept trailing 'Z'.
    if ts.endswith('Z') and '+' not in ts and not ts[11:].count('-'):
        ts = ts[:-1] + '+00:00'
    dt = datetime.datetime.fromisoformat(ts)
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=datetime.timezone.utc)
    return dt.timestamp()

class ZeroConfProcess(multiprocessing.Process): 
    def __init__(self, signalk):
        self.name_type = False
        self.pipe = NonBlockingPipe('zeroconf', True)
        super().__init__(target=self.process, name='signalk_zero_conf', daemon=True)
        self.start()

    def remove_service(self, zc, type, name):
        print('signalk zeroconf ' + _('service removed'), name, type)
        if self.name_type == (name, type):
            self.pipe[1].send('disconnect')
            print('signalk zeroconf ' + _('server lost'))

    def update_service(self, zc, type, name):
        self.add_service(zc, type, name)

    def add_service(self, zc, type, name):
        print('signalk zeroconf ' + _('service add'), name, type)
        info = zc.get_service_info(type, name)
        if not info:
            return
        properties = {}
        for k, v in info.properties.items():
            # zeroconf reports valueless TXT records as (key, None) per RFC 6763;
            # skip rather than crash on None.decode() (issue #266)
            if k is None or v is None:
                continue
            try:
                properties[k.decode()] = v.decode()
            except Exception as e:
                print('signalk zeroconf exception', e, k, v)

        if 'swname' in properties and properties['swname'] == 'signalk-server':
            try:
                host_port = socket.inet_ntoa(info.addresses[0]) + ':' + str(info.port)
            except Exception:
                host_port = socket.inet_ntoa(info.address) + ':' + str(info.port)
            self.name_type = name, type
            self.pipe[1].send(host_port)


    def process(self):
        time.sleep(6) # delay signalk by 6 seconds to start other pypilot processes faster
        global zeroconf
        if zeroconf is None:
            try:
                import zeroconf
            except ImportError:
                print('signalk: ' + _('failed to') + ' import zeroconf, ' + _('autodetection not possible'))
                print(_('try') + ' pip3 install zeroconf ' + _('or') + ' apt install python3-zeroconf')
                while zeroconf is None:
                    time.sleep(10)
                    try:
                        import zeroconf
                        break
                    except:
                        pass

        current_ip_address = []
        zc = None
        # Poll interval for interface set changes. Short enough to react
        # to network flaps, long enough that we are not tearing down the
        # service browser every few seconds under normal conditions.
        poll_interval = 30
        while True:
            try:
                new_ip_address = zeroconf.get_all_addresses()
            except Exception as e:
                print('signalk zeroconf get_all_addresses failed', e)
                time.sleep(poll_interval)
                continue
            if current_ip_address != new_ip_address:
                debug("IP address changed from ", current_ip_address, "to", new_ip_address)
                current_ip_address = new_ip_address
                if zc is not None:
                    try:
                        zc.close()
                    except Exception as e:
                        debug('zeroconf close failed', e)
                try:
                    zc = zeroconf.Zeroconf()
                    self.browser = zeroconf.ServiceBrowser(zc, "_http._tcp.local.", self)
                except Exception as e:
                    print('signalk zeroconf init failed', e)
                    zc = None
                    time.sleep(poll_interval)
                    continue
            time.sleep(poll_interval)

    def poll(self):  # from signalk process
        last = False
        while True:
            p = self.pipe[0].recv()
            if not p:
                return last
            last = p


class signalk:
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

        self.sensors_pipe, self.sensors_pipe_out = NonBlockingPipe('signalk pipe', self.multiprocessing)

        if self.multiprocessing:
            self.zero_conf = ZeroConfProcess(self)
        else:
            print('multiprocessing disabled, signalk will not auto-discover via zeroconf')
            self.zero_conf = False

        if self.multiprocessing:
            import multiprocessing
            # target=self.process resolves to the `def process` method below.
            # Don't assign self.process before this line: an attribute with
            # that name would shadow the method, leaving target=False and
            # the subprocess a silent no-op.
            self.process = multiprocessing.Process(target=self.process, name='signalk', daemon=True)
            self.process.start()
        else:
            self.process = False


    def setup(self):
        try:
            f = open(token_path)
            self.token = f.read().strip()
            print('signalk ' + _('read token'))  # never log the token itself
            f.close()
            # SignalK tokens grant write access to the vessel; make sure the
            # file is not world- or group-readable.
            try:
                st = os.stat(token_path)
                if st.st_mode & 0o077:
                    os.chmod(token_path, 0o600)
            except Exception as e:
                print('signalk ' + _('could not secure token file'), e)
        except Exception as e:
            print('signalk ' + _('failed to read token'), token_path)
            self.invalid_token()

        self.last_values = {}
        self.last_sources = {}
        self.signalk_last_msg_time = {}
        self.gps_filtered_output = False

        # store certain values across parsing invocations to ensure
        # all of the keys are filled with the latest data
        self.last_values_keys = {}
        for sensor in signalk_table:
            for signalk_path_conversion, pypilot_path in signalk_table[sensor].items():
                signalk_path, signalk_conversion = signalk_path_conversion
                if type(pypilot_path) == type({}): # single path translates to multiple pypilot
                    self.last_values_keys[signalk_path] = {}

        self.enabled = self.client.register(BooleanProperty('signalk.enabled', True, persistent=True))
        self.period = self.client.register(RangeProperty('signalk.period', .5, .1, 2, persistent=True))
        self.last_period = False
        self.uid = self.client.register(Property('signalk.uid', 'pypilot', persistent=True))
        # Subscription policy per SignalK v1 spec. 'instant' sends every
        # update (cheap, but can saturate slow links). 'fixed' respects
        # minPeriod strictly. 'ideal' behaves like 'fixed' but batches.
        self.subscribe_policy = self.client.register(
            EnumProperty('signalk.subscribe.policy', 'instant',
                         list(SIGNALK_POLICIES), persistent=True))
        # Allow SignalK clients to command pypilot via PUT deltas. Off by
        # default because it opens a remote control surface; the operator
        # has to opt in.
        self.put_enabled = self.client.register(
            BooleanProperty('signalk.put.enabled', False, persistent=True))
        # When the SignalK server has SSL enabled it redirects http->https
        # and advertises wss:// endpoints. Verify the server's certificate
        # by default; disable to tolerate self-signed certificates, which
        # is the common case for a SignalK server on the boat LAN. This
        # relaxes verification for BOTH the HTTP probe/access requests and
        # the websocket stream.
        self.tls_verify = self.client.register(
            BooleanProperty('signalk.tls.verify', True, persistent=True))
        self.signalk_host = self.client.register(Property('signalk.host', '', persistent=True))

        self.signalk_host_port = False
        self.signalk_ws_url = False
        self.signalk_http_base = False
        self.ws = False
        self.last_manual_signalk_host = False

        self.initialized = True

    def probe_signalk(self):
        debug('signalk ' + _('probe') + '...', self.signalk_host_port)
        global requests
        if requests is None:
            try:
                import requests
            except ImportError:
                print('signalk ' + _('could not') + ' import requests')
                print(_('try') + " 'sudo apt install python3-requests' " + _('or') + " 'pip3 install requests'")
                time.sleep(50)
                return
            else:
                # urllib3 raises InsecureRequestWarning whenever a request runs with
                # verify=False, which happens once signalk.tls.verify is disabled (to
                # accept a self-signed cert) and would then flood the log. Silence just
                # that warning at import: it never fires while verification is on, so
                # disabling it unconditionally is harmless, and the security tradeoff
                # stays visible through the signalk.tls.verify flag.
                try:
                    from urllib3.exceptions import InsecureRequestWarning
                    requests.packages.urllib3.disable_warnings(InsecureRequestWarning)
                except Exception:
                    pass
            

        try:
            r = requests.get('http://' + self.signalk_host_port + '/signalk',
                             timeout=SIGNALK_HTTP_TIMEOUT,
                             verify=self.tls_verify.value)
            contents = pyjson.loads(r.content)
            ws_url = contents['endpoints']['v1']['signalk-ws']
            self.signalk_ws_url = ws_url + '?subscribe=none'
            # Derive the HTTP(S) base for access requests from the
            # advertised ws endpoint. An SSL server advertises
            # wss://host:3443 (https on the same authority) and redirects
            # the plain http://host:3000 we probed. Posting the access
            # request to the http URL would hit that redirect, and requests
            # downgrades a 302'd POST to a GET, silently breaking token
            # enrollment. Use the advertised scheme and authority instead.
            from urllib.parse import urlparse
            parsed = urlparse(ws_url)
            scheme = 'https' if parsed.scheme == 'wss' else 'http'
            self.signalk_http_base = scheme + '://' + parsed.netloc
        except Exception as e:
            print(_('failed to retrieve/parse data from'), self.signalk_host_port, e)
            time.sleep(5)
            self.signalk_host_port = False
            return
        print('signalk ' + _('found'), self.signalk_ws_url)

    def request_access(self):
        if requests is None:
            return
        if self.signalk_access_url:
            r = None
            try:
                r = requests.get(self.signalk_access_url,
                                 timeout=SIGNALK_HTTP_TIMEOUT,
                                 verify=self.tls_verify.value)
                try:
                    contents = pyjson.loads(r.content)
                except:
                    print("request_access failed json loads", r)
                    print("failed content", r.content)
                    raise
                debug('signalk ' + _('see if token is ready'), self.signalk_access_url, contents)
                if contents['state'] == 'COMPLETED':
                    if 'accessRequest' in contents:
                        access = contents['accessRequest']
                        if access['permission'] == 'APPROVED':
                            self.token = access['token']
                            print('signalk ' + _('received token'))  # never log the token itself
                            try:
                                # O_CREAT | O_WRONLY | O_TRUNC with restrictive perms so the token
                                # is never written with default world-readable perms.
                                fd = os.open(token_path,
                                             os.O_CREAT | os.O_WRONLY | os.O_TRUNC,
                                             0o600)
                                with os.fdopen(fd, 'w') as f:
                                    f.write(self.token)
                            except Exception as e:
                                print('signalk ' + _('failed to store token'), token_path, e)
                    else:
                        self.uid.set('pypilot') # re-enumerate a new ID
                        # if permission == DENIED should we try other servers??
                    self.signalk_access_url = False
            except requests.exceptions.Timeout:
                print('signalk ' + _('access request timed out, will retry'), self.signalk_access_url)
            except Exception as e:
                print('signalk ' + _('error requesting access'), e)
                import traceback
                print(traceback.format_exc())
                if r is not None:
                    print('content', r.content)
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
            # Use the scheme/authority discovered in probe_signalk so an
            # SSL server's access request is posted to https directly
            # rather than through the http->https redirect (which would
            # downgrade the POST to a GET). Fall back to the probed host
            # if probe has not run yet.
            base = self.signalk_http_base or ('http://' + self.signalk_host_port)
            r = requests.post(base + '/signalk/v1/access/requests',
                              # pypilot writes heading, attitude and rate of turn to
                              # SignalK: without "permissions" the server offers the
                              # request as read only, and an approval left at that
                              # default silently drops everything pypilot publishes.
                              data={"clientId": self.uid.value, "description": "pypilot",
                                    "permissions": "readwrite"},
                              timeout=SIGNALK_HTTP_TIMEOUT,
                              verify=self.tls_verify.value)

            contents = pyjson.loads(r.content)
            print('signalk post', contents)
            if r.status_code == 202 or r.status_code == 400:
                self.signalk_access_url = base + contents['href']
                print('signalk ' + _('request access url'), self.signalk_access_url)
            elif r.status_code == 404:
                print('signalk ' + _('security not enabled, connecting without token'))
                self.token = 'none'
        except Exception as e:
            print('signalk ' + _('error posting access'), e)
            import traceback
            print(traceback.format_exc())
            try:
                print('content', r.content)
            except Exception as e:
                print('no content', e)
            self.signalk_ws_url = False

    def invalid_token(self):
        self.token = False
        try:
            os.unlink(token_path)
        except OSError:
            pass # ignore

    def connect_signalk(self):
        self.subscribed = {}
        for sensor in list(signalk_table):
            self.subscribed[sensor] = False
        self.subscriptions = [] # track signalk subscriptions
        self.signalk_values = {}
        self.last_gps_fix = {}
        self.server_clock_offset = None
        self.keep_token = False

        global websocket
        if websocket is None:
            try:
                import websocket
            except ImportError:
                print('signalk ' + _('cannot create connection:') + ' websocket-client not installed')
                print(_('try') + ' pip3 install websocket-client ' + _('or') + ' apt install python3-websocket')
                self.signalk_host_port = False
                time.sleep(self._backoff_delay())
                return

        try:
            connect_kwargs = {}
            if self.token and self.token != 'none':
                connect_kwargs['header'] = {'Authorization': 'JWT ' + self.token}
            # For a wss:// stream with verification disabled, tell
            # websocket-client to skip certificate checks (self-signed
            # certs). It also clears check_hostname when cert_reqs is
            # CERT_NONE, so this is sufficient.
            if self.signalk_ws_url.startswith('wss://') and not self.tls_verify.value:
                import ssl
                connect_kwargs['sslopt'] = {'cert_reqs': ssl.CERT_NONE}
            self.ws = websocket.create_connection(self.signalk_ws_url, **connect_kwargs)
            self.ws.settimeout(0) # nonblocking
            self.connect_attempts = 0            
        except websocket.WebSocketBadStatusException as e:
            # Only 401/403 mean the token is bad. A server that is starting
            # or restarting answers 502/503: keep the token and retry, or
            # every SignalK restart costs a new access request that someone
            # has to approve again.
            status = getattr(e, 'status_code', None)
            if status in (401, 403):
                print('signalk ' + _('bad status, rejecting token'), e)
                self.invalid_token()
            else:
                print('signalk ' + _('bad status, keeping token'), e)
            self.ws = False
        except ConnectionRefusedError:
            print('signalk ' + _('connection refused'))
            self.signalk_ws_url = False
            time.sleep(self._backoff_delay())
        except Exception as e:
            print('signalk ' + _('failed to connect'), e)
            self.signalk_ws_url = False
            time.sleep(self._backoff_delay())

    def _backoff_delay(self):
        # Exponential backoff, capped. Counter resets on successful
        # connect (see connect_signalk) so transient flaps do not
        # permanently degrade reconnect speed.
        attempts = getattr(self, 'connect_attempts', 0)
        delay = min(SIGNALK_BACKOFF_MIN * (2 ** attempts), SIGNALK_BACKOFF_MAX)
        self.connect_attempts = attempts + 1
        return delay
            
    def process(self):
        # The subprocess inherits self.process pointing at the Process handle
        # the parent created. poll() uses that attribute to decide whether to
        # take the parent-side "drain pipe and return" branch or the worker
        # branch (probe / connect_signalk / send). Clear it locally so this
        # subprocess executes the worker branch; the parent's copy is
        # unaffected by the fork.
        self.process = False
        print('signalk process', os.getpid())
        time.sleep(6) # let other stuff load
        while True:
            time.sleep(.1)
            self.poll(1)

    def setup_watches(self):
        # setup pypilot watches
        if self.token:
            for watch in ('imu.heading_lowpass', 'imu.headingrate_lowpass', 'imu.roll', 'imu.pitch', 'gps.filtered.output'):
                self.client.watch(watch, self.period.value)
            self.client.watch('timestamp', self.period.value/2)

        for sensor in signalk_table:
            self.client.watch(sensor+'.source')

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

        if not self.enabled.value:
            return
          
        manual_host = self.signalk_host.value.strip()
        if manual_host and ':' not in manual_host:
            # signalk_host_port is consumed as "host:port" throughout this
            # module. A bare hostname from the user config defaults to the
            # SignalK HTTP port so configs like "localhost" keep working
            # without forcing the operator to remember the port.
            manual_host = manual_host + ':' + str(SIGNALK_DEFAULT_PORT)
        if manual_host:
            if manual_host != self.signalk_host_port:
                self.signalk_host_port = manual_host
                self.signalk_ws_url = False
                self.disconnect_signalk()
            self.last_manual_signalk_host = manual_host
        else:
            if self.last_manual_signalk_host:
                self.last_manual_signalk_host = False
                self.signalk_host_port = False
                self.signalk_ws_url = False
                self.disconnect_signalk()

        if not manual_host and self.zero_conf:
            zc = self.zero_conf.poll()
            if zc == 'disconnect':
                self.signalk_host_port = False
                self.disconnect_signalk()
            elif zc:
                host_port = zc
                self.signalk_host_port = host_port
                print('signalk ' + _('server found'), host_port)
        
        self.client.poll(timeout)
        if not self.signalk_host_port:
            return # waiting for signalk to detect

        t1 = time.monotonic()
        if not self.signalk_ws_url:
            self.probe_signalk()
            return

        t2 = time.monotonic()
        if not self.token and t0 - self.last_access_request_time >= 10:
            self.last_access_request_time = t0
            self.request_access()
            if self.token and self.ws:
                self.disconnect_signalk()  # Reconnect with the newly granted credentials.
            return # continue on next iteration

        if not self.ws:
            self.connect_signalk()
            if not self.ws:
                if not self.token:
                    time.sleep(SIGNALK_BACKOFF_MIN)
                return
            print('signalk ' + _('connected to'), self.signalk_ws_url)
            self.setup_watches()
            self.last_period = self.period.value
            return

        if self.last_period != self.period.value: # period changed
            self.disconnect_signalk()
            return

        # at this point we have a connection
        # read all messages from pypilot
        while True:
            msg = self.client.receive_single()
            if not msg:
                break
            #debug('signalk pypilot msg', msg)
            name, value = msg
            if name == 'timestamp':
                if self.token:
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
                self.gps_filtered_output = value
                self.client.watch('gps.fix', not value)
                self.client.watch('gps.filtered.fix', value)
            else:
                self.last_values[name] = value

        t3 = time.monotonic()

        t4 = time.monotonic()
        while True:
            try:
                msg = self.ws.recv()
            except Exception:
                break

            if not msg:
                print('signalk server closed connection')
                if not self.keep_token:
                    debug('signalk invalidating token')
                    self.invalid_token()
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
                if sensor == 'gps':
                    data = self.gps_fix(source, values)
                    if data and self.sensors_pipe:
                        self.sensors_pipe.send([sensor, data])
                    continue
                data = {}
                for signalk_path_conversion, pypilot_path in sensor_table.items():
                    signalk_path, signalk_conversion = signalk_path_conversion
                    if signalk_path in values and values[signalk_path] is not None:
                        try:
                            source_path = (source, signalk_path)
                            if 'timestamp' not in data and source_path in self.signalk_last_msg_time:
                                data['timestamp'] = _parse_signalk_timestamp(
                                    self.signalk_last_msg_time[source_path])

                            value = values[signalk_path]
                            if type(pypilot_path) == dict: # single path translates to multiple pypilot
                                for signalk_key, pypilot_key in pypilot_path.items():
                                    if value[signalk_key] is not None:
                                        data[pypilot_key] = value[signalk_key] / signalk_conversion
                            else:
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

    def gps_fix(self, source, values):
        # Signal K can send position (latitude and longitude), speed, and course in
        # separate WebSocket messages for the same fix. Match later parts by source
        # and timestamp; a stationary receiver may never report a course.
        position = 'navigation.position'
        speed = 'navigation.speedOverGround'
        course = 'navigation.courseOverGroundTrue'
        if not any(path in values for path in (position, speed, course)):
            return

        stamps = self.signalk_last_msg_time
        previous = self.last_gps_fix.get(source)
        if position in values:
            coords = values.pop(position)
            lat, lon = (coords.get('latitude'), coords.get('longitude')) if isinstance(coords, dict) else (None, None)
            if (type(lat) not in (int, float) or not -90 <= lat <= 90 or
                    type(lon) not in (int, float) or not -180 <= lon <= 180):
                self.last_gps_fix.pop(source, None)
                values.pop(speed, None)
                values.pop(course, None)
                return
            stamp = stamps[source, position]
            fix = {'lat': lat, 'lon': lon, 'speed': 0.0}
        elif previous and any(path in values and stamps[source, path] == previous[0]
                              for path in (speed, course)):
            # Signal K may deliver another part of the same fix in a later WebSocket frame.
            stamp, fix = previous
            fix = fix.copy()
        elif values.get(speed) is not None and values.get(course) is not None:
            # Preserve the velocity-only input supported by earlier versions.
            stamp = stamps[source, course]
            fix = {}
        else:
            return

        try:
            fix['timestamp'] = _parse_signalk_timestamp(stamp)
            if self.server_clock_offset is not None:
                age = time.monotonic() + self.server_clock_offset - fix['timestamp']
                if age >= GPS_TIMEOUT:
                    for path in (speed, course):
                        if stamps.get((source, path)) == stamp:
                            values.pop(path, None)
                    return
            for path, key, scale in ((speed, 'speed', meters_s), (course, 'track', radians)):
                if path not in values:
                    continue
                value = values.pop(path)
                # An initial subscription may return a course from long before this fix.
                dt = _parse_signalk_timestamp(stamps[source, path]) - fix['timestamp']
                if abs(dt) >= GPS_TIMEOUT or type(value) not in (int, float) or not math.isfinite(value):
                    fix.pop(key, None)
                else:
                    fix[key] = value / scale
        except (TypeError, ValueError, OverflowError):
            return
        if 'lat' in fix:
            fix.setdefault('speed', 0.0)  # Same default as a valid RMC with no speed.
            self.last_gps_fix[source] = stamp, fix.copy()
        elif 'speed' not in fix or 'track' not in fix:
            return
        fix['device'] = source + 'signalk'
        return fix

    def send_signalk(self):
        # see if we can produce any signalk output from the data we have read
        updates = []
        for sensor in signalk_table:
            if sensor != 'imu' and (sensor not in self.last_sources or\
                                    source_priority[self.last_sources[sensor]]>=signalk_priority):
                #debug('signalk skip send from priority', sensor)
                continue
            sensor_key = sensor
            if sensor == 'gps' and self.gps_filtered_output:
                sensor_key = 'gps.filtered'

            for signalk_path_conversion, pypilot_path in signalk_table[sensor].items():
                signalk_path, signalk_conversion = signalk_path_conversion
                if type(pypilot_path) == dict: # single path translates to multiple pypilot
                    # A GPS position must come from one fix, not coordinates
                    # accumulated from different samples.
                    keys = {} if sensor == 'gps' else self.last_values_keys[signalk_path]
                    # store keys we need for this signalk path in dictionary
                    for signalk_key, pypilot_key in pypilot_path.items():
                        key = sensor_key+'.'+pypilot_key
                        if sensor == 'gps':
                            kf = sensor_key+'.fix'
                            fix = self.last_values.get(kf)
                            if fix and pypilot_key in fix:
                                keys[key] = fix[pypilot_key]
                        else:
                            if key in self.last_values:
                                keys[key] = self.last_values[key]

                    # see if we have the keys needed
                    v = {}
                    for signalk_key, pypilot_key in pypilot_path.items():
                        key = sensor_key+'.'+pypilot_key
                        if not key in keys:
                            break
                        v[signalk_key] = keys[key]*signalk_conversion
                    else: # we have all the keys required
                        updates.append({'path': signalk_path, 'value': v})
                        self.last_values_keys[signalk_path] = {}
                else:
                    v = None
                    if sensor == 'gps': # for now gps fix is stored in dictionary
                        key = sensor_key+'.fix'
                        kv = self.last_values.get(key)
                        if kv and pypilot_path in kv:
                            v = kv[pypilot_path]
                    else:
                        key = sensor+'.'+pypilot_path
                        value = self.last_values.get(key)
                        if value is not None and value is not False:
                            v = value
                    if v is not None:
                        v *= signalk_conversion
                        updates.append({'path': signalk_path, 'value': v})

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
        # Defensive parsing: every field in a SignalK delta is technically
        # optional per the spec, so do not blindly index into the dict.
        try:
            data = pyjson.loads(msg)
        except Exception as e:
            if msg:
                print('signalk ' + _('failed to parse msg:'), e, repr(msg)[:200])
            return

        if not isinstance(data, dict):
            debug('signalk non-dict message', type(data))
            return

        if 'self' in data and 'version' in data and data.get('timestamp'):
            # The Pi's clock need not agree with the server's.
            try:
                self.server_clock_offset = _parse_signalk_timestamp(data['timestamp']) - time.monotonic()
            except (TypeError, ValueError, OverflowError):
                pass

        updates = data.get('updates')
        if not isinstance(updates, list):
            return

        for update in updates:
            if not isinstance(update, dict):
                continue

            source = 'unknown'
            src_field = update.get('$source')
            if isinstance(src_field, str):
                source = src_field
            else:
                src_dict = update.get('source')
                if isinstance(src_dict, dict):
                    source = src_dict.get('talker') or src_dict.get('label') or 'unknown'

            timestamp = update.get('timestamp')
            if not timestamp:
                # Without a timestamp we cannot tell duplicates from fresh
                # data, so skip entirely rather than mis-ordering.
                continue

            if source not in self.signalk_values:
                self.signalk_values[source] = {}

            # SignalK clients can PUT to paths this node is authoritative
            # for. pypilot, as a client, surfaces PUTs from the server as
            # delta updates with a `put` array. Act on them when the user
            # has opted in via signalk.put.enabled.
            puts = update.get('put')
            if isinstance(puts, list) and self.token:
                self._handle_signalk_put(puts, update.get('requestId'))

            values = update.get('values')
            if not isinstance(values, list):
                values = update.get('meta')
                if not isinstance(values, list):
                    debug('signalk update has no values/meta list', update)
                    continue

            for value in values:
                if not isinstance(value, dict):
                    continue
                path = value.get('path')
                if not isinstance(path, str) or 'value' not in value:
                    continue
                # Skip exact duplicate timestamps. Previously the initial
                # message for each path was silently discarded, which lost
                # the first value after every reconnect. Now we store the
                # value on first sight as well.
                source_path = (source, path)
                if self.signalk_last_msg_time.get(source_path) == timestamp:
                    debug('signalk skip duplicate timestamp', source, path, timestamp)
                    continue
                self.signalk_values[source][path] = value['value']
                self.signalk_last_msg_time[source_path] = timestamp

    def _handle_signalk_put(self, puts, request_id):
        if not self.put_enabled.value:
            self._put_response(request_id, 'COMPLETED', 403,
                               'pypilot signalk.put.enabled is false')
            return
        statuses = []
        for put in puts:
            if not isinstance(put, dict):
                continue
            path = put.get('path')
            if not isinstance(path, str) or 'value' not in put:
                statuses.append((path, 400, 'malformed put entry'))
                continue
            mapping = signalk_put_table.get(path)
            if mapping is None:
                statuses.append((path, 404, 'unsupported path'))
                continue
            pypilot_name, conversion = mapping
            try:
                raw = put['value']
                if isinstance(raw, (int, float)) and conversion != 1:
                    raw = raw / conversion
                self.client.set(pypilot_name, raw)
                statuses.append((path, 200, 'ok'))
            except Exception as e:
                print('signalk put failed', path, e)
                statuses.append((path, 500, str(e)))
        # SignalK allows grouped responses; we emit one COMPLETED per put.
        for _path, code, message in statuses:
            self._put_response(request_id, 'COMPLETED', code, message)

    def _put_response(self, request_id, state, status_code, message):
        if not self.ws or not request_id:
            return
        try:
            response = {
                'requestId': request_id,
                'state': state,
                'statusCode': status_code,
                'message': message,
            }
            self.ws.send(pyjson.dumps(response) + '\n')
        except Exception as e:
            debug('signalk put response send failed', e)


    def update_sensor_source(self, sensor, source):
        priority = source_priority[source]
        watch = priority < signalk_priority and bool(self.token) # translate from pypilot -> signalk
        if watch:
            watch = self.period.value

        if sensor == 'gps': # the fix contains everything
            self.client.watch('gps.fix', watch)
        else:
            for signalk_path_conversion, pypilot_path in signalk_table[sensor].items():
                if type(pypilot_path) == dict:
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
            # Node signalk-server only accepts the wildcard sentinel and
            # throws on per-path unsubscribe. Clear everything here; the
            # re-subscribe below restores the sensors we still want.
            unsubscribe_all = {'context': '*', 'unsubscribe': [{'path': '*'}]}
            debug('signalk unsubscribe', unsubscribe_all)
            try:
                self.ws.send(pyjson.dumps(unsubscribe_all)+'\n')
            except Exception as e:
                print('signalk failed to send', e)
                self.disconnect_signalk()
                return

        signalk_sensor = signalk_table[sensor]
        if subscribe: # translate from signalk -> pypilot
            if sensor == 'gps':
                self.last_gps_fix.clear()
            subscriptions = []
            for signalk_path_conversion in signalk_sensor:
                signalk_path, signalk_conversion = signalk_path_conversion
                for source_path in list(self.signalk_last_msg_time):
                    if source_path[1] == signalk_path:
                        del self.signalk_last_msg_time[source_path]
                        self.signalk_values.get(source_path[0], {}).pop(signalk_path, None)
                subscriptions.append({
                    'path': signalk_path,
                    'minPeriod': int(self.period.value * 1000),
                    'format': 'delta',
                    'policy': self.subscribe_policy.value,
                })
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
            self.signalk_values.clear()
            self.last_gps_fix.clear()

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
