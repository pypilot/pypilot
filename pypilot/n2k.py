#!/usr/bin/env python

# NMEA2000 bridge for pypilot

from __future__ import annotations

import asyncio
import calendar
import datetime
import multiprocessing
import select
import time
import version

from client import pypilotClient
from nonblockingpipe import NonBlockingPipe
from values import EnumProperty, Property, RangeProperty, StringValue
import logging

# a missing or broken nmea2000 install disables N2K rather than pypilot
try:
    import nmea2000
    import n2k_control
    import_error = None
except Exception as e:
    nmea2000 = None
    import_error = e

log = logging.getLogger('n2k')


def gps_timestamp(message):
    '''epoch seconds from a message's date and time fields, as the other gps inputs report'''
    try:
        date = message.get_field_by_id('date').value
        time_of_day = message.get_field_by_id('time').value
    except Exception:
        return None
    if not isinstance(date, datetime.date) or not isinstance(time_of_day, datetime.time):
        return None
    return calendar.timegm(datetime.datetime.combine(date, time_of_day).timetuple()) + \
        time_of_day.microsecond / 1e6


class N2KBridge(object):
    # The persistent n2k.* settings arrive from the server some time after
    # they are registered.  Starting the transport with the defaults first
    # would start it twice at boot, so wait until data is arriving from the
    # server and the transport settings have stopped changing.
    SETTINGS_SETTLE_TIME = 1.0
    SETTINGS_MAX_WAIT = 10.0

    def __init__(self, server):
        self.client = pypilotClient(server)
        self.client.connection.name += 'n2kbridge'
        self.wake_event = None  # created in the bridge's process, by n2k_process
        self.multiprocessing = server.multiprocessing
        self.pipe, self.pipe_out = NonBlockingPipe('n2k pipe', self.multiprocessing)
        # Require multiprocessing mode — N2K runs only in a separate process.
        if not self.multiprocessing:
            raise RuntimeError('N2KBridge requires server multiprocessing to be enabled')

        self.process = multiprocessing.Process(target=lambda: asyncio.run(self.n2k_process()), daemon=True)
        self.process.start()

    def set_status(self, value, error=''):
        print("N2KBridge: Status update: %s, error: %s" % (value, error))
        self.n2k_status.set(value)
        self.n2k_error.set(error)

    async def setup(self):
        self.n2k_interface = self.client.register(Property('n2k.interface', 'can0', persistent=True))
        self.n2k_transport = self.client.register(EnumProperty('n2k.transport', 'none',
                                                               ['none', 'socketcan', 'actisense', 'ebyte', 'usb'],
                                                               persistent=True))
        self.n2k_host = self.client.register(Property('n2k.host', '', persistent=True))
        self.n2k_port = self.client.register(Property('n2k.port', 0, persistent=True))
        self.n2k_usb_device = self.client.register(Property('n2k.usb_device', '', persistent=True))
        self.n2k_name = self.client.register(Property('n2k.name', 'pypilot', persistent=True))
        self.n2k_status = self.client.register(StringValue('n2k.status', 'initializing'))
        self.n2k_error = self.client.register(StringValue('n2k.error', ''))

        self.n2k_output_enable = {
            'attitude': self.client.register(Property('n2k.output.attitude', True, persistent=True)),
            'heading': self.client.register(Property('n2k.output.heading', True, persistent=True)),
            'rate_of_turn': self.client.register(Property('n2k.output.rate_of_turn', True, persistent=True)),
            'autopilot': self.client.register(Property('n2k.output.autopilot', True, persistent=True)),
            'wind': self.client.register(Property('n2k.output.wind', False, persistent=True)),
            'rudder': self.client.register(Property('n2k.output.rudder', False, persistent=True)),
            'gps_filtered': self.client.register(Property('n2k.output.gps_filtered', False, persistent=True)),
        }

        default_pgn_filters = [129029, 129026, 129033, 129283, 129284, 130306, 127245, 128259]
        self.pgn_filters = self.client.register(Property('n2k.pgn_filters',
            default_pgn_filters, persistent=True))

        # control of pypilot over n2k, see docs/n2k-control.md
        control_settings = {
            'control': self.client.register(EnumProperty('n2k.control', 'monitor',
                                                         n2k_control.CONTROL_LEVELS, persistent=True)),
            # -1 is off. A range so clients can edit it: an enum of every instance
            # would overflow the clients' 16 KB line buffer for the values list.
            'bank': self.client.register(RangeProperty('n2k.switch.bank', -1, -1, 252, step=1,
                                                       persistent=True)),
            'allowed': self.client.register(Property('n2k.control.allowed', [], persistent=True)),
            'jog_pulse': self.client.register(RangeProperty('n2k.control.jog_pulse', .3, .1, 1, persistent=True)),
            'description': self.client.register(Property('n2k.installation_description', '', persistent=True)),
            'output': self.n2k_output_enable['autopilot'],
        }

        names = ['gps.source', 'wind.source', 'truewind.source', 'rudder.source',
                 'apb.source', 'water.source', 'gps.filtered.output',
                 'ap.mode', 'ap.enabled', 'ap.heading_command']
        self.last_values = {'gps.filtered.output': False,
                            'ap.mode': 'compass',
                            'ap.enabled': False,
                            'ap.heading_command': 0}
        for name in names:
            if not name in self.last_values:
                self.last_values[name] = 'none'

        self.watch_gps_fix = False
        self.sid = 0
        self.gateway = None
        self.transport_config = None  # transport not started yet
        self.pending_transport_config = None
        self.settings_start_time = self.pending_time = time.monotonic()
        self.poller = select.poll()
        self.fd_to_source = {}
        self.msgs = {}
        self.last_imu_time = time.monotonic()
        self.gps_devices = {}
        self.nav_devices = {}
        self.control = n2k_control.N2KControl(self.client, self.last_values, control_settings)

        self.setup_watches()

    def setup_watches(self):
        for name in self.last_values:
            self.client.watch(name)

        watchlist = ['imu.pitch', 'imu.roll', 'imu.heading_lowpass',
                     'imu.headingrate_lowpass', 'wind.direction', 'wind.speed',
                     'truewind.direction', 'truewind.speed', 'rudder.angle']
        for name in watchlist:
            self.client.watch(name)

        for name in ['wind.rate', 'truewind.rate', 'rudder.rate']:
            self.client.watch(name)

        for name in n2k_control.WATCHES:
            self.client.watch(name)

        self.update_gps_fix_watch()

    def update_gps_fix_watch(self):
        watch = self.last_values.get('gps.filtered.output') and self.n2k_output_enable['gps_filtered'].value
        if watch != self.watch_gps_fix:
            self.client.watch('gps.filtered.fix', watch)
            self.watch_gps_fix = watch

    def next_sid(self):
        # SID ties together PGNs derived from the same sample; 0-252 are valid, 255 means unavailable
        self.sid = (self.sid + 1) % 253
        return self.sid

    def current_transport_config(self):
        pgn_filters = self.pgn_filters.value
        if type(pgn_filters) == type([]):
            pgn_filters = tuple(pgn_filters)
        else:
            pgn_filters = ()
        return (self.n2k_transport.value, self.n2k_interface.value, self.n2k_host.value,
                int(self.n2k_port.value or 0), self.n2k_usb_device.value, pgn_filters,
                tuple(self.get_transmit_pgns()), tuple(self.control.include_pgns()))

    def get_transmit_pgns(self):
        pgns = []
        if self.n2k_output_enable['attitude'].value:
            pgns.append(127257)
        if self.n2k_output_enable['heading'].value:
            pgns.append(127250)
        if self.n2k_output_enable['rate_of_turn'].value:
            pgns.append(127251)
        if self.n2k_output_enable['rudder'].value:
            pgns.append(127245)
        return pgns + self.control.transmit_pgns()

    def transport_include_pgns(self):
        pgn_filters = self.pgn_filters.value
        if not isinstance(pgn_filters, list):
            pgn_filters = []

        include_pgns = list(pgn_filters)
        for pgn in self.control.include_pgns():
            if pgn not in include_pgns:
                include_pgns.append(pgn)
        return include_pgns

    async def handle_gateway_status(self, state):
        state_name = getattr(state, 'name', str(state))
        if state_name == 'CONNECTED':
            self.set_status('connected')
        elif state_name == 'DISCONNECTED':
            self.set_status('disconnected')
        elif state_name == 'CLOSED':
            self.set_status('closed')

    async def init_transport(self):
        print("N2KBridge: Initializing transport...")
        await self.close_gateway()
        self.transport_config = self.current_transport_config()

        transport = self.n2k_transport.value
        if transport == 'none':
            self.set_status('disabled')
            return

        include_pgns = self.transport_include_pgns()

        device_options = {
            "preferred_address": 100,
            "manufacturer_code": 999,
            "device_function": 150,
            "device_class": 40,
            "model_id": self.n2k_name.value or "PyPilot",
            "model_version": version.strversion,
            "manufacturer_information": "PyPilot Autopilot",
            "installation_description1": self.control.settings['description'].value or '',
            "installation_description2": self.control.description2,
            "transmit_pgns": self.get_transmit_pgns(),
            "receive_pgns": include_pgns,
            "address_claim_startup_delay": 0.1,
            "address_claim_detection_time": 0.25,
            "heartbeat_interval": 60.0,
            "persistence_key": self.n2k_name.value or 'pypilot',
        }

        transport_classes = {
            'socketcan': (nmea2000.N2KDevice.for_python_can, {"interface": "socketcan", "channel": self.n2k_interface.value or "can0", "client_options": {"include_pgns": include_pgns}, **device_options}),
            'actisense': (nmea2000.N2KDevice.for_actisense, {"host": self.n2k_host.value, "port": int(self.n2k_port.value or 0), "client_options": {"include_pgns": include_pgns}, **device_options}),
            'ebyte': (nmea2000.N2KDevice.for_ebyte, {"host": self.n2k_host.value, "port": int(self.n2k_port.value or 0), "client_options": {"include_pgns": include_pgns}, **device_options}),
            'usb': (nmea2000.N2KDevice.for_waveshare, {"port": self.n2k_usb_device.value, "client_options": {"include_pgns": include_pgns}, **device_options}),
        }

        transport_class, transport_kwargs = transport_classes.get(transport, (None, None))

        if not transport_class:
            self.set_status('invalid transport', 'unsupported transport: %s' % transport)
            return

        try:
            self.gateway = transport_class(**transport_kwargs)
            self.gateway.set_receive_callback(self.parse_pgn)
            self.gateway.set_status_callback(self.handle_gateway_status)
            self.attach_control()
            await self.gateway.start()
        except Exception as e:
            self.set_status('transport error', str(e))
            return

        self.set_status('initializing')

    def attach_control(self):
        self.control.attach(self.gateway)
        # these hooks are needed for 127237 commands and PP: text commands;
        # without them the library NAKs 126208 commands and requests itself
        missing = []
        for setter, handler in [('set_group_function_handler', self.handle_group_function),
                                ('set_iso_request_handler', self.handle_iso_request)]:
            register = getattr(self.gateway, setter, None)
            if register:
                register(handler)
            else:
                missing.append(setter)
        if missing:
            print('N2KBridge: nmea2000 library has no %s; 127237 commands and PP: text commands are unavailable'
                  % ', '.join(missing))

    async def handle_group_function(self, message):
        handled = self.control.handle_group_function(message)
        await self.send_control_messages(self.control.outbox)
        self.control.outbox = []
        self.wake()
        return handled

    async def handle_iso_request(self, message, pgn):
        handled = self.control.handle_iso_request(message, pgn)
        await self.send_control_messages(self.control.outbox)
        self.control.outbox = []
        self.wake()
        return handled

    async def send_control_messages(self, messages):
        for message in messages:
            await self.send_message(message)

    async def close_gateway(self):
        if self.gateway:
            try:
                await self.gateway.close()
            finally:
                self.gateway = None

    def settings_settled(self, config):
        t = time.monotonic()
        if config != self.pending_transport_config or not self.client.received_data:
            self.pending_transport_config = config
            self.pending_time = t
        if t - self.settings_start_time >= self.SETTINGS_MAX_WAIT:
            return True
        return t - self.pending_time >= self.SETTINGS_SETTLE_TIME

    async def ensure_transport(self):
        current = self.current_transport_config()
        if self.transport_config is None and not self.settings_settled(current):
            return
        if current != self.transport_config:
            await self.init_transport()

    def update_gps_message(self, device, updates):
        gps = self.gps_devices.get(device, {'device': device, 'speed': 0})
        gps.update(updates)
        self.gps_devices[device] = gps
        if not 'lat' in gps or not 'lon' in gps:
            return

        msg = {'device': device, 'lat': gps['lat'], 'lon': gps['lon'], 'speed': gps.get('speed', 0)}
        for name in ['track', 'timestamp', 'declination']:
            if name in gps:
                msg[name] = gps[name]
        self.msgs['gps'] = msg

    def course_reference_mode(self, reference):
        if reference is None:
            return 'gps'
        reference = str(reference).lower()
        if 'mag' in reference or reference == 'm':
            return 'compass'
        return 'gps'

    def apb_xte(self, xte):
        # N2K XTE in meters is positive right of track (steer left),
        # pypilot's apb xte in nautical miles is positive to steer right
        return -float(xte) / 1852.0

    def update_apb_message(self, device, updates):
        nav = self.nav_devices.get(device, {'device': device})
        nav.update(updates)
        self.nav_devices[device] = nav
        if not 'track' in nav:
            return

        msg = {'device': device, 'track': nav['track']}
        if 'xte' in nav:
            msg['xte'] = nav['xte']
        if 'mode' in nav:
            msg['mode'] = nav['mode']
        self.msgs['apb'] = msg

    def set_client_value(self, name, value, epsilon=None):
        current = self.last_values.get(name)
        if epsilon is not None:
            try:
                if abs(float(current) - float(value)) <= epsilon:
                    return
            except Exception:
                pass
        elif current == value:
            return
        self.client.set(name, value)
        self.last_values[name] = value

    async def parse_pgn(self, message: nmea2000.NMEA2000Message):
        if self.control.handle_message(message):
            self.wake()  # send what it set to pypilot now
            return
        device = 'N2K%d' % message.source if message.source is not None else 'N2K'

        if message.PGN == 130306:
            try:
                angle_rad = message.get_field_by_id('windAngle').value
                speed_ms = message.get_field_by_id('windSpeed').value
                ref = message.get_field_by_id('reference').value or 'Apparent'
            except Exception:
                return
            if angle_rad is None or speed_ms is None:
                return
            direction_deg = angle_rad * 180.0 / 3.141592653589793
            speed_knots = speed_ms * 1.94384
            if ref == 'Apparent':
                name = 'wind'
            elif ref in ('True (boat referenced)', 'True (water referenced)'):
                name = 'truewind'
            else:
                return  # ground referenced wind is a direction over ground, not an angle off the bow
            self.msgs[name] = {'direction': direction_deg, 'speed': speed_knots, 'device': device}
            return

        if message.PGN == 127245:
            try:
                pos_rad = message.get_field_by_id('position').value
            except Exception:
                return
            if pos_rad is None:
                return
            angle_deg = pos_rad * 180.0 / 3.141592653589793
            self.msgs['rudder'] = {'angle': -angle_deg, 'device': device}
            return

        if message.PGN == 129029:
            try:
                lat = message.get_field_by_id('latitude').value
                lon = message.get_field_by_id('longitude').value
            except Exception:
                return
            if lat is None or lon is None:
                return
            updates = {'lat': lat, 'lon': lon}
            try:
                sog_ms = message.get_field_by_id('sog').value
            except Exception:
                sog_ms = None
            try:
                cog_rad = message.get_field_by_id('cog').value
            except Exception:
                cog_rad = None
            timestamp = gps_timestamp(message)
            if sog_ms is not None:
                updates['speed'] = sog_ms * 1.94384
            if cog_rad is not None:
                updates['track'] = cog_rad * 180.0 / 3.141592653589793
            if timestamp is not None:
                updates['timestamp'] = timestamp
            self.update_gps_message(device, updates)
            return

        if message.PGN == 129026:
            updates = {}
            try:
                sog_ms = message.get_field_by_id('sog').value
            except Exception:
                sog_ms = None
            try:
                cog_rad = message.get_field_by_id('cog').value
            except Exception:
                cog_rad = None
            if sog_ms is not None:
                updates['speed'] = sog_ms * 1.94384
            if cog_rad is not None:
                updates['track'] = cog_rad * 180.0 / 3.141592653589793
            if updates:
                self.update_gps_message(device, updates)
            return

        if message.PGN == 129033:
            timestamp = gps_timestamp(message)
            if timestamp is not None:
                self.update_gps_message(device, {'timestamp': timestamp})
            return

        if message.PGN == 128259:
            try:
                speed_ms = message.get_field_by_id('speedWaterReferenced').value
            except Exception:
                speed_ms = None
            if speed_ms is None:
                return
            self.msgs['water'] = {'speed': speed_ms * 1.94384, 'device': device}
            return

        if message.PGN == 129283:
            try:
                xte = message.get_field_by_id('xte').value
            except Exception:
                xte = None
            if xte is None:
                return
            self.update_apb_message(device, {'xte': self.apb_xte(xte)})
            return

        if message.PGN == 129284:
            try:
                track = message.get_field_by_id('bearingPositionToDestinationWaypoint').value
            except Exception:
                track = None
            if track is None:
                return
            try:
                course_ref = message.get_field_by_id('courseBearingReference').value
            except Exception:
                course_ref = None
            updates = {'track': track * 180.0 / 3.141592653589793,
                       'mode': self.course_reference_mode(course_ref)}
            try:
                xte = message.get_field_by_id('xte').value
            except Exception:
                xte = None
            if xte is not None:
                updates['xte'] = self.apb_xte(xte)
            self.update_apb_message(device, updates)
            return


    async def send_message(self, message: nmea2000.NMEA2000Message):
        if not self.gateway or not self.gateway.ready:
            return False
        try:
            await self.gateway.send(message)
            return True
        except Exception as e:
            self.set_status('n2k output error', str(e))
            return False

    def publish_msgs(self):
        if self.msgs and self.pipe.send(self.msgs):
            self.msgs = {}

    async def output_pgns(self):
        # control timers (jog pulses, pending engage) run even without a bus
        control_messages = self.control.poll()
        if not self.gateway or not self.gateway.ready:
            return

        values = self.last_values
        t = time.monotonic()

        if self.n2k_output_enable['attitude'].value or self.n2k_output_enable['heading'].value or \
           self.n2k_output_enable['rate_of_turn'].value or self.n2k_output_enable['rudder'].value:
            if t - self.last_imu_time > 0.1:
                sid = self.next_sid()
                if self.n2k_output_enable['attitude'].value and 'imu.pitch' in values and 'imu.roll' in values:
                    pitch_rad = values['imu.pitch'] * 3.141592653589793 / 180.0
                    roll_rad = values['imu.roll'] * 3.141592653589793 / 180.0
                    # PGN 127257 requires: sid, yaw, pitch, roll, reserved_56
                    fields = [
                        nmea2000.NMEA2000Field(id='sid', value=sid),
                        nmea2000.NMEA2000Field(id='yaw', value=0.0),
                        nmea2000.NMEA2000Field(id='pitch', value=pitch_rad),
                        nmea2000.NMEA2000Field(id='roll', value=roll_rad),
                        nmea2000.NMEA2000Field(id='reserved_56', value=0),
                    ]
                    message = nmea2000.NMEA2000Message(PGN=127257, source=0, destination=255, priority=3, fields=fields)
                    await self.send_message(message)

                if self.n2k_output_enable['heading'].value and 'imu.heading_lowpass' in values:
                    heading_rad = values['imu.heading_lowpass'] * 3.141592653589793 / 180.0
                    # Include required PGN 127250 fields: sid, heading, deviation, variation, reference, reserved_58
                    fields = [
                        nmea2000.NMEA2000Field(id='sid', value=sid),
                        nmea2000.NMEA2000Field(id='heading', value=heading_rad),
                        nmea2000.NMEA2000Field(id='deviation', value=0.0),
                        nmea2000.NMEA2000Field(id='variation', value=0.0),
                        nmea2000.NMEA2000Field(id='reference', value='Magnetic'),
                        nmea2000.NMEA2000Field(id='reserved_58', value=0),
                    ]
                    message = nmea2000.NMEA2000Message(PGN=127250, source=0, destination=255, priority=3, fields=fields)
                    await self.send_message(message)

                if self.n2k_output_enable['rate_of_turn'].value and 'imu.headingrate_lowpass' in values:
                    rot_rad_per_s = values['imu.headingrate_lowpass'] * 3.141592653589793 / 180.0
                    # PGN 127251 requires: sid, rate, reserved_40
                    fields = [
                        nmea2000.NMEA2000Field(id='sid', value=sid),
                        nmea2000.NMEA2000Field(id='rate', value=rot_rad_per_s),
                        nmea2000.NMEA2000Field(id='reserved_40', value=0),
                    ]
                    message = nmea2000.NMEA2000Message(PGN=127251, source=0, destination=255, priority=3, fields=fields)
                    await self.send_message(message)

                if self.n2k_output_enable['rudder'].value:
                    message = self.control.rudder()
                    if message:
                        await self.send_message(message)

                self.last_imu_time = t

        await self.send_control_messages(control_messages)

    def wake(self):
        '''run the next poll now rather than after its timeout'''
        event = getattr(self, 'wake_event', None)
        if event:
            event.set()

    def wake_on_pypilot(self):
        '''wake when pypilot sends us something, so changes reach the bus at once'''
        fileno = getattr(getattr(self.client, 'connection', None), 'fileno', None)
        fd = fileno() if fileno else None
        if fd:
            asyncio.get_running_loop().add_reader(fd, self.wake)

    async def poll(self, timeout=100):
        event = getattr(self, 'wake_event', None)
        if event:
            event.clear()  # anything arriving from here on runs the next poll
        await self.ensure_transport()

        self.publish_msgs()

        pypilot_msgs = self.client.receive()
        for name in pypilot_msgs:
            value = pypilot_msgs[name]
            self.last_values[name] = value
            self.control.on_value(name, value)
            if name == 'gps.filtered.output' or name == 'n2k.output.gps_filtered':
                self.update_gps_fix_watch()

        await self.output_pgns()

        if timeout:
            # until pypilot or a control message on the bus needs us, for
            # commands to take effect and be confirmed without waiting a poll
            if event:
                try:
                    await asyncio.wait_for(event.wait(), timeout / 1000.0)
                except asyncio.TimeoutError:
                    pass
            else:
                await asyncio.sleep(timeout / 1000.0)


    async def n2k_process(self):
        await self.setup()
        self.wake_event = asyncio.Event()
        self.wake_on_pypilot()
        while True:
            await self.poll(100)



class N2K(object):
    def __init__(self, sensors):
        self.client = sensors.client
        self.sensors = sensors
        self.n2k_bridge = None
        if import_error:
            print('N2K: disabled, nmea2000 failed to import: %s' % import_error)
            return
        if not self.client.server.multiprocessing:
            print('N2K: disabled, server multiprocessing is off')
            return
        self.n2k_bridge = N2KBridge(self.client.server)
        self.process = self.n2k_bridge.process
        self.pipe = self.n2k_bridge.pipe_out
        self.poller = select.poll()
        self.process_fd = self.pipe.fileno()
        self.poller.register(self.process_fd, select.POLLIN)

    def read_process_pipe(self):
        while True:
            msgs = self.pipe.recv()
            if not msgs:
                return
            if isinstance(msgs, dict):
                for name in msgs:
                    self.sensors.write(name, msgs[name], 'can')

    def poll(self):
        if not self.n2k_bridge:
            return
        while True:
            events = self.poller.poll(0)
            if not events:
                break
            for fd, flag in events:
                if fd == self.process_fd and flag == select.POLLIN:
                    self.read_process_pipe()
