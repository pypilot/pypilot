import json
from datetime import datetime, timezone
from types import SimpleNamespace

import signalk


def test_two_sources_can_publish_same_path_at_same_time():
    obj = signalk.signalk.__new__(signalk.signalk)
    obj.signalk_values = {}
    obj.signalk_last_msg_time = {}
    path = 'navigation.speedOverGround'

    def receive(source, value, stamp='2026-07-15T12:00:00Z'):
        obj.receive_signalk(json.dumps({'updates': [{'$source': source,
            'timestamp': stamp, 'values': [{'path': path, 'value': value}]}]}))

    receive('receiver-a', 1)
    receive('receiver-b', 2)
    assert obj.signalk_values['receiver-a'][path] == 1
    assert obj.signalk_values['receiver-b'][path] == 2
    receive('receiver-a', 99)
    assert obj.signalk_values['receiver-a'][path] == 1
    receive('receiver-a', 3, '2026-07-15T12:00:01Z')
    assert obj.signalk_values['receiver-a'][path] == 3
    assert obj.signalk_values['receiver-b'][path] == 2


def test_converted_gps_fix_keeps_its_own_source_time():
    obj = signalk.signalk.__new__(signalk.signalk)
    obj.signalk_values = {}
    obj.signalk_last_msg_time = {}
    obj.last_gps_fix = {}
    obj.server_clock_offset = None
    for source, second in [('a', 0), ('b', 1)]:
        obj.receive_signalk(json.dumps({'updates': [{'$source': source,
            'timestamp': f'2026-07-15T12:00:0{second}Z',
            'values': [
                {'path': 'navigation.speedOverGround', 'value': 2},
                {'path': 'navigation.courseOverGroundTrue', 'value': 1},
                {'path': 'navigation.position', 'value': {'latitude': 50, 'longitude': -1}},
            ]}]}))
    obj.process = False
    obj.initialized = True
    obj.enabled = SimpleNamespace(value=True)
    obj.signalk_host = SimpleNamespace(value='')
    obj.last_manual_signalk_host = False
    obj.zero_conf = False
    obj.signalk_host_port = 'test:3000'
    obj.signalk_ws_url = 'ws://unused'
    obj.token = 'test'
    obj.period = SimpleNamespace(value=1)
    obj.last_period = 1
    obj.client = SimpleNamespace(poll=lambda timeout: None, receive_single=lambda: None)

    def no_messages():
        raise BlockingIOError()

    obj.ws = SimpleNamespace(recv=no_messages)
    sent = []
    obj.sensors_pipe = SimpleNamespace(send=sent.append)
    obj.poll()
    obj.poll()
    gps = [data for sensor, data in sent if sensor == 'gps']
    assert [data['device'] for data in gps] == ['asignalk', 'bsignalk']
    expected = datetime(2026, 7, 15, 12, tzinfo=timezone.utc).timestamp()
    assert [data['timestamp'] for data in gps] == [expected, expected + 1]


def test_stopped_fix_and_separately_delivered_course():
    obj = signalk.signalk.__new__(signalk.signalk)
    obj.signalk_values = {}
    obj.signalk_last_msg_time = {}
    obj.last_gps_fix = {}
    obj.server_clock_offset = None

    def fix(second, fields):
        obj.receive_signalk(json.dumps({'updates': [{'$source': 'receiver',
            'timestamp': f'2026-07-15T12:00:{second:02}Z',
            'values': [{'path': 'navigation.' + name, 'value': value}
                       for name, value in fields.items()]}]}))
        return obj.gps_fix('receiver', obj.signalk_values['receiver'])

    pos = {'latitude': 0, 'longitude': 0}
    stopped = fix(0, {'position': pos, 'speedOverGround': 0})
    assert stopped['speed'] == 0 and 'track' not in stopped
    assert stopped['lat'] == stopped['lon'] == 0
    assert obj.gps_fix('receiver', obj.signalk_values['receiver']) is None
    assert fix(0, {'courseOverGroundTrue': 0})['track'] == 0
    fresh = fix(1, {'position': pos})
    assert 'track' not in fresh and fresh['speed'] == 0
    assert fix(1, {'speedOverGround': signalk.meters_s})['speed'] == 1
    assert fix(2, {'position': {'latitude': None, 'longitude': 2}}) is None
    obj.receive_signalk(json.dumps({'self': 'vessels.example', 'version': '1.0.0',
                                    'timestamp': '2026-07-15T12:00:45Z'}))
    assert fix(3, {'position': pos, 'speedOverGround': 2}) is None


def test_resubscribing_clears_all_sources_for_that_sensor():
    path = 'environment.wind.speedApparent'
    obj = signalk.signalk.__new__(signalk.signalk)
    obj.signalk_values = {}
    obj.signalk_last_msg_time = {}
    for source, stamp in [('a', 'time-a'), ('b', 'time-b')]:
        obj.receive_signalk(json.dumps({'updates': [{'$source': source,
            'timestamp': stamp, 'values': [{'path': path, 'value': 1}]}]}))
    obj.client = SimpleNamespace(watch=lambda *args: None)
    obj.period = SimpleNamespace(value=1)
    obj.subscribe_policy = SimpleNamespace(value='instant')
    obj.subscribed = {'wind': False}
    obj.subscriptions = []
    obj.last_values = {}
    obj.ws = SimpleNamespace(send=lambda message: None)
    obj.update_sensor_source('wind', 'none')
    for source, stamp in [('a', 'time-a'), ('b', 'time-b')]:
        obj.receive_signalk(json.dumps({'updates': [{'$source': source,
            'timestamp': stamp, 'values': [{'path': path, 'value': 2}]}]}))
        assert obj.signalk_values[source][path] == 2
