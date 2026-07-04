"""
Tests for SignalK delta parsing and PUT handling.

These tests avoid the zeroconf / multiprocessing startup path by
instantiating only the pure parsing logic from `signalk.signalk` via a
stub that bypasses `__init__`.
"""
import pytest


@pytest.fixture
def signalk_instance(monkeypatch):
    """Return a `signalk` object with just enough state to exercise
    `receive_signalk` and `_handle_signalk_put`."""
    import signalk as sk

    obj = sk.signalk.__new__(sk.signalk)
    obj.signalk_values = {}
    obj.signalk_last_msg_time = {}
    obj.ws = None  # PUT responses are no-ops without a socket
    # Properties normally registered in setup(); tests can override.
    class _Bool:
        def __init__(self, v): self.value = v
    obj.put_enabled = _Bool(False)

    class _FakeClient:
        def __init__(self): self.writes = []
        def set(self, name, value): self.writes.append((name, value))
    obj.client = _FakeClient()
    return obj


def test_parses_basic_delta(signalk_instance):
    msg = (
        '{"updates":[{"$source":"test","timestamp":"2026-04-19T00:00:00.000Z",'
        '"values":[{"path":"navigation.speedOverGround","value":5.14}]}]}'
    )
    signalk_instance.receive_signalk(msg)
    assert signalk_instance.signalk_values['test']['navigation.speedOverGround'] == 5.14


def test_skips_duplicate_timestamp(signalk_instance):
    ts = '2026-04-19T00:00:00.000Z'

    def body(v):
        return (
            '{"updates":[{"$source":"test","timestamp":"' + ts + '",'
            '"values":[{"path":"navigation.speedOverGround","value":'
            + str(v) + '}]}]}'
        )

    signalk_instance.receive_signalk(body(1.0))
    signalk_instance.receive_signalk(body(2.0))  # same ts, different value
    # First write wins because the second has the duplicate timestamp.
    assert signalk_instance.signalk_values['test']['navigation.speedOverGround'] == 1.0


def test_initial_message_is_not_silently_dropped(signalk_instance):
    """Regression: pre-fix code logged "skip initial message" and did
    not store the first value per path after each reconnect."""
    msg = (
        '{"updates":[{"$source":"test","timestamp":"2026-04-19T00:00:01.000Z",'
        '"values":[{"path":"navigation.position","value":{"latitude":37.0,'
        '"longitude":-122.0}}]}]}'
    )
    signalk_instance.receive_signalk(msg)
    assert 'navigation.position' in signalk_instance.signalk_values['test']


def test_malformed_delta_does_not_raise(signalk_instance):
    # Missing "values", non-dict "source", missing path/value — each
    # previously could raise. The parser must simply drop them.
    for msg in [
        '{"updates":[{}]}',
        '{"updates":[{"values":"not-a-list","timestamp":"t"}]}',
        '{"updates":[{"timestamp":"t","values":[{"path":"x"}]}]}',
        '{"updates":[{"timestamp":"t","values":[{"value":1}]}]}',
        'not-json-at-all',
        '',
    ]:
        signalk_instance.receive_signalk(msg)  # must not raise


def test_put_rejected_when_disabled(signalk_instance):
    import signalk as sk
    # put_enabled defaults to False via the fixture.
    signalk_instance._handle_signalk_put(
        [{'path': 'steering.autopilot.target.headingMagnetic', 'value': 1.0}],
        request_id='rq1',
    )
    assert signalk_instance.client.writes == []
    _ = sk  # reference module; silences F401 in IDEs


def test_put_applies_when_enabled(signalk_instance):
    signalk_instance.put_enabled.value = True
    signalk_instance._handle_signalk_put(
        [{'path': 'steering.autopilot.target.headingMagnetic', 'value': 1.5}],
        request_id='rq2',
    )
    # Mapping divides by `radians` (pi/180), so 1.5 rad → ~85.943 deg.
    assert signalk_instance.client.writes
    name, value = signalk_instance.client.writes[0]
    assert name == 'ap.heading_command'
    assert abs(value - 85.9437) < 1e-3


def test_put_unknown_path_ignored(signalk_instance):
    signalk_instance.put_enabled.value = True
    signalk_instance._handle_signalk_put(
        [{'path': 'environment.outside.temperature', 'value': 293.0}],
        request_id='rq3',
    )
    assert signalk_instance.client.writes == []


def test_backoff_grows_and_resets():
    import signalk as sk
    obj = sk.signalk.__new__(sk.signalk)
    delays = [obj._backoff_delay() for _ in range(6)]
    # Strictly non-decreasing up to the cap.
    assert delays[0] == sk.SIGNALK_BACKOFF_MIN
    assert delays[-1] == sk.SIGNALK_BACKOFF_MAX
    for a, b in zip(delays, delays[1:]):
        assert b >= a


def _utc_epoch(year, month, day, hour, minute, second=0, micro=0):
    import datetime
    return datetime.datetime(
        year, month, day, hour, minute, second, micro,
        tzinfo=datetime.timezone.utc,
    ).timestamp()


def test_timestamp_zulu_form_parses():
    import signalk as sk
    got = sk._parse_signalk_timestamp('2026-04-19T14:28:00.123Z')
    assert abs(got - _utc_epoch(2026, 4, 19, 14, 28, 0, 123000)) < 1e-6


def test_timestamp_numeric_offset_form_parses():
    """Regression for #282: pre-fix the converter used
    strptime('%Y-%m-%dT%H:%M:%S.%fZ'), which rejected numeric offsets
    and spammed the log on every NMEA-2000-bridged delta."""
    import signalk as sk
    # 10:28 at -04:00 == 14:28 UTC
    got = sk._parse_signalk_timestamp('2026-04-19T10:28:00.000-04:00')
    assert abs(got - _utc_epoch(2026, 4, 19, 14, 28)) < 1e-6


def test_timestamp_non_string_rejected():
    import signalk as sk
    with pytest.raises(ValueError):
        sk._parse_signalk_timestamp(1234567890)


class _Value:
    def __init__(self, v): self.value = v
    def set(self, v): self.value = v


def _bad_status_websocket(status_code):
    """A stand-in for the websocket module whose handshake fails."""
    class WebSocketBadStatusException(Exception):
        def __init__(self, msg, status):
            super().__init__(msg)
            self.status_code = status

    class _Module:
        pass
    mod = _Module()
    mod.WebSocketBadStatusException = WebSocketBadStatusException

    def create_connection(url, **kwargs):
        raise WebSocketBadStatusException('Handshake status %d' % status_code, status_code)
    mod.create_connection = create_connection
    return mod


def _connect_with_status(monkeypatch, tmp_path, status_code):
    import signalk as sk
    token_file = tmp_path / 'signalk-token'
    token_file.write_text('stored-token')
    monkeypatch.setattr(sk, 'token_path', str(token_file))
    monkeypatch.setattr(sk, 'websocket', _bad_status_websocket(status_code))
    obj = sk.signalk.__new__(sk.signalk)
    obj.token = 'stored-token'
    obj.signalk_ws_url = 'ws://192.168.1.104:3000/signalk/v1/stream?subscribe=none'
    obj.tls_verify = _Value(True)
    obj.connect_signalk()
    return obj, token_file


def test_restarting_server_does_not_discard_token(monkeypatch, tmp_path):
    """A SignalK server that is (re)starting answers 503: the token is still
    good and must not be thrown away (that forced a new access request, to be
    approved again by hand, after every SignalK restart)."""
    obj, token_file = _connect_with_status(monkeypatch, tmp_path, 503)
    assert obj.token == 'stored-token'
    assert token_file.exists()


def test_unauthorized_discards_token(monkeypatch, tmp_path):
    obj, token_file = _connect_with_status(monkeypatch, tmp_path, 401)
    assert obj.token is False
    assert not token_file.exists()


def test_access_request_asks_for_readwrite(monkeypatch):
    """pypilot publishes heading and attitude, so it needs write access;
    without "permissions" the SignalK admin offers the request as read only."""
    import signalk as sk
    sent = {}

    class _Resp:
        status_code = 202
        content = b'{"state": "PENDING", "href": "/signalk/v1/requests/1"}'

    class _Requests:
        def post(self, url, data=None, **kwargs):
            sent['url'] = url
            sent['data'] = data
            return _Resp()
    monkeypatch.setattr(sk, 'requests', _Requests())
    obj = sk.signalk.__new__(sk.signalk)
    obj.signalk_access_url = False
    obj.uid = _Value('pypilot-12345678901')
    obj.signalk_http_base = 'http://192.168.1.104:3000'
    obj.signalk_host_port = '192.168.1.104:3000'
    obj.tls_verify = _Value(True)
    obj.request_access()
    assert sent['data']['permissions'] == 'readwrite'
    assert sent['data']['clientId'] == 'pypilot-12345678901'


def test_anonymous_input_then_approved_connection(monkeypatch, tmp_path, capsys):
    import json
    from types import SimpleNamespace as Stub
    from unittest.mock import Mock
    import signalk as sk

    token_file = tmp_path / 'signalk-token'
    monkeypatch.setattr(sk, 'token_path', str(token_file))
    requests = pytest.importorskip('requests')
    http = Mock(spec=requests)
    http.exceptions = requests.exceptions
    http.get.side_effect = [
        requests.exceptions.ConnectTimeout(),
        Stub(content=b'{"state":"PENDING"}'),
        Stub(content=b'{"state":"COMPLETED","accessRequest":{"permission":"APPROVED","token":"approved"}}'),
    ]
    monkeypatch.setattr(sk, 'requests', http)
    now = 100
    monkeypatch.setattr(sk.time, 'monotonic', lambda: now)
    sockets = []
    def connect(url, **kwargs):
        ws = Mock()
        ws.recv.side_effect = BlockingIOError
        sockets.append((kwargs, ws))
        return ws
    monkeypatch.setattr(sk, 'websocket', Stub(create_connection=connect))
    obj = sk.signalk.__new__(sk.signalk)
    obj.process = obj.zero_conf = obj.last_manual_signalk_host = obj.token = obj.ws = False
    obj.initialized = True
    obj.enabled = obj.tls_verify = obj.put_enabled = _Value(True)
    obj.signalk_host = _Value('')
    obj.signalk_host_port = 'test:3000'
    obj.signalk_ws_url = 'ws://test:3000/stream'
    obj.signalk_access_url = 'http://test:3000/requests/request'
    obj.last_access_request_time = float('-inf')
    obj.period = _Value(.5)
    obj.subscribe_policy = _Value('instant')
    obj.last_values = {}
    obj.last_sources = {}
    obj.signalk_last_msg_time = {}
    obj.client = Mock()
    pending_url = obj.signalk_access_url
    obj.poll()
    lines = capsys.readouterr().out.splitlines()
    assert len(lines) == 1 and 'timed out' in lines[0]
    assert obj.signalk_access_url == pending_url
    now += 9
    obj.poll()
    assert http.get.call_count == 1
    assert len(sockets) == 1 and 'header' not in sockets[0][0]
    assert obj.token is False
    obj.client.receive_single.side_effect = [('gps.source', 'none'), ('timestamp', 1), None]
    obj.send_signalk = Mock()
    obj.poll()
    assert 'navigation.position' in sockets[0][1].send.call_args.args[0]
    obj.send_signalk.assert_not_called()
    obj.receive_signalk(json.dumps({'updates': [{'timestamp': 't', 'put': [
        {'path': 'steering.autopilot.engaged', 'value': True}]}]}))
    obj.client.set.assert_not_called()
    now += 1
    obj.poll()  # Retry the same pending GET after ten seconds.
    assert http.get.call_count == 2 and obj.signalk_access_url == pending_url
    now += 10
    obj.poll()  # Approval arrives.
    assert [call.args[0] for call in http.get.call_args_list] == [pending_url] * 3
    http.post.assert_not_called()
    sockets[0][1].close.assert_called_once()
    obj.poll()  # Connect using the approved token.
    assert len(sockets) == 2 and sockets[1][0]['header'] == {'Authorization': 'JWT approved'}
    assert token_file.read_text() == 'approved'
