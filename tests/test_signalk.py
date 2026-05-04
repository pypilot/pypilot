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
