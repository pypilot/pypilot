"""Exercise the production IMU output and bridge with synthetic measurements."""
from collections import deque
from types import SimpleNamespace

import pytest

from pypilot.nmea import Nmea, nmeaBridge, check_nmea_cksum


@pytest.fixture
def output(monkeypatch):
    pending = deque()
    lines = []
    bridge = nmeaBridge.__new__(nmeaBridge)
    bridge.process = True
    bridge.pipe = SimpleNamespace(recv=lambda: pending.popleft() if pending else False)
    bridge.sockets = [SimpleNamespace(write=lines.append)]
    sender = Nmea.__new__(Nmea)
    sender.nmea_bridge = bridge
    sender.pipe = SimpleNamespace(send=pending.append)
    sender.probe_serial = lambda: None  # Never probe real serial devices.
    sender.poller = SimpleNamespace(poll=lambda timeout: [])
    sender.devices = []
    sender.sockets = True
    sender.last_imu_time = 0
    sender.start_time = 0
    sender.sensors = SimpleNamespace(sensors={
        name: SimpleNamespace(source=SimpleNamespace(value='none'))
        for name in ('wind', 'truewind', 'rudder')
    })
    values = {
        name: SimpleNamespace(value=value)
        for name, value in {
            'imu.pitch': 2.0, 'imu.roll': -3.0,
            'imu.heading_lowpass': 120.0,
            'imu.headingrate_lowpass': 0.0,
            'imu.headingrate': 99.0,
        }.items()
    }
    sender.client = SimpleNamespace(values=SimpleNamespace(values=values))
    monkeypatch.setattr('pypilot.nmea.time.monotonic', lambda: 10.0)
    return sender, bridge, values, lines


@pytest.mark.parametrize('degrees_per_second,expected', [
    (1.0, 60.0), (-1.0, -60.0), (0.0, 0.0), (0.125, 7.5),
])
def test_turn_rate_sent_in_degrees_per_minute(output, degrees_per_second, expected):
    sender, bridge, values, lines = output
    values['imu.headingrate_lowpass'].value = degrees_per_second
    sender.poll()
    bridge.receive_pipe()
    assert len(lines) == 4
    assert all(check_nmea_cksum(line.strip()) for line in lines)
    rot = next(line for line in lines if line.startswith('$APROT,'))
    assert float(rot.split(',')[1]) == pytest.approx(expected)
    assert rot.split(',')[2].startswith('A*')
    # Conversion is confined to the wire output, not the control-loop values.
    assert values['imu.headingrate_lowpass'].value == degrees_per_second
    assert any(line.startswith('$APHDM,120.000,M*') for line in lines)
    # Polling again before the output interval must not duplicate measurements.
    sender.poll()
    bridge.receive_pipe()
    assert len(lines) == 4
