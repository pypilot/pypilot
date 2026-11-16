"""GPS sample times must retain the fractional seconds sent by the receiver."""
from datetime import datetime

import pytest

import nmea


@pytest.mark.parametrize('clock,received,expected', [
    ('2026-11-14T12:00:00+00:00', '120000.000', '2026-11-14T12:00:00+00:00'),
    ('2026-11-14T12:00:00.2+00:00', '120000.125', '2026-11-14T12:00:00.125+00:00'),
    ('2026-11-14T12:00:01+00:00', '120000.999', '2026-11-14T12:00:00.999+00:00'),
    ('2026-11-14T12:00:00.2+00:00', '120000.123456', '2026-11-14T12:00:00.123456+00:00'),
    ('2026-11-15T00:00:00+00:00', '235959.875', '2026-11-14T23:59:59.875+00:00'),
    ('2026-11-14T23:59:59.9+00:00', '000000.125', '2026-11-15T00:00:00.125+00:00'),
])
def test_rmc_sample_time_keeps_fractional_seconds(monkeypatch, clock, received, expected):
    monkeypatch.setattr(nmea.time, 'time', lambda: datetime.fromisoformat(clock).timestamp())
    monkeypatch.setattr(nmea, 'gps_timeoffset', 0)
    body = 'GPRMC,' + received + ',A,5000.000,N,00100.000,W,5.0,90.0,141126,,,A'
    sentence = '$' + body + '*%02X' % nmea.nmea_cksum(body)
    name, fix = nmea.parse_nmea_gps(sentence)
    assert name == 'gps'
    assert fix['timestamp'] == pytest.approx(datetime.fromisoformat(expected).timestamp(), rel=0, abs=1e-6)
    assert fix['lat'] == 50
    assert fix['lon'] == -1
