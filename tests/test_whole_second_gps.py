"""Valid whole-second receiver times must not be replaced by the host clock."""
from datetime import datetime, timezone

import pytest

import nmea
from gpsd import gpsProcess


@pytest.mark.parametrize('clock', ['000000', '143207', '235959'])
@pytest.mark.parametrize('fraction', ['', '.000', '.125'])
def test_rmc_whole_and_fractional_seconds(monkeypatch, clock, fraction):
    monkeypatch.setattr(nmea.time, 'time', lambda: 1_000_000_000)
    monkeypatch.setattr(nmea, 'gps_timeoffset', 0)
    message = f'GPRMC,{clock}{fraction},A,5000.0,N,00100.0,W,1.0,90,150726,,,'
    sentence = f'${message}*{nmea.nmea_cksum(message):02X}'
    name, fix = nmea.parse_nmea_gps(sentence)
    expected = datetime(2026, 7, 15, int(clock[:2]), int(clock[2:4]),
                        int(clock[4:]), 125000 if fraction == '.125' else 0,
                        tzinfo=timezone.utc).timestamp()
    assert name == 'gps'
    assert fix['timestamp'] == pytest.approx(expected, abs=1e-6, rel=0)


@pytest.mark.parametrize('stamp', ['2026-07-15T14:32:07Z',
                                   '2026-07-15T10:32:07-04:00',
                                   '2026-07-15T14:32:07.000Z'])
def test_gpsd_whole_seconds(stamp):
    process = gpsProcess.__new__(gpsProcess)
    process.devices = ['/dev/test-gps']
    process.baud_boot_device_hint = '/dev/test-gps'

    class Pipe:
        def send(self, fix, blocking):
            self.fix = fix

    pipe = Pipe()
    process.parse_gpsd({'class': 'TPV', 'mode': 3, 'device': '/dev/test-gps',
                       'time': stamp, 'lat': 50, 'lon': -1, 'speed': 1}, pipe)
    expected = datetime(2026, 7, 15, 14, 32, 7, tzinfo=timezone.utc).timestamp()
    assert pipe.fix['timestamp'] == expected
