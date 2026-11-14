"""Exercise GPSD TPV parsing without GPS hardware or a subprocess."""
import os
import time
from datetime import datetime, timezone

import pytest

from gpsd import gpsProcess


class Pipe:
    def __init__(self):
        self.fixes = []

    def send(self, fix, blocking):
        self.fixes.append(fix)


@pytest.mark.parametrize('zone', ['UTC0', 'EST5', 'JST-9', 'GMT0BST,M3.5.0/1,M10.5.0'])
@pytest.mark.parametrize('stamp', [
    '2026-07-15T14:32:07.125Z',
    '2026-01-15T14:32:07.000Z',
    '2026-07-15T10:32:07.125-04:00',
])
def test_tpv_timestamp_is_independent_of_local_timezone(zone, stamp):
    previous = os.environ.get('TZ')
    try:
        os.environ['TZ'] = zone
        time.tzset()
        process = gpsProcess.__new__(gpsProcess)
        process.devices = ['/dev/test-gps']
        process.baud_boot_device_hint = '/dev/test-gps'
        pipe = Pipe()
        process.parse_gpsd({
            'class': 'TPV', 'mode': 3, 'device': '/dev/test-gps',
            'time': stamp, 'lat': 50.0, 'lon': -1.0, 'speed': 1.0,
        }, pipe)
        expected = datetime(2026, 7 if '07-15' in stamp else 1, 15,
                            14, 32, 7, 125000 if '.125' in stamp else 0,
                            tzinfo=timezone.utc).timestamp()
        assert len(pipe.fixes) == 1
        assert pipe.fixes[0]['timestamp'] == pytest.approx(expected, abs=0.000001, rel=0)
        assert pipe.fixes[0]['speed'] == pytest.approx(1.944)
    finally:
        if previous is None:
            os.environ.pop('TZ', None)
        else:
            os.environ['TZ'] = previous
        time.tzset()
