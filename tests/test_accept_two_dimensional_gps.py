from pypilot import gpsd
import pytest

class Pipe:
    def __init__(self): self.messages = []
    def send(self, value, *args): self.messages.append(value)

@pytest.mark.parametrize('mode', [2, 3])
def test_horizontal_gps_fix_is_forwarded(mode):
    process = gpsd.gpsProcess.__new__(gpsd.gpsProcess)
    process.devices = []
    process.baud_boot_device_hint = '/dev/test'
    pipe = Pipe()
    assert process.parse_gpsd({'class': 'TPV', 'mode': mode,
        'device': '/dev/test', 'lat': 50, 'lon': -1, 'speed': 2}, pipe)
    assert len(pipe.messages) == 1
    assert pipe.messages[0]['lat'] == 50
    assert pipe.messages[0]['speed'] == pytest.approx(3.888)
    assert 'alt' not in pipe.messages[0]


def test_no_fix_is_not_forwarded():
    process = gpsd.gpsProcess.__new__(gpsd.gpsProcess)
    pipe = Pipe()
    assert not process.parse_gpsd({'class': 'TPV', 'mode': 1}, pipe)
    assert pipe.messages == []
