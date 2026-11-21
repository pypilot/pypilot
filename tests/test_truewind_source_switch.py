"""Exercise the production controller method without importing hardware startup.

The class is compiled unchanged from its AST because importing autopilot.py
rewraps stdout and imports IMU/motor modules. No controller is initialized.
"""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from pypilot import sensors

from resolv import resolv
from values import HeadingOffset, SensorValue, StringValue


def autopilot_class():
    path = Path(__file__).resolve().parents[1] / 'pypilot/autopilot.py'
    tree = ast.parse(path.read_text(), filename=str(path))
    cls = next(node for node in tree.body
               if isinstance(node, ast.ClassDef) and node.name == 'Autopilot')
    namespace = {'math': math, 'resolv': resolv}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace['Autopilot']


class Client:
    def register(self, value):
        return value


@pytest.mark.parametrize('initial,water,expected', [
    ('none', True, 'water+wind'), ('none', False, 'gps+wind'),
    ('gps+wind', True, 'water+wind'), ('water+wind', False, 'gps+wind'),
    ('water+wind', True, 'water+wind'), ('gps+wind', False, 'gps+wind'),
    ('serial', True, 'serial'), ('signalk', False, 'signalk'),
])
def test_synthetic_source_tracks_current_speed_provider(initial, water, expected):
    cls = autopilot_class()
    ap = cls.__new__(cls)
    ap.server = SimpleNamespace(__del__=lambda: None)
    ap.watchdog_device = False
    ap.boatimu = SimpleNamespace(SensorValues={'heading_lowpass': SensorValue('heading', 90)})
    client = Client()
    wind = sensors.Wind(client, None)
    wind.update({'direction': 45, 'speed': 10})
    wind.source.set('serial')
    truewind = sensors.TrueWind(client, None)
    truewind.update({'direction': 120, 'speed': 20})
    truewind.source.set(initial)
    ap.sensors = SimpleNamespace(
        wind=wind, truewind=truewind,
        water=SimpleNamespace(source=StringValue('water.source', 'serial' if water else 'none'),
                              speed=SensorValue('water.speed', 5)),
        gps=SimpleNamespace(source=StringValue('gps.source', 'gpsd'),
                            speed=SensorValue('gps.speed', 4), track=SensorValue('gps.track', 90)),
    )
    ap.gps_speed = 4
    ap.gps_compass_offset = HeadingOffset('gps.offset')
    ap.wind_compass_offset = HeadingOffset('wind.offset')
    ap.true_wind_compass_offset = HeadingOffset('truewind.offset')
    ap.wind_offset_filter = SensorValue('filter', 0.1)
    with patch.object(truewind.source, 'update', wraps=truewind.source.update) as update:
        ap.compute_offsets()
        if initial == expected:
            update.assert_not_called()
        else:
            update.assert_called_once_with(expected)
    assert truewind.source.value == expected
    if initial in ('serial', 'signalk'):
        assert truewind.direction.value == 120
        assert truewind.speed.value == 20
    else:
        assert truewind.speed.value == pytest.approx(
            sensors.TrueWind.compute_true_wind_speed(5 if water else 4, 10, 45))
