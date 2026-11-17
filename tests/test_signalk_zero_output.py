import json
from types import SimpleNamespace

import pytest

import signalk


@pytest.mark.parametrize('name,path', [
    ('imu.headingrate_lowpass', 'navigation.rateOfTurn'),
    ('wind.speed', 'environment.wind.speedApparent'),
    ('wind.direction', 'environment.wind.angleApparent'),
    ('rudder.angle', 'steering.rudderAngle'),
    ('water.speed', 'navigation.speedThroughWater'),
])
@pytest.mark.parametrize('value', [0, 0.0, False, None, 12])
def test_zero_is_published_but_unavailable_is_not(name, path, value):
    obj = signalk.signalk.__new__(signalk.signalk)
    messages = []
    obj.ws = SimpleNamespace(send=messages.append)
    obj.last_sources = {name.split('.')[0]: 'serial'}
    obj.gps_filtered_output = False
    obj.last_values = {name: value}
    obj.last_values_keys = {'navigation.attitude': {}}
    obj.send_signalk()
    updates = [u for message in messages for u in json.loads(message)['updates'][0]['values']]
    found = [u for u in updates if u['path'] == path]
    if value is False or value is None:
        assert not found
    else:
        assert len(found) == 1
        assert found[0]['value'] == pytest.approx(value * dict(
            (key[0], key[1]) for table in signalk.signalk_table.values() for key in table
        )[path])
