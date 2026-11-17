import json
from types import SimpleNamespace

import pytest

import signalk


@pytest.mark.parametrize('filtered', [False, True])
@pytest.mark.parametrize('missing', ['lat', 'lon'])
def test_partial_gps_fix_does_not_abort_publishing(filtered, missing):
    obj = signalk.signalk.__new__(signalk.signalk)
    messages = []
    obj.ws = SimpleNamespace(send=messages.append)
    obj.last_sources = {'gps': 'gpsd'}
    obj.gps_filtered_output = filtered
    obj.last_values_keys = {'navigation.attitude': {}, 'navigation.position': {}}
    fix = {'lat': 50.0, 'lon': -1.0, 'speed': 3.0, 'track': 90}
    del fix[missing]
    key = 'gps.filtered.fix' if filtered else 'gps.fix'
    obj.last_values = {key: fix}
    obj.send_signalk()
    values = {u['path']: u['value'] for message in messages
              for u in json.loads(message)['updates'][0]['values']}
    assert 'navigation.position' not in values
    assert values['navigation.speedOverGround'] > 0
    # Complementary incomplete fixes must not create a mixed-sample position.
    other_missing = 'lon' if missing == 'lat' else 'lat'
    next_fix = {'lat': 60.0, 'lon': -10.0, 'speed': 3.0, 'track': 90}
    del next_fix[other_missing]
    obj.last_values = {key: next_fix}
    messages.clear()
    obj.send_signalk()
    values = {u['path']: u['value'] for message in messages
              for u in json.loads(message)['updates'][0]['values']}
    assert 'navigation.position' not in values
    # A later complete fix must still publish both coordinates.
    obj.last_values = {key: {'lat': 51.0, 'lon': -2.0, 'speed': 4.0, 'track': 100}}
    messages.clear()
    obj.send_signalk()
    values = {u['path']: u['value'] for message in messages
              for u in json.loads(message)['updates'][0]['values']}
    assert values['navigation.position'] == {'latitude': 51.0, 'longitude': -2.0}
