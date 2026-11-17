import pytest
from pypilot import sensors


class Client:
    def register(self, value):
        return value


@pytest.mark.parametrize('sensor_type', [sensors.Wind, sensors.TrueWind])
@pytest.mark.parametrize('new_direction', [0, 120, -120])
def test_wind_reacquisition_starts_a_fresh_filter(sensor_type, new_direction):
    wind = sensor_type(Client(), None)
    wind.update({'direction': -60, 'speed': 30})
    wind.reset()
    assert wind.filtered_direction.value is False
    assert wind.filtered_speed.value == 0
    assert wind.filter_factor.value is False
    wind.update({'direction': new_direction, 'speed': 5})
    fresh = sensor_type(Client(), None)
    fresh.update({'direction': new_direction, 'speed': 5})
    assert wind.filtered_direction.value == pytest.approx(fresh.filtered_direction.value)
    assert wind.filtered_speed.value == pytest.approx(fresh.filtered_speed.value)
