from pypilot import sensors
import pytest

class Client:
    def register(self, value): return value

@pytest.mark.parametrize('direction', [0, 120, -120])
def test_first_wind_direction_has_no_invented_zero_sample(direction):
    wind = sensors.Wind(Client(), None)
    wind.update({'direction': direction, 'speed': 10})
    assert wind.filtered_direction.value == pytest.approx(direction)
    wind.update({'direction': direction + 2, 'speed': 10})
    assert direction < wind.filtered_direction.value < direction + 2
