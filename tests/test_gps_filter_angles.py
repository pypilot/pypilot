import math
import numpy as np
import pytest
from pypilot import gps_filter


class Value:
    def __init__(self, value): self.value = value
    def set(self, value): self.value = value


@pytest.mark.parametrize('declination,offset', [(90, 0), (0, 90), (-30, 45)])
def test_predict_converts_degree_corrections_to_radians(monkeypatch, declination, offset):
    monkeypatch.setattr(gps_filter, 'np', np, raising=False)
    f = gps_filter.GPSFilter.__new__(gps_filter.GPSFilter)
    f.enabled = Value(True)
    f.declination = Value(declination)
    f.compass_offset = Value(offset)
    f.use3d = False
    f.predict_t = 1
    f.X = np.zeros(4)
    f.lastll = (50, -1)
    f.history = []
    f.P = np.identity(4)
    f.speed = Value(0)
    f.track = Value(0)
    f.fix = Value(False)
    accelerations = []
    f.apply_prediction = lambda dt, u: accelerations.append(u)
    f.predict([1, 0, 1], [1, 0, 0, 0], 1.1)
    angle = math.radians(declination + offset)
    assert accelerations[0] == pytest.approx([9.81*math.cos(angle), 9.81*math.sin(angle)])
