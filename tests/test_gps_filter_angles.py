import math
from types import SimpleNamespace
import numpy as np
import pytest
from pypilot import gps_filter


@pytest.mark.parametrize('declination,offset', [(0, 0), (90, 0), (0, 90), (-30, 45)])
def test_predict_converts_degree_corrections_to_radians(monkeypatch, declination, offset):
    monkeypatch.setattr(gps_filter, 'np', np, raising=False)
    monkeypatch.setattr(gps_filter, 'pygeomag', None)
    f = gps_filter.GPSFilter(SimpleNamespace(register=lambda value: value))
    f.enabled.set(True)
    f.gps_time_offset.set(0)
    f.update({'lat': 50, 'lon': -1, 'timestamp': 1.}, 1.)
    f.declination.set(declination)
    f.compass_offset.set(offset)
    # A level, north-facing RTIMU reads [-1, 0, 1] for 1 g of northward acceleration.
    for t in (1., 1.125):
        f.predict([-1, 0, 1], [1, 0, 0, 0], t)
    angle = math.radians(declination + offset)
    east = 9.81 * math.sin(angle)
    north = 9.81 * math.cos(angle)
    assert f.X[2:] == pytest.approx([east * .125, north * .125])
