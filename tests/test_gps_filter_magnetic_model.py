import datetime
from types import SimpleNamespace
import numpy as np
from pypilot import gps_filter


class Client:
    def register(self, value): return value


def test_magnetic_model_receives_current_year(monkeypatch):
    monkeypatch.setattr(gps_filter, 'np', np, raising=False)
    calls = []
    def magnetic_model(lat, lon, altitude, year):
        calls.append((lat, lon, altitude, year))
        return SimpleNamespace(decl=7.5)
    monkeypatch.setattr(gps_filter, 'wmm2020', SimpleNamespace(wmm=magnetic_model))
    f = gps_filter.GPSFilter(Client())
    f.enabled.set(True)
    f.update({'lat': 50, 'lon': -1, 'speed': 0, 'track': 0,
              'timestamp': 20000}, 20000)
    assert calls == [(50, -1, 0, datetime.date.today().year)]
    assert f.declination.value == 7.5
