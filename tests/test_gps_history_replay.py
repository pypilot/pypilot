import numpy as np
import pytest
from pypilot import gps_filter


class Client:
    def register(self, value):
        return value


@pytest.mark.parametrize('sample_time,replay_count', [(10.4, 0), (10.25, 1), (10.15, 2)])
def test_replay_only_predictions_after_measurement(monkeypatch, sample_time, replay_count):
    monkeypatch.setattr(gps_filter, 'np', np, raising=False)
    f = gps_filter.GPSFilter(Client())
    f.enabled.set(True)
    f.gps_time_offset.set(0)
    f.lastll = (50, -1)
    f.X = np.array([0., 0., 0., 1.])
    f.history = [{'t': t, 'dt': 0.1, 'U': np.zeros(2),
                  'X': f.X.copy(), 'P': f.P.copy()} for t in [10.1, 10.2, 10.3]]
    calls = []
    apply = f.apply_prediction

    def predict(dt, acceleration):
        calls.append(dt)
        apply(dt, acceleration)

    f.apply_prediction = predict
    f.update({'timestamp': sample_time, 'lat': 50, 'lon': -1,
              'speed': 1.944, 'track': 0}, 10.5)
    assert len(calls) == replay_count
    assert len(f.history) == replay_count
    assert f.X[1] == pytest.approx(replay_count * 0.1)
