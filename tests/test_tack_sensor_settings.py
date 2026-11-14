from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import tacking


class Client:
    def register(self, value):
        return value


def make_tack(monkeypatch, wind_source='serial'):
    monkeypatch.setattr(tacking.time, 'monotonic', lambda: 100)
    ap = SimpleNamespace(
        client=Client(),
        sensors=SimpleNamespace(wind=SimpleNamespace(
            source=SimpleNamespace(value=wind_source),
            direction=SimpleNamespace(value=45))),
        boatimu=SimpleNamespace(heel=10),
    )
    tack = tacking.Tack(ap)
    tack.time = 0
    tack.wind_log.update = Mock(return_value='starboard')
    tack.heel_log.update = Mock(return_value='port')
    return tack


@pytest.mark.parametrize('source', ['serial', 'none'])
def test_disabled_sensors_do_not_change_tack_direction(monkeypatch, source):
    tack = make_tack(monkeypatch, source)
    tack.use_wind_direction.set(False)
    tack.use_heel.set(False)
    tack.poll()
    assert tack.direction.value == 'none'
    tack.wind_log.update.assert_not_called()
    tack.heel_log.update.assert_not_called()


def test_disabled_wind_allows_enabled_heel(monkeypatch):
    tack = make_tack(monkeypatch)
    tack.use_wind_direction.set(False)
    tack.use_heel.set(True)
    tack.poll()
    assert tack.direction.value == 'port'
    tack.wind_log.update.assert_not_called()
    tack.heel_log.update.assert_called_once_with(10)


def test_enabled_wind_takes_priority(monkeypatch):
    tack = make_tack(monkeypatch)
    tack.use_heel.set(True)
    tack.poll()
    assert tack.direction.value == 'starboard'
    tack.wind_log.update.assert_called_once_with(45)
    tack.heel_log.update.assert_not_called()


def test_enabled_heel_waits_thirty_seconds(monkeypatch):
    tack = make_tack(monkeypatch, 'none')
    tack.use_heel.set(True)
    tack.time = 75
    tack.poll()
    assert tack.direction.value == 'none'
    tack.heel_log.update.assert_not_called()
