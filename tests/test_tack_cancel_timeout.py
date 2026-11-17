from types import SimpleNamespace
from unittest.mock import Mock

import pytest

import tacking
from values import BooleanProperty


class Client:
    def register(self, value):
        return value


@pytest.mark.parametrize('disengage', [False, True])
def test_cancelling_waiting_tack_clears_countdown(monkeypatch, disengage):
    now = [100.0]
    monkeypatch.setattr(tacking.time, 'monotonic', lambda: now[0])
    ap = SimpleNamespace(client=Client(), enabled=BooleanProperty('enabled', True),
                         servo=SimpleNamespace(command=Mock()))
    tack = tacking.Tack(ap)
    tack.direction.set('port')
    tack.delay.set(10)
    tack.state.set('begin')
    tack.process()
    assert tack.state.value == 'waiting'
    assert tack.timeout.value == 10
    if disengage:
        ap.enabled.set(False)
    else:
        tack.state.set('none')
    now[0] += 1
    tack.process()
    assert tack.state.value == 'none'
    assert tack.timeout.value == 0
    ap.servo.command.command.assert_not_called()
    # A subsequent tack gets its own countdown.
    ap.enabled.set(True)
    tack.direction.set('port')
    tack.state.set('begin')
    tack.process()
    assert tack.timeout.value == 10
