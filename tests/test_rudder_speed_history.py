"""Rudder speed must use consecutive valid readings from the same sensor."""

import pytest

import rudder


class Client:
    def register(self, value):
        return value


@pytest.fixture
def sensor(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(rudder.time, "monotonic", lambda: clock[0])
    instance = rudder.Rudder(Client())
    assert instance.update({"device": "one", "angle": 0.2}) is False
    return instance, clock


@pytest.mark.parametrize("angle", [0, 20, -20])
def test_first_reading_establishes_speed_baseline(sensor, angle):
    instance, clock = sensor
    clock[0] = 100.1
    assert instance.update({"device": "one", "angle": angle / 100})
    assert instance.speed.value == 0
    clock[0] = 100.3
    assert instance.update({"device": "one", "angle": (angle + 10) / 100})
    assert instance.speed.value == pytest.approx(5)


@pytest.mark.parametrize("invalidation", ["reset", "empty", "nan", "device"])
def test_invalid_reading_discards_speed_history(sensor, invalidation):
    instance, clock = sensor
    clock[0] = 100.1
    instance.update({"device": "one", "angle": 0.2})
    clock[0] = 100.3
    instance.update({"device": "one", "angle": 0.3})
    device = "one"
    if invalidation == "reset":
        instance.reset()
    elif invalidation == "empty":
        instance.update({})
    elif invalidation == "nan":
        instance.update({"device": device, "angle": float("nan")})
    else:
        device = "two"
        instance.update({"device": device, "angle": 0.1})
    assert instance.angle.value is False
    assert instance.speed.value is False
    clock[0] = 200
    assert instance.update({"device": device, "angle": 0.1})
    assert instance.speed.value == 0
