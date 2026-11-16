from pypilot import sensors

class Client:
    def register(self, value): return value


def test_combined_water_and_leeway_update():
    water = sensors.Water(Client())
    assert water.update({'speed': 4, 'leeway': 5})
    assert water.speed.value == 4
    assert water.leeway.value == 5
    assert water.leeway.source.value == 'sensor'


def test_leeway_only_message_retains_speed():
    water = sensors.Water(Client())
    water.speed.set(4)
    assert water.update({'leeway': -3})
    assert water.speed.value == 4
    assert water.leeway.value == -3
    assert water.leeway.source.value == 'sensor'
    assert water.update({}) is False
