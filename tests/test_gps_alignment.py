from types import SimpleNamespace

import pytest
from pypilot import sensors


class StubValue:
    def __init__(self, value=False):
        self.value = value

    def set(self, value):
        self.value = value


class StubFiltered:
    def update(self, _data, _timestamp):
        pass


def make_gps(counter, heading, heading_offset, declination=False):
    gps = sensors.gps.__new__(sensors.gps)
    gps.speed = StubValue()
    gps.track = StubValue()
    gps.fix = StubValue()
    gps.declination = StubValue(declination)
    gps.alignmentCounter = StubValue(counter)
    gps.last_alignmentCounter = False
    gps.gps_alignment_track = False
    gps.filtered = StubFiltered()
    gps.client = SimpleNamespace(values=SimpleNamespace(values={
        'imu.heading': StubValue(heading),
        'imu.heading_offset': StubValue(heading_offset),
    }))
    return gps


def update(gps, track):
    assert gps.update({'speed': 5, 'track': track})


def test_gps_alignment_averages_tracks_and_updates_existing_offset(monkeypatch):
    timestamps = iter([0, 0, 1, 1, 2, 2])
    monkeypatch.setattr(sensors.time, 'monotonic', lambda: next(timestamps))
    gps = make_gps(counter=3, heading=340, heading_offset=20, declination=10)

    update(gps, 359)
    update(gps, 0)
    update(gps, 1)

    assert gps.alignmentCounter.value == 0
    assert gps.client.values.values['imu.heading_offset'].value == pytest.approx(30)
    assert gps.gps_alignment_track is False


def test_gps_alignment_stops_after_long_gap(monkeypatch):
    timestamps = iter([0, 0, 21, 21])
    monkeypatch.setattr(sensors.time, 'monotonic', lambda: next(timestamps))
    gps = make_gps(counter=2, heading=90, heading_offset=5)

    update(gps, 90)
    update(gps, 90)

    assert gps.alignmentCounter.value == 0
    assert gps.client.values.values['imu.heading_offset'].value == 5
    assert gps.gps_alignment_track is False


def test_gps_alignment_waits_for_a_track_sample(monkeypatch):
    timestamps = iter([0])
    monkeypatch.setattr(sensors.time, 'monotonic', lambda: next(timestamps))
    gps = make_gps(counter=2, heading=90, heading_offset=5)

    assert gps.update({'speed': 5})

    assert gps.alignmentCounter.value == 2
    assert gps.gps_alignment_track is False
