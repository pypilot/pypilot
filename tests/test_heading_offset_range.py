"""Regression for #154: allow a full turn on either side of zero."""
from types import SimpleNamespace

import pytest

import boatimu


@pytest.fixture
def imu(monkeypatch):
    # Exercise the real property registration and alignment calculation,
    # without starting sensor or calibration processes.
    monkeypatch.setattr(boatimu, 'IMU', lambda server: None)
    monkeypatch.setattr(boatimu, 'AutomaticCalibrationProcess', lambda server: None)
    client = SimpleNamespace(server=None, register=lambda value: value)
    return boatimu.BoatIMU(client)


@pytest.mark.parametrize('offset', [-360, -220, -180, -90, 0, 90, 180, 220, 360])
def test_full_turn_offsets_are_accepted(imu, offset):
    imu.heading_off.set(offset)
    assert imu.heading_off.value == offset


@pytest.mark.parametrize('offset', [-361, 361])
def test_offsets_beyond_full_turn_leave_setting_unchanged(imu, offset):
    imu.heading_off.set(25)
    imu.heading_off.set(offset)
    assert imu.heading_off.value == 25


@pytest.mark.parametrize('offset,equivalent', [(220, -140), (-220, 140), (360, 0), (-360, 0)])
def test_equivalent_offsets_produce_same_alignment(imu, offset, equivalent):
    # A tilted starting alignment also needs to remain equivalent.
    q = boatimu.quaternion.angvec2quat(.3, [1, 0, 0])
    imu.heading_off.set(offset)
    imu.update_alignment(q)
    actual = boatimu.quaternion.rotvecquat([1, 0, 0], imu.alignmentQ.value)
    imu.heading_off.set(equivalent)
    imu.update_alignment(q)
    expected = boatimu.quaternion.rotvecquat([1, 0, 0], imu.alignmentQ.value)
    assert actual == pytest.approx(expected)


def test_client_metadata_advertises_full_range(imu):
    assert imu.heading_off.info['min'] == -360
    assert imu.heading_off.info['max'] == 360
    assert imu.heading_off.info['persistent'] is True
