import datetime
from unittest.mock import Mock, call

import numpy as np
import pytest

from pypilot import gps_filter


class Client:
    def register(self, value): return value


@pytest.fixture
def gps(monkeypatch):
    monkeypatch.setattr(gps_filter, 'np', np, raising=False)
    f = gps_filter.GPSFilter(Client())
    f.enabled.set(True)
    return f


def test_declination_uses_fix_date_and_refreshes_every_four_hours(gps, monkeypatch):
    geomag = Mock()
    geomag.decimal_year_from_date.return_value = 2028.5
    calculate = geomag.GeoMag.return_value.calculate
    calculate.return_value.d = -7.5
    monkeypatch.setattr(gps_filter, 'pygeomag', geomag)
    start = datetime.datetime(2028, 7, 2, tzinfo=datetime.timezone.utc)

    for elapsed, lon in [(0, -1), (14399, -2), (14400, -3)]:
        gps.update({'lat': 50, 'lon': lon, 'timestamp': start.timestamp() + elapsed}, 1 + elapsed)

    assert geomag.decimal_year_from_date.call_args_list == [call(start), call(start + datetime.timedelta(hours=4))]
    assert geomag.GeoMag.call_args_list == [call(base_year=2028)] * 2
    assert calculate.call_args_list == [call(50, -1, 0, 2028.5), call(50, -3, 0, 2028.5)]
    assert gps.declination.value == -7.5


def test_wmm2025_seattle_reference(gps):
    pytest.importorskip('pygeomag')
    # pygeomag's Space Needle example gives 15.06563 degrees east at 2025.25.
    date = datetime.datetime(2025, 4, 2, tzinfo=datetime.timezone.utc)
    gps.update({'lat': 47.6205, 'lon': -122.3493, 'timestamp': date.timestamp()}, 1)
    assert gps.declination.value == pytest.approx(15.06563, abs=0.001)


def test_position_still_updates_without_model(gps, monkeypatch):
    monkeypatch.setattr(gps_filter, 'pygeomag', None)
    gps.update({'lat': 50, 'lon': -1, 'timestamp': 100}, 1)
    assert gps.declination.value is False
    assert gps.lastll == (50, -1)


def test_unavailable_model_year_keeps_previous_declination(gps, monkeypatch):
    geomag = Mock()
    geomag.GeoMag.side_effect = ValueError('No coefficients for this year')
    monkeypatch.setattr(gps_filter, 'pygeomag', geomag)
    gps.declination.update(7.5)
    gps.update({'lat': 50, 'lon': -1, 'timestamp': 100}, 1)
    assert gps.declination.value == 7.5
    assert gps.lastll == (50, -1)
