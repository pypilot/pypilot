from pypilot import gps_filter
import pytest

@pytest.mark.parametrize('origin,destination', [(179.9, -179.9), (-179.9, 179.9), (-1, 1)])
def test_local_gps_roundtrip_across_date_line(origin, destination):
    x, y = gps_filter.ll_to_xy(50, destination, 50, origin)
    lat, lon = gps_filter.xy_to_ll(x, y, 50, origin)
    assert lat == pytest.approx(50)
    assert lon == pytest.approx(destination)
    assert -180 <= lon <= 180
