import calendar
import pytest
import nmea

@pytest.mark.parametrize('wall_time', [0, 1900000000])
def test_rmc_date_does_not_depend_on_computer_clock(monkeypatch, wall_time):
    monkeypatch.setattr(nmea.time, 'time', lambda: wall_time)
    monkeypatch.setattr(nmea, 'gps_timeoffset', 0)
    result = nmea.parse_nmea_gps('$GPRMC,123456.000,A,5000.000,N,00100.000,W,2,90,151126,,,A*00')
    assert result
    assert result[1]['timestamp'] == calendar.timegm((2026, 11, 15, 12, 34, 56))
