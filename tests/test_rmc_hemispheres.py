"""RMC coordinates need explicit, valid latitude and longitude hemispheres."""

import pytest

import nmea


def sentence(latitude, longitude):
    body = f"GPRMC,120000.000,A,5000.0000,{latitude},00100.0000,{longitude},5.0,90.0,161126,,,A"
    return f"${body}*{nmea.nmea_cksum(body):02X}"


@pytest.mark.parametrize("latitude, longitude", [("", "E"), ("X", "E"), ("E", "E"), ("N", ""), ("N", "X"), ("N", "N")])
def test_missing_or_unknown_hemisphere_is_rejected(latitude, longitude):
    message = sentence(latitude, longitude)
    assert nmea.check_nmea_cksum(message)
    assert nmea.parse_nmea_gps(message) is False


@pytest.mark.parametrize("latitude, lat", [("N", 50), ("S", -50)])
@pytest.mark.parametrize("longitude, lon", [("E", 1), ("W", -1)])
def test_all_valid_quadrants_remain_supported(latitude, lat, longitude, lon):
    kind, reading = nmea.parse_nmea_gps(sentence(latitude, longitude))
    assert kind == "gps"
    assert reading["lat"] == lat
    assert reading["lon"] == lon
