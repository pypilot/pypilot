"""RMC fix validity flags must be honored independently of numeric fields."""

import pytest

import nmea


def sentence(status="A", mode="A"):
    body = f"GPRMC,120000.000,{status},5000.0000,N,00100.0000,W,5.0,90.0,151126,,"
    if mode is not None:
        body += "," + mode
    return f"${body}*{nmea.nmea_cksum(body):02X}"


@pytest.mark.parametrize("status,mode", [("", "A"), ("X", "A"), ("V", "A"), ("A", "N")])
def test_rmc_rejects_unavailable_fixes(status, mode):
    line = sentence(status, mode)
    assert nmea.check_nmea_cksum(line)
    assert nmea.parse_nmea_gps(line) is False


@pytest.mark.parametrize("mode", ["A", "D", "", None])
def test_rmc_keeps_valid_and_legacy_fixes(mode):
    line = sentence(mode=mode)
    assert nmea.check_nmea_cksum(line)
    name, data = nmea.parse_nmea_gps(line)
    assert name == "gps"
    assert data["lat"] == pytest.approx(50)
    assert data["lon"] == pytest.approx(-1)
    assert data["speed"] == 5
