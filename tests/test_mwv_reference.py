"""Only explicitly relative or true MWV readings have a usable reference."""

import pytest

import nmea


def sentence(reference):
    body = f"WIMWV,90.0,{reference},10.0,N,A"
    return f"${body}*{nmea.nmea_cksum(body):02X}"


@pytest.mark.parametrize("reference", ["", "X", "r", "M"])
def test_unknown_reference_is_not_true_wind(reference):
    message = sentence(reference)
    assert nmea.check_nmea_cksum(message)
    assert nmea.parse_nmea_wind(message) is False


@pytest.mark.parametrize("reference, sensor", [("R", "wind"), ("T", "truewind")])
def test_known_reference_preserves_reading(reference, sensor):
    assert nmea.parse_nmea_wind(sentence(reference)) == (
        sensor,
        {"direction": 90.0, "speed": 10.0},
    )
