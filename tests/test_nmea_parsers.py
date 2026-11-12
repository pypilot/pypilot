"""
Regression tests for the NMEA sentence parsers in pypilot.nmea.

These are pure-function parsers so we can exercise them without the
autopilot's sensor/socket/thread machinery.
"""
import pytest


@pytest.fixture
def nmea_module():
    # Imported here (not at module load) so conftest can finish setting up
    # sys.path and the `_` builtin first.
    import nmea
    return nmea


def _full_sentence(body):
    """Wrap a sentence body with $ and *CK."""
    import nmea
    return f'${body}*{nmea.nmea_cksum(body):02X}'


def test_cksum_roundtrip(nmea_module):
    body = 'APRMC,152030.000,A,3723.2475,N,12158.3416,W,0.12,311.8,030725,,,A'
    sentence = _full_sentence(body)
    assert nmea_module.check_nmea_cksum(sentence)


def test_cksum_detects_corruption(nmea_module):
    sentence = _full_sentence('APRMC,152030.000,A,3723.2475,N')
    # flip one character
    corrupted = sentence[:10] + 'X' + sentence[11:]
    assert not nmea_module.check_nmea_cksum(corrupted)


def test_apb_rejects_short_sentence(nmea_module):
    # Short APB — only 3 fields where 14 are expected. Should not crash.
    short = '$APAPB,A,A,0.1*00'
    assert nmea_module.parse_nmea_apb(short) is False


def test_apb_accepts_full_sentence(nmea_module):
    # Minimum-viable APB with all 14 fields
    body = 'APAPB,A,A,0.05,R,N,A,A,12.3,M,WP1,45.0,M,67.0,M'
    result = nmea_module.parse_nmea_apb('$' + body + '*00')
    assert result is not False
    name, payload = result
    assert name == 'apb'
    assert payload['mode'] == 'compass'
    assert abs(payload['track'] - 67.0) < 1e-6
    # xte sign follows direction-to-steer (R => +, L => -), and the
    # parser clamps to 0.15 miles.
    assert abs(payload['xte'] - 0.05) < 1e-6


def test_apb_left_xte_is_negative(nmea_module):
    body = 'APAPB,A,A,0.05,L,N,A,A,12.3,M,WP1,45.0,M,67.0,M'
    _, payload = nmea_module.parse_nmea_apb('$' + body + '*00')
    assert payload['xte'] < 0


def test_apb_clamps_xte(nmea_module):
    body = 'APAPB,A,A,9.99,R,N,A,A,12.3,M,WP1,45.0,M,67.0,M'
    _, payload = nmea_module.parse_nmea_apb('$' + body + '*00')
    assert payload['xte'] == 0.15


def test_mwv_apparent_knots(nmea_module):
    # MWV with speed unit 'N' (knots) passes through unchanged.
    result = nmea_module.parse_nmea_wind('$APMWV,90.0,R,10.0,N,A*00')
    assert result is not False
    name, payload = result
    assert name == 'wind'
    assert abs(payload['speed'] - 10.0) < 1e-6


def test_mwv_true_vs_apparent(nmea_module):
    apparent = nmea_module.parse_nmea_wind('$APMWV,90.0,R,10.0,N,A*00')
    true = nmea_module.parse_nmea_wind('$APMWV,90.0,T,10.0,N,A*00')
    assert apparent[0] == 'wind'
    assert true[0] == 'truewind'


@pytest.mark.parametrize('status', ['V', 'N', ''])
def test_rsa_rejects_invalid_rudder(nmea_module, status):
    sentence = _full_sentence(f'IIRSA,12.5,{status},,')
    assert nmea_module.check_nmea_cksum(sentence)
    assert nmea_module.parse_nmea_rudder(sentence) is False


def test_rsa_accepts_valid_rudder(nmea_module):
    sentence = _full_sentence('IIRSA,12.5,A,,')
    assert nmea_module.parse_nmea_rudder(sentence) == ('rudder', {'angle': -12.5})

@pytest.mark.parametrize('unit', ['', 'X'])
def test_mwv_rejects_unknown_speed_units(nmea_module, unit):
    sentence = _full_sentence(f'WIMWV,90.0,R,10.0,{unit},A')
    assert nmea_module.parse_nmea_wind(sentence) is False


@pytest.mark.parametrize('unit, expected', [('N', 10.0), ('K', 5.3995), ('M', 19.4384)])
def test_mwv_converts_supported_speed_units(nmea_module, unit, expected):
    sentence = _full_sentence(f'WIMWV,90.0,R,10.0,{unit},A')
    assert nmea_module.parse_nmea_wind(sentence)[1]['speed'] == pytest.approx(expected)
