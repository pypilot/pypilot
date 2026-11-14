"""
Tests for N2K control of pypilot (pypilot/n2k_control.py, docs/n2k-control.md).

Messages are round-tripped through the nmea2000 library's encoder and
decoder so the field layouts are checked against the real codec.
"""
import json
import math

import pytest

nmea2000 = pytest.importorskip('nmea2000')
from nmea2000 import backend, pgns

import n2k_control as nc


class Setting:
    def __init__(self, value):
        self.value = value

    def set(self, value):
        self.value = value


class FakeClient:
    def __init__(self):
        self.sets = []
        self.sent = []
        self.watches = {}

    def set(self, name, value):
        self.sets.append((name, value))

    def send(self, msg):
        self.sent.append(msg)

    def watch(self, name, value=True):
        if value is False:
            self.watches.pop(name, None)
        else:
            self.watches[name] = value


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


OWN_NAME = 0x1234567890abcdef
OTHER_NAME = 0x1111


class Gateway:
    own_name = OWN_NAME
    manufacturer_information = 'PyPilot Autopilot'


def make_control(level='steer', bank=1, mode='compass', enabled=False, **values):
    clock = Clock()
    client = FakeClient()
    v = {'ap.enabled': enabled, 'ap.mode': mode, 'ap.preferred_mode': mode,
         'ap.modes': ['compass', 'gps', 'wind'], 'ap.heading': 100.0,
         'ap.heading_command': 90.0, 'ap.heading_error': 0, 'ap.tack.state': 'none',
         'ap.tack.direction': 'port', 'servo.flags': '', 'servo.controller': 'arduino',
         'servo.command': 0, 'imu.heading_lowpass': 100.0, 'imu.error': '',
         'imu.warning': '', 'rudder.source': 'none', 'rudder.angle': False,
         'profile': 'default', 'profiles': ['default', 'heavy']}
    v.update(values)
    settings = {'control': Setting(level), 'bank': Setting(bank), 'allowed': Setting([]),
                'jog_pulse': Setting(.3), 'description': Setting('saloon'),
                'output': Setting(True)}
    control = nc.N2KControl(client, v, settings, now=clock)
    for name in nc.WATCHES:  # as the bridge does
        client.watch(name)
    control.attach(Gateway())
    return control, client, clock


def encode(message):
    fn = getattr(pgns, 'encode_pgn_%d' % message.PGN, None) or \
        getattr(pgns, 'encode_pgn_%d_%s' % (message.PGN, message.id))
    return fn(message)


def decode(pgn, data):
    return getattr(pgns, 'decode_pgn_%d' % pgn)(int.from_bytes(data, 'little'), len(data) * 8)


def field(message, id):
    return message.get_field_by_id(id)


def sent(control, pgn):
    return [m for m in control.poll() if m.PGN == pgn]


class Source:
    def __init__(self, source=7, name=OTHER_NAME):
        self.source = source
        self.source_iso_name = type('IsoName', (), {'name': name})()


def command_payload(pgn, params):
    '''126208 Command group function with (field index, value bytes) params'''
    payload = bytes([1]) + pgn.to_bytes(3, 'little') + bytes([0xf8, len(params)])
    for index, value in params:
        payload += bytes([index]) + value
    return payload


def u16(radians):
    return round(radians / .0001).to_bytes(2, 'little')


def lau(text):
    return bytes([len(text) + 2, 1]) + text.encode()


def command(payload, source=None):
    '''the library's decoding of a 126208 payload, as the device hands it over'''
    source = source or Source()
    message = backend.decode(126208, payload, source.source, 100, 3)
    assert message is not None
    message.source_iso_name = source.source_iso_name
    return message


def handle(control, source, payload):
    return control.handle_group_function(command(payload, source))


def ack_codes(control):
    acks = [m for m in control.outbox if m.PGN == 126208]
    assert len(acks) == 1
    data = encode(acks[0])
    control.outbox = [m for m in control.outbox if m.PGN != 126208]
    params = [data[6 + i // 2] >> (4 * (i % 2)) & 0xf for i in range(data[5])]
    return data[4] & 0xf, params


def sets(client):
    return dict(client.sets)


# ---- 126208 commands, as the library decodes them ---------------------------

def test_command_parameters():
    payload = command_payload(127237, [(5, bytes([4])), (7, bytes([1])), (11, u16(math.pi / 2))])
    params = nc.command_parameters(command(payload))
    assert params[:2] == [(5, 4), (7, 1)]
    assert params[2][0] == 11 and params[2][1] == pytest.approx(math.pi / 2, abs=1e-4)


def test_command_parameters_signed_and_not_available():
    payload = command_payload(127237, [(10, (-1000).to_bytes(2, 'little', signed=True)),
                                       (11, b'\xff\xff')])
    params = nc.command_parameters(command(payload))
    assert params[0][1] == pytest.approx(-.1)
    assert params[1] == (11, None)


def test_command_parameters_string():
    params = nc.command_parameters(command(command_payload(126998, [(1, lau('PP:ap.mode'))])))
    assert params == [(1, 'PP:ap.mode')]


def test_unreadable_lookup_refused_as_out_of_range():
    control, client, clock = make_control()
    handle(control, Source(), command_payload(127237, [(5, bytes([6]))]))  # 6: out of range
    assert ack_codes(control) == (nc.PGN_ACK, [nc.PARAM_OUT_OF_RANGE])
    assert client.sets == []


# ---- path A: 127237 commands -------------------------------------------------

def test_engage_with_heading_to_steer():
    control, client, clock = make_control()
    payload = command_payload(127237, [(5, bytes([4])), (7, bytes([1])), (11, u16(math.radians(45)))])
    assert handle(control, Source(), payload)
    assert ack_codes(control) == (nc.PGN_ACK, [0, 0, 0])
    s = sets(client)
    assert s['ap.enabled'] is True
    assert s['ap.heading_command'] == pytest.approx(45, abs=.01)


def test_standby_allowed_at_monitor_but_engage_denied():
    control, client, clock = make_control(level='monitor', enabled=True)
    assert handle(control, Source(), command_payload(127237, [(5, bytes([4]))]))
    assert ack_codes(control) == (nc.PGN_ACCESS_DENIED, [nc.PARAM_ACCESS_DENIED])
    assert client.sets == []

    handle(control, Source(), command_payload(127237, [(5, bytes([0]))]))
    assert ack_codes(control) == (nc.PGN_ACK, [0])
    assert sets(client)['ap.enabled'] is False


def test_control_off_leaves_commands_to_library():
    control, client, clock = make_control(level='off')
    assert not handle(control, Source(), command_payload(127237, [(5, bytes([0]))]))
    assert client.sets == []


def test_heading_to_steer_refused_in_wind_mode():
    control, client, clock = make_control(mode='wind', enabled=True)
    handle(control, Source(), command_payload(127237, [(11, u16(1))]))
    assert ack_codes(control) == (nc.PGN_ACK, [nc.PARAM_TEMPORARY_ERROR])
    assert client.sets == []


def test_unsupported_field_rejects_whole_command():
    control, client, clock = make_control()
    handle(control, Source(), command_payload(127237, [(5, bytes([4])), (13, u16(.5))]))
    assert ack_codes(control) == (nc.PGN_ACK, [0, nc.PARAM_NOT_SUPPORTED])
    assert client.sets == []


def test_true_heading_reference_not_supported():
    control, client, clock = make_control()
    payload = command_payload(127237, [(5, bytes([4])), (7, bytes([0])), (11, u16(1))])
    handle(control, Source(), payload)
    assert ack_codes(control) == (nc.PGN_ACK, [0, nc.PARAM_NOT_SUPPORTED, 0])
    assert client.sets == []


def test_unavailable_mode_refused():
    control, client, clock = make_control()
    handle(control, Source(), command_payload(127237, [(5, bytes([5]))]))  # track -> nav
    assert ack_codes(control) == (nc.PGN_ACK, [nc.PARAM_TEMPORARY_ERROR])


def test_non_follow_up_jogs_starboard():
    control, client, clock = make_control()
    handle(control, Source(), command_payload(127237, [(5, bytes([1])), (9, bytes([1]))]))
    assert ack_codes(control) == (nc.PGN_ACK, [0, 0])
    assert client.sets[-1] == ('servo.command', -1)
    clock.t += .5
    control.poll()
    assert client.sets[-1] == ('servo.command', 0)


def test_allowed_list_restricts_steering_but_not_standby():
    control, client, clock = make_control(enabled=True)
    control.settings['allowed'].value = ['0x2222']
    handle(control, Source(), command_payload(127237, [(11, u16(1))]))
    assert ack_codes(control)[0] == nc.PGN_ACCESS_DENIED
    handle(control, Source(), command_payload(127237, [(5, bytes([0]))]))
    assert ack_codes(control) == (nc.PGN_ACK, [0])
    handle(control, Source(name=0x2222), command_payload(127237, [(5, bytes([4]))]))
    assert ack_codes(control) == (nc.PGN_ACK, [0])


# ---- path B: switch bank -----------------------------------------------------

def switch_message(instance=1, **channels):
    fields = {'instance': instance}
    for ch in range(1, 29):
        fields['switch%d' % ch] = nc.Lookup(channels.get('ch%d' % ch, nc.SWITCH_NA))
    data = encode(nc.make_message(127502, fields))
    message = decode(127502, data)
    message.source_iso_name = None
    return message


def test_switch_edge_triggered():
    control, client, clock = make_control(enabled=True)
    control.handle_message(switch_message(ch8=nc.SWITCH_ON))
    control.handle_message(switch_message(ch8=nc.SWITCH_ON))  # repeated ON: no action
    assert [s for s in client.sets if s[0] == 'ap.heading_command'] == [('ap.heading_command', 91.0)]
    control.handle_message(switch_message(ch8=nc.SWITCH_OFF))
    control.handle_message(switch_message(ch8=nc.SWITCH_ON))
    assert sets(client)['ap.heading_command'] == 92.0


def test_switch_rearms_after_timeout():
    control, client, clock = make_control(enabled=True)
    control.handle_message(switch_message(ch10=nc.SWITCH_ON))
    clock.t += 1.5
    control.poll()
    control.handle_message(switch_message(ch10=nc.SWITCH_ON))
    assert sets(client)['ap.heading_command'] == 110.0


def test_wind_mode_adjust_is_inverted():
    control, client, clock = make_control(mode='wind', enabled=True, **{'ap.heading_command': 40.0})
    control.handle_message(switch_message(ch8=nc.SWITCH_ON))  # +1 starboard
    assert sets(client)['ap.heading_command'] == 39.0


def test_other_bank_ignored():
    control, client, clock = make_control(enabled=True)
    control.handle_message(switch_message(instance=2, ch1=nc.SWITCH_ON))
    assert client.sets == []


def test_bank_setting_values():
    # n2k.switch.bank is a slider from -1 (off) to 252; 'off' was its old default
    for value, bank in [(-1, None), (-0.4, 0), (5, 5), (5.3, 5), (4.7, 5), ('5', 5), (252, 252),
                        (253, None), ('off', None), (None, None)]:
        control, client, clock = make_control(bank=value)
        assert control.bank() == bank, value


def test_mode_channel_off_does_not_disengage():
    control, client, clock = make_control(enabled=True)
    control.handle_message(switch_message(ch2=nc.SWITCH_OFF))
    assert client.sets == []


def test_standby_switch():
    control, client, clock = make_control(level='monitor', enabled=True)
    control.handle_message(switch_message(ch1=nc.SWITCH_ON))
    assert sets(client)['ap.enabled'] is False


def test_engage_in_new_mode_waits_for_heading_in_that_mode():
    control, client, clock = make_control()
    control.handle_message(switch_message(ch3=nc.SWITCH_ON))  # gps
    assert client.sets == [('ap.mode', 'gps')]
    control.on_value('ap.heading', 100.0)  # old mode, ignored
    control.on_value('ap.mode', 'gps')
    clock.t += .3
    control.on_value('ap.heading', 101.0)  # may still be in the old mode
    assert 'ap.enabled' not in sets(client)
    clock.t += .3
    control.on_value('ap.heading', 102.0)
    assert sets(client)['ap.heading_command'] == 102.0
    assert sets(client)['ap.enabled'] is True


def test_mode_change_while_engaged_with_heading_waits_for_mode():
    control, client, clock = make_control(mode='gps', enabled=True)
    payload = command_payload(127237, [(5, bytes([4])), (7, bytes([1])), (11, u16(math.radians(45)))])
    handle(control, Source(), payload)
    assert ack_codes(control) == (nc.PGN_ACK, [0, 0, 0])
    assert client.sets == [('ap.mode', 'compass')]  # pypilot would overwrite a command set now
    control.on_value('ap.mode', 'compass')
    clock.t += nc.ENGAGE_SETTLE
    control.on_value('ap.heading', 102.0)
    assert sets(client)['ap.heading_command'] == pytest.approx(45, abs=.01)


def test_mode_change_while_engaged_keeps_course():
    control, client, clock = make_control(enabled=True)
    control.handle_message(switch_message(ch3=nc.SWITCH_ON))
    assert client.sets == [('ap.mode', 'gps')]
    assert control.pending_engage is None


def test_engage_same_mode_uses_current_heading():
    control, client, clock = make_control()
    control.handle_message(switch_message(ch2=nc.SWITCH_ON))
    assert client.sets == [('ap.heading_command', 100.0), ('ap.enabled', True)]


def test_engage_timeout_names_mode_and_updates_active_alert():
    control, client, clock = make_control(**{'ap.modes': ['compass', 'gps', 'nav']})
    control.handle_message(switch_message(ch3=nc.SWITCH_ON))   # gps, never confirmed by pypilot
    clock.t += nc.ENGAGE_TIMEOUT + .1
    control.poll()
    assert control.alerts[nc.ALERT_REJECTED].text == 'Autopilot rejected: engage in gps mode timed out'
    clock.t += .1
    control.handle_message(switch_message(ch4=nc.SWITCH_ON))   # nav, also never confirmed
    control.values['ap.modes'] = ['compass']
    control.handle_message(switch_message(ch4=nc.SWITCH_OFF))
    control.handle_message(switch_message(ch2=nc.SWITCH_ON))   # compass engages at once
    control.values['ap.modes'] = ['compass', 'gps']
    control.values['ap.enabled'] = False
    control.handle_message(switch_message(ch4=nc.SWITCH_ON))   # nav not available
    clock.t += .1
    texts = [decode(126985, encode(m)) for m in control.poll() if m.PGN == 126985]
    assert [field(t, 'alertTextDescription').value for t in texts] == ['Autopilot rejected: nav mode not available']


def test_rejected_press_raises_caution_alert():
    control, client, clock = make_control()
    control.handle_message(switch_message(ch4=nc.SWITCH_ON))  # nav not available
    assert client.sets == []
    alerts = sent(control, 126983)
    assert [field(decode(126983, encode(m)), 'alertId').value for m in alerts] == [nc.ALERT_REJECTED]
    clock.t += nc.REJECT_ALERT_TIME + 1
    cleared = sent(control, 126983)
    assert field(decode(126983, encode(cleared[0])), 'alertState').raw_value == nc.STATE_NORMAL


def test_tack_and_cancel():
    control, client, clock = make_control(enabled=True)
    control.handle_message(switch_message(ch13=nc.SWITCH_ON))
    assert client.sets == [('ap.tack.direction', 'starboard'), ('ap.tack.state', 'begin')]
    control.values['ap.tack.state'] = 'waiting'
    control.handle_message(switch_message(ch13=nc.SWITCH_OFF))
    control.handle_message(switch_message(ch11=nc.SWITCH_ON))
    assert client.sets[-1] == ('ap.tack.state', 'none')


def test_next_profile_wraps():
    control, client, clock = make_control(**{'profile': 'heavy'})
    control.handle_message(switch_message(ch17=nc.SWITCH_ON))
    assert sets(client)['profile'] == 'default'


def test_jog_refused_while_engaged():
    control, client, clock = make_control(enabled=True)
    control.handle_message(switch_message(ch15=nc.SWITCH_ON))
    assert 'servo.command' not in sets(client)


def test_indicators():
    control, client, clock = make_control(mode='wind', enabled=True, **{'servo.flags': 'SYNC OVERTEMP_FAULT'})
    control.handle_message(switch_message(ch8=nc.SWITCH_ON))
    status = [m for m in control.outbox if m.PGN == 127501][-1]
    decoded = decode(127501, encode(status))
    state = [field(decoded, 'indicator%d' % ch).raw_value for ch in range(1, 29)]
    assert field(decoded, 'instance').value == 1
    assert state[nc.CH_STANDBY - 1] == nc.SWITCH_OFF
    assert state[5 - 1] == nc.SWITCH_ON   # wind
    assert state[2 - 1] == nc.SWITCH_OFF  # compass
    assert state[8 - 1] == nc.SWITCH_ON   # momentary echo of +1
    assert state[nc.CH_SERVO_FAULT - 1] == nc.SWITCH_ON
    assert state[19 - 1] == nc.SWITCH_NA
    clock.t += 1
    status = sent(control, 127501)[-1]
    assert field(decode(127501, encode(status)), 'indicator8').raw_value == nc.SWITCH_OFF


# ---- requested values -------------------------------------------------------

def press(control, channel):
    control.handle_message(switch_message(**{'ch%d' % channel: nc.SWITCH_ON}))
    control.handle_message(switch_message(**{'ch%d' % channel: nc.SWITCH_OFF}))


def test_presses_add_up_before_pypilot_reports():
    control, client, clock = make_control(enabled=True)
    for i in range(3):
        press(control, 8)
    commands = [v for n, v in client.sets if n == 'ap.heading_command']
    assert commands == [91.0, 92.0, 93.0]
    control.on_value('ap.heading_command', 91.0)  # pypilot catching up: still 93 requested
    press(control, 8)
    assert sets(client)['ap.heading_command'] == 94.0


def test_status_shows_only_what_pypilot_reports():
    control, client, clock = make_control(enabled=True)
    press(control, 8)
    decoded = decode(127237, encode(control.heading_track_control()))
    assert field(decoded, 'headingToSteerCourse').value == pytest.approx(math.radians(90), abs=1e-3)
    control.on_value('ap.heading_command', 91.0)
    decoded = decode(127237, encode(control.heading_track_control()))
    assert field(decoded, 'headingToSteerCourse').value == pytest.approx(math.radians(91), abs=1e-3)

    press(control, nc.CH_STANDBY)
    assert control.indicators(clock.t)[nc.CH_STANDBY - 1] == nc.SWITCH_OFF  # still engaged
    control.on_value('ap.enabled', False)
    assert control.indicators(clock.t)[nc.CH_STANDBY - 1] == nc.SWITCH_ON


def test_command_result_sent_without_waiting():
    control, client, clock = make_control(enabled=True)
    press(control, nc.CH_STANDBY)  # echoes 127501 at once, still engaged
    control.poll()
    clock.t += .05
    control.on_value('ap.enabled', False)
    status = sent(control, 127501)  # well within STATUS_MIN_PERIOD of the echo
    assert status and field(decode(127501, encode(status[-1])), 'indicator1').raw_value == nc.SWITCH_ON
    clock.t += .05
    control.on_value('ap.enabled', True)  # a further change is rate limited again
    assert sent(control, 127501) == []


def test_unreported_request_expires():
    control, client, clock = make_control(enabled=True)
    press(control, 8)
    clock.t += nc.REQUEST_TIMEOUT  # pypilot never reported 91, so it refused or changed it
    press(control, 8)
    assert sets(client)['ap.heading_command'] == 91.0


def test_rounded_report_matches_request():
    control, client, clock = make_control(enabled=True)
    press(control, 8)
    control.on_value('ap.heading_command', 91.004)
    assert 'ap.heading_command' not in control.requested


# ---- 127237 / 127245 status ------------------------------------------------

def test_heading_track_control_status_compass():
    control, client, clock = make_control(enabled=True)
    decoded = decode(127237, encode(control.heading_track_control()))
    assert field(decoded, 'steeringMode').raw_value == nc.STEERING_HEADING
    assert field(decoded, 'headingReference').raw_value == nc.REFERENCE_MAGNETIC
    assert field(decoded, 'headingToSteerCourse').value == pytest.approx(math.radians(90), abs=1e-3)
    assert field(decoded, 'vesselHeading').value == pytest.approx(math.radians(100), abs=1e-3)


def test_heading_track_control_status_wind_reports_compass_equivalent():
    control, client, clock = make_control(mode='wind', enabled=True,
                                          **{'ap.heading_error': 10, 'imu.heading_lowpass': 200.0})
    decoded = decode(127237, encode(control.heading_track_control()))
    assert field(decoded, 'headingToSteerCourse').value == pytest.approx(math.radians(190), abs=1e-3)


def test_heading_track_control_standby():
    control, client, clock = make_control(**{'servo.command': -.5})
    decoded = decode(127237, encode(control.heading_track_control()))
    assert field(decoded, 'steeringMode').raw_value == nc.STEERING_MAIN
    assert field(decoded, 'override').raw_value == nc.YES
    assert field(decoded, 'commandedRudderDirection').raw_value == nc.RUDDER_STARBOARD
    assert field(decoded, 'headingToSteerCourse').value is None


def test_127237_sent_periodically_and_on_change():
    control, client, clock = make_control()
    assert len(sent(control, 127237)) == 1
    clock.t += .1
    assert sent(control, 127237) == []
    control.values['ap.enabled'] = True
    clock.t += .2
    assert len(sent(control, 127237)) == 1


def test_rudder_only_for_local_sensor():
    control, client, clock = make_control(**{'rudder.source': 'serial', 'rudder.angle': 10.0})
    decoded = decode(127245, encode(control.rudder()))
    assert field(decoded, 'position').value == pytest.approx(math.radians(-10), abs=1e-3)
    control.values['rudder.source'] = 'N2K12'
    assert control.rudder() is None


# ---- path C: PP: text commands ----------------------------------------------

def text(control, command, source=None):
    handle(control, source or Source(), command_payload(126998, [(1, lau(command))]))
    return ack_codes(control)


def description2(control):
    configs = [m for m in control.outbox if m.PGN == 126998]
    control.outbox = []
    decoded = decode(126998, encode(configs[-1]))
    assert field(decoded, 'installationDescription1').value == control.settings['description'].value
    return field(decoded, 'installationDescription2').value


def test_text_get_watched_value():
    control, client, clock = make_control()
    assert text(control, 'PP:ap.mode') == (nc.PGN_ACK, [0])
    control.poll()
    assert control.description2 == 'ap.mode="compass"'


def test_text_get_unwatched_value():
    control, client, clock = make_control()
    text(control, 'PP:servo.voltage')
    assert client.watches['servo.voltage'] is True
    control.on_value('servo.voltage', 12.5)
    control.poll()
    assert control.description2 == 'servo.voltage=12.5'
    # it stays watched for a while so repeat reads are immediate, then the watch is dropped
    assert client.watches['servo.voltage'] is True
    clock.t += nc.TEXT_LEASE + 1
    control.poll()
    assert 'servo.voltage' not in client.watches
    assert 'servo.voltage' not in control.values


def test_text_repeat_read_answered_immediately_from_lease():
    control, client, clock = make_control()
    text(control, 'PP:servo.voltage')
    control.on_value('servo.voltage', 12.5)
    control.on_value('servo.voltage', 12.1)  # kept up to date while leased
    text(control, 'PP:servo.voltage')
    assert description2(control) == 'servo.voltage=12.1'  # answered in the handler, no poll


def test_text_does_not_reuse_value_after_lease():
    control, client, clock = make_control()
    text(control, 'PP:servo.voltage')
    control.on_value('servo.voltage', 12.5)
    clock.t += nc.TEXT_LEASE + 1
    control.poll()
    text(control, 'PP:servo.voltage')
    assert control.pending_texts  # waits for a fresh value
    control.on_value('servo.voltage', 12.0)
    assert description2(control) == 'servo.voltage=12.0'


def test_text_lease_limit_drops_oldest():
    control, client, clock = make_control()
    for i in range(nc.MAX_TEXT_LEASES + 1):
        text(control, 'PP:x.v%d' % i)
        control.on_value('x.v%d' % i, i)
        clock.t += .01
    assert len(control.leases) == nc.MAX_TEXT_LEASES
    assert 'x.v0' not in client.watches and 'x.v1' in client.watches


def test_text_get_unknown_times_out():
    control, client, clock = make_control()
    text(control, 'PP:nope')
    clock.t += 3
    control.poll()
    assert control.description2 == 'ERR unknown nope'


def test_text_set_requires_full():
    control, client, clock = make_control(level='steer')
    text(control, 'PP:ap.pilot.basic.P=0.004')
    assert client.sent == []
    assert description2(control).startswith('ERR')


@pytest.mark.parametrize('name', ['', 'ap.mode\nservo.command', 'ap.mode\rservo.command',
                                  'ap mode', 'ap-mode', 'ap/mode', 'ap.mode\x00suffix'])
@pytest.mark.parametrize('write', [False, True])
def test_text_rejects_invalid_names(name, write):
    control, client, clock = make_control(level='full')
    watches = dict(client.watches)
    control.text_command(name + ('=1' if write else '@0'), Source())
    assert control.description2 == 'ERR bad name'
    assert client.sent == []
    assert client.watches == watches
    assert control.pending_texts == []


def test_text_checks_name_before_write_permission():
    control, client, clock = make_control(level='monitor')
    control.text_command('bad name=1', Source())
    assert control.description2 == 'ERR bad name'
    assert client.sent == []


def test_text_serializes_parsed_json_on_write():
    control, client, clock = make_control(level='full')
    control.text_command('test_value.1={\n"text": "a\\nb",\n"count": 2}', Source())
    assert client.sent == ['test_value.1={"text": "a\\nb", "count": 2}\n']
    assert len(client.sent[0].splitlines()) == 1


def test_text_set_and_read_back():
    control, client, clock = make_control(level='full', values={
        'ap.pilot.basic.P': {'type': 'RangeProperty', 'min': 0, 'max': .03, 'writable': True}})
    text(control, 'PP:ap.pilot.basic.P=0.004')
    assert client.sent == ['ap.pilot.basic.P=0.004\n']
    control.on_value('ap.pilot.basic.P', .003)  # the value before the write: not the answer
    assert control.description2 == 'PP:HELP'
    control.on_value('ap.pilot.basic.P', .004)  # reported as soon as pypilot applies it
    assert control.description2 == 'OK ap.pilot.basic.P=0.004'


def test_text_set_reports_clamped_value_after_settle():
    control, client, clock = make_control(level='full')
    text(control, 'PP:ap.pilot.basic.P=5')
    control.on_value('ap.pilot.basic.P', .03)  # clamped to its maximum
    control.poll()
    assert control.description2 == 'PP:HELP'
    clock.t += nc.WRITE_SETTLE
    control.poll()
    assert control.description2 == 'OK ap.pilot.basic.P=0.03'


def test_text_set_same_value_as_live_is_immediate():
    control, client, clock = make_control(level='full')
    text(control, 'PP:ap.mode="compass"')
    assert description2(control) == 'OK ap.mode="compass"'


def test_text_tags():
    control, client, clock = make_control()
    text(control, 'PP#7a:ap.mode')
    assert description2(control) == '#7a ap.mode="compass"'
    text(control, 'PP#x1:servo.voltage')
    control.on_value('servo.voltage', 12.5)
    assert description2(control) == '#x1 servo.voltage=12.5'
    text(control, 'PP#toolongtag:ap.mode')
    assert description2(control).startswith('ERR tag')
    text(control, 'PP#9:INFO nope')
    assert description2(control) == '#9 ERR unknown nope'


def descriptions2(control):
    '''every installation description 2 sent, in order'''
    configs = [m for m in control.outbox if m.PGN == 126998]
    control.outbox = []
    return [field(decode(126998, encode(m)), 'installationDescription2').value for m in configs]


def test_text_requests_from_several_heads_are_all_answered():
    # neither value is watched yet, so both requests wait; the second must not drop the first
    control, client, clock = make_control()
    text(control, 'PP#a:servo.voltage', Source(source=35))
    text(control, 'PP#b:servo.current', Source(source=101))
    assert len(control.pending_texts) == 2
    control.on_value('servo.current', 2.0)
    control.on_value('servo.voltage', 12.5)
    assert descriptions2(control) == ['#b servo.current=2.0', '#a servo.voltage=12.5']
    assert control.pending_texts == []


def test_text_requests_time_out_independently():
    control, client, clock = make_control()
    text(control, 'PP#a:nope.one')
    clock.t += nc.TEXT_TIMEOUT / 2
    text(control, 'PP#b:nope.two')
    clock.t += nc.TEXT_TIMEOUT / 2
    control.poll()
    assert control.description2 == '#a ERR unknown nope.one'
    assert [p['tag'] for p in control.pending_texts] == ['b']
    clock.t += nc.TEXT_TIMEOUT / 2
    control.poll()
    assert control.description2 == '#b ERR unknown nope.two'


def test_text_busy_when_too_many_requests_wait():
    control, client, clock = make_control()
    for i in range(nc.MAX_PENDING_TEXT):
        text(control, 'PP#%d:nope.%d' % (i, i))
    control.outbox = []
    text(control, 'PP#x:nope.x')
    assert descriptions2(control) == ['#x ERR busy']
    assert len(control.pending_texts) == nc.MAX_PENDING_TEXT


def test_text_read_value_list():
    # 'values' isn't reported by pypilotClient: the bridge keeps it as client.values
    info = {'ap.mode': {'type': 'EnumProperty', 'choices': ['compass', 'gps'], 'writable': True},
            'servo.voltage': {'type': 'SensorValue'}}
    control, client, clock = make_control()
    client.values = Setting(info)
    full = json.dumps(info, separators=(',', ':'))
    got, offset = '', 0
    while offset < len(full):
        text(control, 'PP#4:values@%d' % offset)
        header, chunk = description2(control).split('=', 1)
        assert header == '#4 values@%d/%d' % (offset, len(full))
        got += chunk
        offset += len(chunk)
    assert json.loads(got) == info


def test_text_chunked_read():
    points = [[round(i * 1.1, 1), -i, i * 2] for i in range(30)]
    control, client, clock = make_control(**{'imu.compass.calibration.points': points})
    client.watch('imu.compass.calibration.points')
    full = json.dumps(points, separators=(',', ':'))
    got, offset = '', 0
    while True:
        text(control, 'PP#3:imu.compass.calibration.points@%d' % offset)
        result = description2(control)
        assert len(result) <= nc.TEXT_LENGTH
        header, chunk = result.split('=', 1)
        assert header == '#3 imu.compass.calibration.points@%d/%d' % (offset, len(full))
        got += chunk
        offset += len(chunk)
        if offset >= len(full):
            break
    assert json.loads(got) == points
    text(control, 'PP:imu.compass.calibration.points@%d' % (len(full) + 1))
    assert description2(control).startswith('ERR offset past end')
    text(control, 'PP:imu.compass.calibration.points@x')
    assert description2(control).startswith('ERR offset')


def test_next_mode_channel_cycles_without_engaging():
    control, client, clock = make_control()
    control.handle_message(switch_message(ch18=nc.SWITCH_ON))
    assert client.sets == [('ap.mode', 'gps')]
    control.handle_message(switch_message(ch18=nc.SWITCH_OFF))
    control.handle_message(switch_message(ch18=nc.SWITCH_ON))
    control.handle_message(switch_message(ch18=nc.SWITCH_OFF))
    control.handle_message(switch_message(ch18=nc.SWITCH_ON))
    assert [v for n, v in client.sets if n == 'ap.mode'] == ['gps', 'wind', 'compass']
    assert 'ap.enabled' not in sets(client)


def test_text_set_denied_names():
    control, client, clock = make_control(level='full', values={
        'servo.flags': {'type': 'Value'}})
    for command in ['PP:n2k.transport="none"', 'PP:imu.alignmentQ=[1,0,0,0]',
                    'PP:imu.compass.calibration.points=[]', 'PP:servo.flags=0',
                    'PP:ap.mode=compass']:
        text(control, command)
        assert description2(control).startswith('ERR'), command
    assert client.sent == []


def test_text_set_allows_n2k_outputs():
    control, client, clock = make_control(level='full')
    text(control, 'PP:n2k.output.rudder=true')
    assert client.sent == ['n2k.output.rudder=true\n']


def test_text_info_and_list():
    info = {'ap.pilot.basic.%s' % g: {'type': 'RangeProperty', 'min': 0, 'max': 1, 'writable': True}
            for g in ['P', 'D', 'DD', 'PR', 'FF']}
    info.update({'ap.pilot.simple.%s%d' % (g, i): {} for g in 'PID' for i in range(5)})
    control, client, clock = make_control(values=info)
    text(control, 'PP:INFO ap.pilot.basic.P')
    assert description2(control) == 'RangeProperty 0..1 writable'
    text(control, 'PP:LIST ap.pilot.basic')
    assert description2(control) == 'ap.pilot.basic: .D .DD .FF .P .PR'
    text(control, 'PP:LIST ap.pilot.simple')
    first = description2(control)
    assert first.startswith('ap.pilot.simple: .D0 .D1') and first.endswith('(+1)')
    assert len(first) <= nc.TEXT_LENGTH
    text(control, 'PP:LIST ap.pilot.simple 1')
    second = description2(control)
    assert second.startswith('ap.pilot.simple: ') and second.endswith('.P4')
    text(control, 'PP:LIST ap.pilot.simple 2')
    assert description2(control) == 'ERR no page 2'


def test_plain_description_is_stored():
    control, client, clock = make_control(level='monitor')
    text(control, 'nav station')
    assert control.settings['description'].value == 'nav station'
    assert control.gateway.installation_description1 == 'nav station'


def test_description2_not_writable():
    control, client, clock = make_control()
    handle(control, Source(), command_payload(126998, [(2, lau('x'))]))
    assert ack_codes(control) == (nc.PGN_ACK, [nc.PARAM_ACCESS_DENIED])


def test_long_result_truncated():
    control, client, clock = make_control(**{'profiles': ['p%d' % i for i in range(40)]})
    text(control, 'PP:profiles')
    control.poll()
    assert len(control.description2) == nc.TEXT_LENGTH
    assert control.description2.endswith('...')


# ---- alerts -----------------------------------------------------------------

def alert_response(alert_id, occurrence, response, source_name=OWN_NAME):
    fields = {'alertType': nc.Lookup(2), 'alertCategory': nc.Lookup(1),
              'alertSystem': nc.ALERT_SYSTEM, 'alertSubSystem': 0, 'alertId': alert_id,
              'dataSourceNetworkIdName': source_name, 'dataSourceInstance': 0,
              'dataSourceIndexSource': 0, 'alertOccurrenceNumber': occurrence,
              'acknowledgeSourceNetworkIdName': OTHER_NAME,
              'responseCommand': nc.Lookup(response), 'reserved_194': None}
    message = decode(126984, encode(nc.make_message(126984, fields)))
    message.source_iso_name = None
    return message


def test_servo_fault_alert_lifecycle():
    control, client, clock = make_control(enabled=True)
    control.poll()
    control.values['servo.flags'] = 'SYNC OVERCURRENT_FAULT'
    clock.t += 1
    messages = control.poll()
    alert = decode(126983, encode([m for m in messages if m.PGN == 126983][0]))
    alert_text = decode(126985, encode([m for m in messages if m.PGN == 126985][0]))
    assert field(alert, 'alertId').value == nc.ALERT_SERVO_OVERCURRENT
    assert field(alert, 'alertType').raw_value == nc.ALERT_ALARM
    assert field(alert, 'alertState').raw_value == nc.STATE_ACTIVE
    assert field(alert_text, 'alertTextDescription').value == 'Autopilot servo overcurrent'

    control.handle_message(alert_response(nc.ALERT_SERVO_OVERCURRENT, 1, nc.RESPONSE_ACKNOWLEDGE))
    clock.t += .1
    alert = decode(126983, encode(sent(control, 126983)[0]))
    assert field(alert, 'alertState').raw_value == nc.STATE_ACKNOWLEDGED

    control.values['servo.flags'] = 'SYNC'
    clock.t += .1
    alert = decode(126983, encode(sent(control, 126983)[0]))
    assert field(alert, 'alertState').raw_value == nc.STATE_NORMAL


def test_alert_response_for_other_device_ignored():
    control, client, clock = make_control(**{'imu.error': 'No IMU'})
    control.poll()
    control.handle_message(alert_response(nc.ALERT_IMU_ERROR, 1, nc.RESPONSE_ACKNOWLEDGE, OTHER_NAME))
    assert not control.alerts[nc.ALERT_IMU_ERROR].acknowledged


def test_dismiss_switch_acknowledges_alerts():
    control, client, clock = make_control(**{'imu.warning': 'compass calibration age'})
    control.poll()
    control.handle_message(switch_message(ch16=nc.SWITCH_ON))
    assert control.alerts[nc.ALERT_COMPASS_WARNING].acknowledged


def test_mode_fallback_alert():
    control, client, clock = make_control(mode='compass', enabled=True, **{'ap.preferred_mode': 'wind'})
    alerts = [decode(126983, encode(m)) for m in sent(control, 126983)]
    assert [field(a, 'alertId').value for a in alerts] == [nc.ALERT_MODE_FALLBACK]


def test_no_alerts_without_output():
    control, client, clock = make_control(**{'imu.error': 'No IMU'})
    control.settings['output'].value = False
    assert sent(control, 126983) == []


class OwnValues:
    """pypilotClient.values for a process that registered values itself"""
    def __init__(self, **values):
        self.values = {name.replace('__', '.'): Setting(v) for name, v in values.items()}
        self.value = None


def test_text_reads_values_registered_by_this_process():
    # the n2k.* settings belong to pypilot's n2k process: never received, read directly
    control, client, clock = make_control()
    client.values = OwnValues(n2k__switch__bank=5)
    text(control, 'PP:n2k.switch.bank')
    assert description2(control) == 'n2k.switch.bank=5'
    assert 'n2k.switch.bank' not in client.watches


def test_text_write_of_own_value_reported_after_settle():
    control, client, clock = make_control(level='full')
    client.values = OwnValues(n2k__output__rudder=False)
    text(control, 'PP:n2k.output.rudder=true')
    assert client.sent == ['n2k.output.rudder=true\n']
    client.values.values['n2k.output.rudder'].value = True  # the server hands the write back to us
    clock.t += nc.WRITE_SETTLE
    control.poll()
    assert control.description2 == 'OK n2k.output.rudder=true'


def test_text_list_uses_client_value_list():
    # pypilotClient keeps the 'values' list in client.values.value, not in receive()
    control, client, clock = make_control()
    client.values = Setting({'ap.pilot.basic.P': {'type': 'RangeProperty', 'min': 0, 'max': 1}})
    text(control, 'PP:LIST ap.pilot.basic')
    assert description2(control) == 'ap.pilot.basic: .P'
    text(control, 'PP:INFO ap.pilot.basic.P')
    assert description2(control) == 'RangeProperty 0..1'
