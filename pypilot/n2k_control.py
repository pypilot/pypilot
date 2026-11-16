#!/usr/bin/env python
#
# NMEA 2000 control of pypilot using only standard PGNs.
# See docs/n2k-control.md for the design. Three control paths:
#   A. 126208 Command -> 127237 Heading/Track Control
#   B. 127502 Switch Bank Control in, 127501 Binary Switch Bank Status out
#   C. 'PP:' text commands written into 126998 installation description 1
# plus 126983/126985 alerts out and 126984 alert responses in.
#
# This module only builds and interprets messages; N2KBridge in n2k.py owns
# the transport and sends whatever ends up in N2KControl.outbox.

import json
import math
import time

import nmea2000

CONTROL_LEVELS = ['off', 'monitor', 'steer', 'full']
OFF, MONITOR, STEER, FULL = range(4)

MODES = ['compass', 'gps', 'nav', 'wind', 'true wind']

# 127237 lookups
STEERING_MAIN, STEERING_NFU, STEERING_FU, STEERING_STANDALONE, STEERING_HEADING, STEERING_TRACK = range(6)
REFERENCE_TRUE, REFERENCE_MAGNETIC = 0, 1
RUDDER_NO_ORDER, RUDDER_STARBOARD, RUDDER_PORT = 0, 1, 2
NO, YES = 0, 1

# 126208 acknowledge codes
PGN_ACK, PGN_NOT_SUPPORTED, PGN_NOT_AVAILABLE, PGN_ACCESS_DENIED = 0, 1, 2, 3
PARAM_ACK, PARAM_INVALID, PARAM_TEMPORARY_ERROR, PARAM_OUT_OF_RANGE, PARAM_ACCESS_DENIED, PARAM_NOT_SUPPORTED = range(6)

# 127501 / 127502 switch states
SWITCH_OFF, SWITCH_ON, SWITCH_NA = 0, 1, 3

# switch bank channels
CH_STANDBY = 1
CH_MODES = {2: 'compass', 3: 'gps', 4: 'nav', 5: 'wind', 6: 'true wind'}
CH_ADJUST = {7: -1, 8: 1, 9: -10, 10: 10}
CH_TACK = {11: None, 12: 'port', 13: 'starboard'}
CH_JOG = {14: -1, 15: 1}
CH_DISMISS = 16
CH_NEXT_PROFILE = 17
CH_NEXT_MODE = 18
CH_SERVO_FAULT, CH_IMU_ERROR, CH_MODE_FALLBACK = 21, 22, 23
MOMENTARY_CHANNELS = list(CH_ADJUST) + list(CH_JOG) + [CH_NEXT_PROFILE, CH_NEXT_MODE]

MOMENTARY_TIME = .5   # momentary channels show ON this long after acting
SWITCH_REARM_TIME = 1 # a channel left ON re-arms after this long
ENGAGE_TIMEOUT = 3
ENGAGE_SETTLE = .5    # pypilot reports the new mode before its heading is in that mode
REQUEST_TIMEOUT = 2   # a value we set that pypilot hasn't reported by then was refused or changed
REJECT_ALERT_TIME = 5
SILENCE_TIME = 30
STATUS_PERIOD = 1
STATUS_MIN_PERIOD = .2
ALERT_TEXT_PERIOD = 10
JOG_SPEED = 1
TEXT_LEASE = 30       # a value read with PP: stays watched this long, so repeat reads are immediate
MAX_TEXT_LEASES = 32
WRITE_SETTLE = .5     # longest a PP: write waits for pypilot to report the new value
TEXT_TIMEOUT = 2
MAX_PENDING_TEXT = 16  # PP: requests waiting for a value, from all control heads together
TEXT_LENGTH = 70      # usable characters in an installation description

# alert type, category and state lookups
ALERT_ALARM, ALERT_WARNING, ALERT_CAUTION = 2, 5, 8
CATEGORY_NAVIGATIONAL, CATEGORY_TECHNICAL = 0, 1
STATE_NORMAL, STATE_ACTIVE, STATE_SILENCED, STATE_ACKNOWLEDGED = 1, 2, 3, 4
RESPONSE_ACKNOWLEDGE, RESPONSE_SILENCE = 0, 1
ALERT_SYSTEM = 1

ALERT_SERVO_OVERCURRENT, ALERT_SERVO_OVERTEMP, ALERT_BAD_VOLTAGE, ALERT_DRIVER, \
    ALERT_RUDDER_LIMIT, ALERT_IMU_ERROR, ALERT_COMPASS_WARNING, ALERT_MODE_FALLBACK, \
    ALERT_REJECTED = range(1, 10)

SERVO_FAULTS = {'OVERTEMP_FAULT', 'OVERCURRENT_FAULT', 'BADVOLTAGE_FAULT',
                'PORT_PIN_FAULT', 'STARBOARD_PIN_FAULT', 'MIN_RUDDER_FAULT',
                'MAX_RUDDER_FAULT', 'BAD_FUSES', 'PORT_OVERCURRENT_FAULT',
                'STARBOARD_OVERCURRENT_FAULT', 'DRIVER_TIMEOUT'}

# pypilot values the control logic needs to follow
WATCHES = ['ap.enabled', 'ap.mode', 'ap.modes', 'ap.preferred_mode', 'ap.heading',
           'ap.heading_command', 'ap.heading_error', 'ap.tack.state',
           'ap.tack.direction', 'servo.flags', 'servo.controller', 'servo.command',
           'rudder.angle', 'rudder.range', 'rudder.source', 'imu.heading_lowpass',
           'imu.error', 'imu.warning', 'apb.track', 'profile', 'profiles', 'values']


def deg2rad(degrees):
    return degrees * math.pi / 180


def rad2deg(radians):
    return radians * 180 / math.pi


def is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def heading_rad(degrees):
    # unsigned 0..2pi field
    if not is_number(degrees):
        return None
    return deg2rad(degrees % 360)


class Lookup(int):
    '''Raw value for a LOOKUP field; the encoder only accepts these in raw_value.'''


def make_message(pgn, fields, id='', destination=255, priority=6):
    nfields = []
    for name, value in fields.items():
        if isinstance(value, Lookup):
            nfields.append(nmea2000.NMEA2000Field(id=name, value=None, raw_value=int(value)))
        else:
            nfields.append(nmea2000.NMEA2000Field(id=name, value=value))
    return nmea2000.NMEA2000Message(PGN=pgn, id=id, source=0, destination=destination,
                                    priority=priority, fields=nfields)


# The PGNs we accept 126208 Commands for
COMMAND_PGNS = (127237, 126998)
# 127237 fields that are lookups, refused as out of range when unreadable
COMMAND_LOOKUPS = (5, 7, 9)


def field(message, name):
    try:
        return message.get_field_by_id(name).value
    except ValueError:
        return None


def command_parameters(message):
    '''A decoded 126208 Command's parameters as [(field index, value), ...].
    nmea2000 types each value by the field of the commanded PGN it refers to;
    its raw_value is a lookup's code, or a number's or string's value. A
    value that isn't available, or is out of range, is None.'''
    params = []
    for entry in field(message, '##list##') or []:
        value = entry['value']
        params.append((entry['parameter'].value, None if value.value is None else value.raw_value))
    return params


def build_ack(destination, pgn, pgn_error, param_errors):
    params = [{'parameter': nmea2000.NMEA2000Field(id='parameter', value=None, raw_value=e)}
              for e in param_errors]
    return make_message(126208, {'functionCode': Lookup(2),
                                 'pgn': pgn,
                                 'pgnErrorCode': Lookup(pgn_error),
                                 'transmissionIntervalPriorityErrorCode': Lookup(0),
                                 'numberOfParameters': len(params),
                                 # repeating fields: '##list##' in current nmea2000, 'list' in older releases
                                 '##list##': params,
                                 'list': params},
                        id='nmeaAcknowledgeGroupFunction', destination=destination, priority=3)


def truncate(text, length=TEXT_LENGTH):
    text = str(text)
    if len(text) > length:
        return text[:length - 3] + '...'
    return text


def split_text_command(text):
    '''"PP:command" or "PP#tag:command" -> (tag or None, command), None if malformed'''
    if text.startswith('PP:'):
        return None, text[3:].strip()
    tag, sep, command = text[3:].partition(':')
    if not sep or not tag or len(tag) > 8 or not tag.isalnum():
        return None
    return tag, command.strip()


def values_match(value, expected):
    if is_number(value) and is_number(expected):
        return abs(value - expected) <= 1e-9 + 1e-6 * abs(expected)
    return value == expected


def requested_match(value, requested):
    '''pypilot reporting a value we set, allowing for it rounding numbers'''
    if is_number(value) and is_number(requested):
        return abs(value - requested) < .01
    return value == requested


def denied_write(name):
    if name.startswith('n2k.'):
        return not (name.startswith('n2k.output.') or name == 'n2k.installation_description')
    return name in ('servo.calibration', 'imu.alignmentQ') or name.endswith('.calibration.points')


class Alert(object):
    def __init__(self, id, category, text):
        self.id = id
        self.category = category
        self.text = text
        self.type = ALERT_WARNING
        self.active = False
        self.occurrence = 0
        self.acknowledged = False
        self.silenced_until = 0
        self.last_time = 0
        self.last_text_time = 0

    def state(self, now):
        if not self.active:
            return STATE_NORMAL
        if self.acknowledged:
            return STATE_ACKNOWLEDGED
        if now < self.silenced_until:
            return STATE_SILENCED
        return STATE_ACTIVE


class N2KControl(object):
    def __init__(self, client, values, settings, now=time.monotonic):
        self.client = client      # pypilotClient, or anything with set/send/watch
        self.values = values      # last known pypilot values, shared with the bridge
        self.settings = settings  # registered n2k.control.* properties
        self.now = now
        self.gateway = None
        self.outbox = []

        # values we set that pypilot hasn't reported yet: name -> (value, time).
        # Actions build on them, so +1 +1 +1 adds 3 even before pypilot has reported
        # the first; status sent on the bus only shows what pypilot reports.
        self.requested = {}
        # status PGN -> time until which its next change is sent without waiting
        # STATUS_MIN_PERIOD: the result of a command, rather than a value flapping
        self.prompt_status = {}
        self.pending_engage = None
        self.jog_end = None
        self.follow_up = None     # (time, angle in rad) of the last follow-up command
        self.switches_on = {}     # channel -> time it was turned on
        self.momentary_until = {}
        self.pending_texts = []   # PP: reads and writes waiting for a value, oldest first
        self.text_tag = None      # tag of the PP# command being run
        self.leases = {}          # values watched for PP: reads -> expiry time
        self.description2 = 'PP:HELP'
        self.reject_until = 0
        self.last_127237 = (0, None)
        self.last_127501 = (0, None)

        self.alerts = {}
        for id, category, text in [
                (ALERT_SERVO_OVERCURRENT, CATEGORY_TECHNICAL, 'Autopilot servo overcurrent'),
                (ALERT_SERVO_OVERTEMP, CATEGORY_TECHNICAL, 'Autopilot servo over temperature'),
                (ALERT_BAD_VOLTAGE, CATEGORY_TECHNICAL, 'Autopilot servo bad voltage'),
                (ALERT_DRIVER, CATEGORY_TECHNICAL, 'Autopilot servo not responding'),
                (ALERT_RUDDER_LIMIT, CATEGORY_TECHNICAL, 'Autopilot rudder at limit'),
                (ALERT_IMU_ERROR, CATEGORY_TECHNICAL, 'Autopilot compass error'),
                (ALERT_COMPASS_WARNING, CATEGORY_TECHNICAL, 'Autopilot compass warning'),
                (ALERT_MODE_FALLBACK, CATEGORY_NAVIGATIONAL, 'Autopilot mode changed: sensor lost'),
                (ALERT_REJECTED, CATEGORY_TECHNICAL, 'Autopilot command rejected')]:
            self.alerts[id] = Alert(id, category, text)

    # ---- settings and access -------------------------------------------------

    def level(self):
        try:
            return CONTROL_LEVELS.index(self.settings['control'].value)
        except ValueError:
            return OFF

    def bank(self):
        try:  # set with a slider, so round to an instance; -1 is off
            bank = round(float(self.settings['bank'].value))
        except (TypeError, ValueError):
            return None
        if 0 <= bank <= 252:
            return bank
        return None

    def own_name(self):
        for attr in ['own_name', '_own_name']:
            name = getattr(self.gateway, attr, None)
            if isinstance(name, int):
                return name
        return 0

    def source_allowed(self, message):
        allowed = self.settings['allowed'].value
        if not allowed:
            return True
        iso_name = getattr(message, 'source_iso_name', None)
        name = getattr(iso_name, 'name', None)
        for entry in allowed:
            try:
                if isinstance(entry, str):
                    entry = int(entry, 0)
                if entry == name:
                    return True
            except ValueError:
                pass
        return False

    def permitted(self, needed, message):
        if self.level() < needed:
            return False
        # standby is always honoured, from any device, unless control is off
        return needed <= MONITOR or self.source_allowed(message)

    def include_pgns(self):
        if self.level() == OFF:
            return []
        return [126208, 127502, 126984, 59904]

    def transmit_pgns(self):
        pgns = []
        if self.settings['output'].value:
            pgns += [127237, 126983, 126985]
        if self.bank() is not None:
            pgns.append(127501)
        return pgns

    # ---- pypilot actions -----------------------------------------------------

    def set(self, name, value):
        self.client.set(name, value)
        now = self.now()
        self.requested[name] = (value, now)
        for pgn in (127237, 127501):
            self.prompt_status[pgn] = now + REQUEST_TIMEOUT

    def value(self, name, default=None):
        '''what a value will be: as we set it if pypilot hasn't reported that yet,
        otherwise as pypilot reports it. For actions; status uses self.values.'''
        if name in self.requested:
            value, t = self.requested[name]
            if self.now() - t < REQUEST_TIMEOUT:
                return value
            del self.requested[name]
        return self.values.get(name, default)

    def enabled(self):
        '''engaged, as pypilot reports'''
        return self.values.get('ap.enabled') is True

    def will_be_enabled(self):
        '''engaged, including an engage or standby pypilot hasn't reported yet'''
        return self.value('ap.enabled') is True

    def reject(self, reason):
        self.reject_until = self.now() + REJECT_ALERT_TIME
        alert = self.alerts[ALERT_REJECTED]
        alert.text = truncate('Autopilot rejected: ' + reason)
        if alert.active:  # already showing an earlier rejection: send the new text now
            alert.last_time = alert.last_text_time = 0
        return False

    def standby(self):
        self.pending_engage = None
        self.jog_end = None
        self.set('servo.command', 0)
        self.set('ap.enabled', False)
        return True

    def engage(self, mode=None, command=None):
        '''engage in mode (None keeps the current one), at command if given,
        otherwise holding the current heading'''
        current = self.value('ap.mode')
        if mode is None:
            mode = current
        if mode not in (self.value('ap.modes') or []):
            return self.reject('%s mode not available' % mode)

        if mode != current:
            self.set('ap.mode', mode)
            if self.will_be_enabled() and command is None:
                return True  # pypilot keeps the same course across the mode change
            # ap.heading is still in the old mode's frame, and pypilot resets
            # the command when it sees the new mode, so wait for that before
            # setting the command
            self.pending_engage = {'mode': mode, 'command': command, 'time': self.now()}
            return True

        if command is None and not self.will_be_enabled():
            command = self.value('ap.heading')
        if command is not None:
            self.set('ap.heading_command', command)
        self.set('ap.enabled', True)
        return True

    def adjust(self, degrees):
        '''change the heading command, +ve to starboard'''
        if not self.will_be_enabled():
            return self.reject('not engaged')
        if self.value('ap.tack.state', 'none') != 'none':
            return self.reject('tacking')
        command = self.value('ap.heading_command')
        if not is_number(command):
            return self.reject('no heading command')
        if 'wind' in str(self.value('ap.mode')):
            degrees = -degrees  # wind angle decreases turning to starboard
        self.set('ap.heading_command', command + degrees)
        return True

    def tack(self, direction):
        '''tack to direction, None for the direction pypilot detected'''
        if not self.will_be_enabled():
            return self.reject('not engaged')
        state = self.value('ap.tack.state', 'none')
        if state in ['begin', 'waiting']:
            self.set('ap.tack.state', 'none')  # pressing again cancels
            return True
        if state == 'tacking':
            return self.reject('already tacking')
        if direction:
            self.set('ap.tack.direction', direction)
        elif self.value('ap.tack.direction', 'none') == 'none':
            return self.reject('tack direction unknown')
        self.set('ap.tack.state', 'begin')
        return True

    def jog(self, direction):
        '''one short pulse of the servo, +1 starboard, -1 port'''
        if self.will_be_enabled():
            return self.reject('engaged')
        self.set('servo.command', -JOG_SPEED * direction)  # servo.command is -ve to starboard
        self.jog_end = self.now() + self.settings['jog_pulse'].value
        return True

    def stop_jog(self):
        self.jog_end = None
        self.set('servo.command', 0)

    def next_mode(self):
        '''select the next available mode without engaging, like the hat's mode key'''
        modes = self.value('ap.modes') or []
        mode = self.value('ap.mode')
        if len(modes) < 2:
            return self.reject('no other mode available')
        i = modes.index(mode) + 1 if mode in modes else 0
        self.set('ap.mode', modes[i % len(modes)])
        return True

    def next_profile(self):
        profiles = self.value('profiles') or []
        if not profiles:
            return self.reject('no profiles')
        profile = self.value('profile')
        i = profiles.index(profile) + 1 if profile in profiles else 0
        self.set('profile', profiles[i % len(profiles)])
        return True

    # ---- path A: 126208 command -> 127237 -------------------------------------

    def command_127237(self, params, message):
        values = dict(params)
        errors = {index: PARAM_ACK for index in values}
        for index, value in values.items():
            if index not in [5, 7, 9, 10, 11]:
                errors[index] = PARAM_NOT_SUPPORTED
            elif index in COMMAND_LOOKUPS and value is None:
                errors[index] = PARAM_OUT_OF_RANGE  # not available, or out of the lookup's range

        steering = values.get(5)
        reference = values.get(7)
        direction = values.get(9)
        rudder = values.get(10)
        hts = values.get(11)

        if steering == STEERING_MAIN and len(values) == 1:
            needed = MONITOR  # bare standby
        else:
            needed = STEER
        if not self.permitted(needed, message):
            return PGN_ACCESS_DENIED, [PARAM_ACCESS_DENIED] * len(params)

        mode = self.value('ap.mode')
        if steering in [STEERING_STANDALONE, STEERING_HEADING]:
            if reference == REFERENCE_MAGNETIC:
                mode = 'compass'
            elif reference == REFERENCE_TRUE:
                # no true heading mode; gps mode steers course over ground, which differs
                errors[7] = PARAM_NOT_SUPPORTED
            elif mode == 'nav':
                mode = 'compass'
        elif steering == STEERING_TRACK:
            mode = 'nav'

        if steering in [STEERING_STANDALONE, STEERING_HEADING, STEERING_TRACK] and \
           mode not in (self.value('ap.modes') or []):
            errors[5] = PARAM_TEMPORARY_ERROR
        if reference is not None and reference not in [REFERENCE_TRUE, REFERENCE_MAGNETIC]:
            errors[7] = PARAM_OUT_OF_RANGE
        if hts is not None and 'wind' in str(mode):
            errors[11] = PARAM_TEMPORARY_ERROR  # never reinterpret a heading as a wind angle
        if steering == STEERING_NFU and direction is None:
            errors[5] = PARAM_INVALID
        if direction is not None and (steering != STEERING_NFU or direction > RUDDER_PORT):
            errors[9] = PARAM_INVALID
        if steering == STEERING_FU and (rudder is None or not self.rudder_present()):
            errors[5] = PARAM_TEMPORARY_ERROR
        if rudder is not None and steering != STEERING_FU:
            errors[10] = PARAM_INVALID

        param_errors = [errors[index] for index, value in params]
        if any(param_errors):
            return PGN_ACK, param_errors

        command = None if hts is None else rad2deg(hts)
        if steering == STEERING_MAIN:
            self.standby()
        elif steering in [STEERING_STANDALONE, STEERING_HEADING, STEERING_TRACK]:
            self.engage(mode, command)
        elif steering == STEERING_NFU:
            if self.will_be_enabled():
                self.standby()
            if direction == RUDDER_NO_ORDER:
                self.stop_jog()
            else:
                self.jog(1 if direction == RUDDER_STARBOARD else -1)
        elif steering == STEERING_FU:
            if self.will_be_enabled():
                self.standby()
            self.follow_up = (self.now(), rudder)
            self.set('servo.position_command', -rad2deg(rudder))  # pypilot rudder is +ve to port
        elif command is not None:
            self.set('ap.heading_command', command)
        self.send_127237()
        return PGN_ACK, param_errors

    # ---- path C: 'PP:' text commands in 126998 --------------------------------

    def command_126998(self, params, message):
        errors = []
        for index, text in params:
            if index != 1:
                errors.append(PARAM_ACCESS_DENIED)  # description 2 is our output
            elif not isinstance(text, str):
                errors.append(PARAM_INVALID)  # not available
            elif text.startswith('PP:') or text.startswith('PP#'):
                if not self.permitted(MONITOR, message):
                    errors.append(PARAM_ACCESS_DENIED)
                    continue
                errors.append(PARAM_ACK)
                split = split_text_command(text)
                if split:
                    self.text_command(split[1], message, split[0])
                else:
                    self.text_result('ERR tag must be 1-8 letters or digits')
            else:
                errors.append(PARAM_ACK)
                self.settings['description'].set(text)
                self.send_126998()
        return PGN_ACK, errors

    def text_result(self, text, tag=None):
        tag = self.text_tag if tag is None else tag
        self.description2 = truncate(('#%s ' % tag if tag else '') + text)
        self.send_126998()

    def value_list(self):
        '''info for every pypilot value, by name. pypilotClient keeps the 'values'
        list to itself (client.values) rather than returning it from receive()'''
        info = getattr(getattr(self.client, 'values', None), 'value', None)
        if not isinstance(info, dict):
            info = self.values.get('values')
        return info if isinstance(info, dict) else None

    def value_info(self, name):
        info = self.value_list()
        return info.get(name) if info else None

    def text_command(self, command, message, tag=None):
        self.text_tag = tag
        try:
            self.run_text_command(command, message, tag)
        finally:
            self.text_tag = None

    def run_text_command(self, command, message, tag):
        word = command.split(' ', 1)[0].upper()
        if not command or word == 'HELP':
            return self.text_result('PP:name[@offset] | PP:name=json | PP:INFO name | PP:LIST prefix [page]')

        if word == 'INFO':
            name = command[5:].strip()
            info = self.value_info(name)
            if info is None:
                return self.text_result('ERR unknown ' + name)
            desc = [info.get('type', 'Value')]
            if 'min' in info:
                desc.append('%s..%s' % (info['min'], info['max']))
            if 'choices' in info:
                desc.append('|'.join(map(str, info['choices'])))
            for flag in ['writable', 'persistent', 'profiled']:
                if info.get(flag):
                    desc.append(flag)
            return self.text_result(' '.join(desc))

        if word == 'LIST':
            args = command.split()[1:]
            prefix = args[0] if args else ''
            page = int(args[1]) if len(args) > 1 and args[1].isdigit() else 0
            info = self.value_list()
            if not info:
                return self.text_result('ERR value list not loaded')
            # the prefix is shown once, followed by the rest of each name
            names = sorted(n[len(prefix):] for n in info if n.startswith(prefix))
            return self.text_result(self.list_page(prefix, names, page))

        if '=' in command:
            name, value = command.split('=', 1)
            name, value = name.strip(), value.strip()
            if not name or not all(c.isalnum() or c in '._' for c in name):
                return self.text_result('ERR bad name')
            if not self.permitted(FULL, message):
                return self.text_result('ERR n2k.control must be full to write')
            if denied_write(name):
                return self.text_result('ERR %s not writable over n2k' % name)
            info = self.value_info(name)
            if info is not None and not info.get('writable'):
                return self.text_result('ERR %s is read only' % name)
            try:
                expected = json.loads(value)
            except ValueError:
                return self.text_result('ERR value must be json')
            self.client.send(name + '=' + json.dumps(expected) + '\n')
            # report what pypilot actually applied, which may be clamped or rounded
            return self.request_value(name, tag, write=expected)

        name, offset = command, None
        if '@' in command:
            name, _, offset = command.partition('@')
        name = name.strip()
        if not name or not all(c.isalnum() or c in '._' for c in name):
            return self.text_result('ERR bad name')
        if offset is not None:
            if not offset.isdigit():
                return self.text_result('ERR offset must be a number')
            offset = int(offset)
        return self.request_value(name, tag, offset=offset)

    def list_page(self, prefix, names, page):
        # pack as many names as fit, each page continuing where the last stopped
        header = prefix + ': ' if prefix else ''
        pages, current = [], []
        for name in names:
            if current and len(header + ' '.join(current + [name])) > TEXT_LENGTH - 6:
                pages.append(current)
                current = []
            current.append(name)
        if current:
            pages.append(current)
        if page >= len(pages):
            return 'ERR no page %d' % page if pages else 'ERR no match'
        text = header + ' '.join(pages[page])
        if page + 1 < len(pages):
            text += ' (+%d)' % (len(pages) - page - 1)
        return text

    def lease(self, name, now):
        '''Keep a value read with PP: watched for a while, so repeat reads (a control
        head polling a few values) are answered at once from a live value.'''
        watches = getattr(self.client, 'watches', {})
        if name not in self.leases and name in watches:
            return  # the bridge watches it anyway
        if name not in self.leases:
            if len(self.leases) >= MAX_TEXT_LEASES:
                self.end_lease(min(self.leases, key=self.leases.get))
            self.values.pop(name, None)  # stale until the watch delivers
            self.client.watch(name)
        self.leases[name] = now + TEXT_LEASE

    def end_lease(self, name):
        del self.leases[name]
        self.client.watch(name, False)
        self.values.pop(name, None)  # no longer kept up to date

    def own_value(self, name):
        '''(True, value) for a value this process registered itself, such as the
        n2k.* settings: its client keeps those and never reports them as received'''
        if name == 'values':  # the info for every value, as pypilot_web's catalog
            info = self.value_list()
            return info is not None, info
        own = getattr(getattr(self.client, 'values', None), 'values', None)
        if isinstance(own, dict) and name in own and name != 'watch':
            return True, own[name].value
        return False, None

    def current_value(self, name):
        own, value = self.own_value(name)
        if own:
            return True, value
        if name in self.values:
            return True, self.values[name]
        return False, None

    def request_value(self, name, tag, write=None, offset=None):
        now = self.now()
        own, value = self.own_value(name)
        if own:
            live = True
        else:
            live = name in self.values and (name in self.leases or
                                            name in getattr(self.client, 'watches', {}))
            value = self.values.get(name)
            self.lease(name, now)
        pending = {'name': name, 'tag': tag, 'write': write, 'offset': offset, 'start': now}
        if live and (write is None or values_match(value, write)):
            self.finish_text(pending)
        elif len(self.pending_texts) >= MAX_PENDING_TEXT:
            self.text_result('ERR busy', tag)
        else:
            # several control heads can be waiting at once; each gets its own answer
            self.pending_texts.append(pending)

    def finish_text(self, pending):
        if pending in self.pending_texts:
            self.pending_texts.remove(pending)
        name, tag, offset = pending['name'], pending['tag'], pending['offset']
        value = self.current_value(name)[1]
        if offset is None:
            prefix = 'OK ' if pending['write'] is not None else ''
            return self.text_result('%s%s=%s' % (prefix, name, json.dumps(value)), tag)
        # a slice of a long value: name@offset/total=chunk, compact json
        text = json.dumps(value, separators=(',', ':'))
        if offset > len(text):
            return self.text_result('ERR offset past end of %s (%d)' % (name, len(text)), tag)
        header = '%s@%d/%d=' % (name, offset, len(text))
        room = TEXT_LENGTH - len(header) - (len(tag) + 2 if tag else 0)
        self.text_result(header + text[offset:offset + max(room, 0)], tag)

    def poll_text(self, now):
        waiting = {pending['name'] for pending in self.pending_texts}
        for name, expiry in list(self.leases.items()):
            if now > expiry and name not in waiting:
                self.end_lease(name)
        for pending in list(self.pending_texts):
            elapsed = now - pending['start']
            if pending['write'] is not None and elapsed >= WRITE_SETTLE and \
                    self.current_value(pending['name'])[0]:
                self.finish_text(pending)  # no update matching the request: report what it is now
            elif elapsed >= TEXT_TIMEOUT:
                self.pending_texts.remove(pending)
                self.text_result('ERR unknown ' + pending['name'], pending['tag'])

    # ---- path B: switch bank --------------------------------------------------

    def switch_bank_control(self, message):
        bank = self.bank()
        if bank is None:
            return
        try:
            if message.get_field_by_id('instance').value != bank:
                return
        except ValueError:
            return
        now = self.now()
        for channel in range(1, 29):
            try:
                state = message.get_field_by_id('switch%d' % channel).raw_value
            except ValueError:
                continue
            if state == SWITCH_ON:
                if channel not in self.switches_on:  # act on the edge only
                    self.switches_on[channel] = now
                    self.press(channel, message)
            elif state == SWITCH_OFF:
                self.switches_on.pop(channel, None)
        self.send_127501()

    def press(self, channel, message):
        if channel == CH_STANDBY:
            needed = MONITOR
        elif channel == CH_DISMISS:
            needed = MONITOR
        elif channel in CH_MODES or channel in CH_ADJUST or channel in CH_TACK or \
             channel in CH_JOG or channel in [CH_NEXT_PROFILE, CH_NEXT_MODE]:
            needed = STEER
        else:
            return  # reserved or status only channel
        if not self.permitted(needed, message):
            self.reject('n2k control level is %s' % CONTROL_LEVELS[self.level()])
            return

        if channel == CH_STANDBY:
            done = self.standby()
        elif channel in CH_MODES:
            done = self.engage(CH_MODES[channel])
        elif channel in CH_ADJUST:
            done = self.adjust(CH_ADJUST[channel])
        elif channel in CH_TACK:
            done = self.tack(CH_TACK[channel])
        elif channel in CH_JOG:
            done = self.jog(CH_JOG[channel])
        elif channel == CH_DISMISS:
            done = self.acknowledge_all()
        elif channel == CH_NEXT_MODE:
            done = self.next_mode()
        else:
            done = self.next_profile()

        if done and channel in MOMENTARY_CHANNELS:
            self.momentary_until[channel] = self.now() + MOMENTARY_TIME

    def indicators(self, now):
        values = self.values
        enabled = self.enabled()
        mode = values.get('ap.mode')
        state = [SWITCH_NA] * 29  # index 0 unused

        def on(condition):
            return SWITCH_ON if condition else SWITCH_OFF

        state[CH_STANDBY] = on(not enabled)
        for channel, channel_mode in CH_MODES.items():
            state[channel] = on(enabled and mode == channel_mode)
        for channel in MOMENTARY_CHANNELS:
            state[channel] = on(self.momentary_until.get(channel, 0) > now)
        tacking = values.get('ap.tack.state', 'none') != 'none'
        for channel in CH_TACK:
            state[channel] = on(tacking)
        state[CH_DISMISS] = on(any(a.active and not a.acknowledged for a in self.alerts.values()))
        state[CH_SERVO_FAULT] = on(bool(self.servo_flags() & SERVO_FAULTS))
        state[CH_IMU_ERROR] = on(bool(values.get('imu.error')))
        state[CH_MODE_FALLBACK] = on(self.mode_fallback())
        return state[1:]

    def send_127501(self):
        bank = self.bank()
        if bank is None:
            return
        now = self.now()
        state = self.indicators(now)
        fields = {'instance': bank}
        for i, s in enumerate(state):
            fields['indicator%d' % (i + 1)] = Lookup(s)
        self.outbox.append(make_message(127501, fields, priority=3))
        self.last_127501 = (now, state)

    # ---- status ---------------------------------------------------------------

    def servo_flags(self):
        flags = self.values.get('servo.flags')
        return set(flags.split()) if isinstance(flags, str) else set()

    def rudder_present(self):
        return self.values.get('rudder.source', 'none') != 'none' and \
            is_number(self.values.get('rudder.angle'))

    def mode_fallback(self):
        preferred = self.values.get('ap.preferred_mode')
        return self.enabled() and preferred in MODES and preferred != self.values.get('ap.mode')

    def manual_command(self):
        command = self.values.get('servo.command')
        if self.enabled() or not is_number(command):
            return 0
        return command

    def heading_track_control(self):
        values = self.values
        enabled = self.enabled()
        mode = values.get('ap.mode')
        flags = self.servo_flags()

        if not enabled:
            steering = STEERING_MAIN
        elif mode == 'nav':
            steering = STEERING_TRACK
        else:
            steering = STEERING_HEADING
        reference = REFERENCE_MAGNETIC if mode in ['compass', 'wind', 'true wind'] else REFERENCE_TRUE

        command = values.get('ap.heading_command')
        if 'wind' in str(mode):
            # the compass heading equivalent to the wind angle being steered
            # (ap.heading_error is clipped to +-30, so this is approximate in big turns)
            compass, error = values.get('imu.heading_lowpass'), values.get('ap.heading_error')
            command = compass - error if is_number(compass) and is_number(error) else None

        manual = self.manual_command()
        if manual < 0:
            direction = RUDDER_STARBOARD
        elif manual > 0:
            direction = RUDDER_PORT
        else:
            direction = RUDDER_NO_ORDER

        rudder_angle = None
        if self.follow_up and self.now() - self.follow_up[0] < 5:
            rudder_angle = self.follow_up[1]

        rudder_range = values.get('rudder.range')
        return make_message(127237, {
            'rudderLimitExceeded': Lookup(YES if flags & {'MIN_RUDDER_FAULT', 'MAX_RUDDER_FAULT'} else NO),
            'offHeadingLimitExceeded': Lookup(3),
            'offTrackLimitExceeded': Lookup(3),
            'override': Lookup(YES if manual or self.jog_end else NO),
            'steeringMode': Lookup(steering),
            'turnMode': Lookup(7),
            'headingReference': Lookup(reference),
            'reserved_16': None,
            'commandedRudderDirection': Lookup(direction),
            'commandedRudderAngle': rudder_angle,
            'headingToSteerCourse': heading_rad(command) if enabled else None,
            'track': heading_rad(values.get('apb.track')) if enabled and mode == 'nav' else None,
            'rudderLimit': deg2rad(rudder_range) if self.rudder_present() and is_number(rudder_range) else None,
            'offHeadingLimit': None,
            'radiusOfTurnOrder': None,
            'rateOfTurnOrder': None,
            'offTrackLimit': None,
            'vesselHeading': heading_rad(values.get('imu.heading_lowpass')),
        }, priority=2)

    def status_key(self):
        values = self.values
        return (values.get('ap.enabled'), values.get('ap.mode'),
                values.get('ap.heading_command'), bool(self.manual_command()))

    def send_127237(self):
        if not self.settings['output'].value:
            return
        self.outbox.append(self.heading_track_control())
        self.last_127237 = (self.now(), self.status_key())

    def rudder(self):
        '''127245, None when we have no rudder angle or it came from N2K anyway'''
        angle = self.values.get('rudder.angle')
        source = str(self.values.get('rudder.source', 'none'))
        if not is_number(angle) or source == 'none' or source.startswith('N2K') or source == 'can':
            return None
        manual = self.manual_command()
        direction = RUDDER_STARBOARD if manual < 0 else RUDDER_PORT if manual > 0 else RUDDER_NO_ORDER
        return make_message(127245, {'instance': 0,
                                     'directionOrder': Lookup(direction),
                                     'reserved_11': None,
                                     'angleOrder': None,
                                     'position': -deg2rad(angle),  # pypilot rudder is +ve to port
                                     'reserved_48': None}, priority=2)

    def send_126998(self):
        description = self.settings['description'].value or ''
        manufacturer = getattr(self.gateway, 'manufacturer_information', None) or 'PyPilot Autopilot'
        if self.gateway is not None:
            # so the library's own answers to ISO requests stay consistent
            self.gateway.installation_description1 = description
            self.gateway.installation_description2 = self.description2
        self.outbox.append(make_message(126998, {
            'installationDescription1': description,
            'installationDescription2': self.description2,
            'manufacturerInformation': manufacturer}))

    # ---- alerts ---------------------------------------------------------------

    def alert_conditions(self, now):
        values = self.values
        flags = self.servo_flags()
        enabled = self.enabled()
        imu_error = values.get('imu.error')
        imu_warning = values.get('imu.warning')
        conditions = {
            ALERT_SERVO_OVERCURRENT: bool(flags & {'OVERCURRENT_FAULT', 'PORT_OVERCURRENT_FAULT',
                                                  'STARBOARD_OVERCURRENT_FAULT'}),
            ALERT_SERVO_OVERTEMP: 'OVERTEMP_FAULT' in flags,
            ALERT_BAD_VOLTAGE: 'BADVOLTAGE_FAULT' in flags,
            ALERT_DRIVER: enabled and ('DRIVER_TIMEOUT' in flags or
                                       values.get('servo.controller') == 'none'),
            ALERT_RUDDER_LIMIT: bool(flags & {'MIN_RUDDER_FAULT', 'MAX_RUDDER_FAULT'}),
            ALERT_IMU_ERROR: bool(imu_error),
            ALERT_COMPASS_WARNING: bool(imu_warning),
            ALERT_MODE_FALLBACK: self.mode_fallback(),
            ALERT_REJECTED: now < self.reject_until,
        }
        types = {ALERT_SERVO_OVERCURRENT: ALERT_ALARM, ALERT_SERVO_OVERTEMP: ALERT_WARNING,
                 ALERT_BAD_VOLTAGE: ALERT_WARNING, ALERT_DRIVER: ALERT_ALARM,
                 ALERT_RUDDER_LIMIT: ALERT_CAUTION,
                 ALERT_IMU_ERROR: ALERT_ALARM if enabled else ALERT_WARNING,
                 ALERT_COMPASS_WARNING: ALERT_CAUTION, ALERT_MODE_FALLBACK: ALERT_WARNING,
                 ALERT_REJECTED: ALERT_CAUTION}
        if imu_error:
            self.alerts[ALERT_IMU_ERROR].text = truncate('Autopilot compass: %s' % imu_error)
        if imu_warning:
            self.alerts[ALERT_COMPASS_WARNING].text = truncate('Autopilot compass: %s' % imu_warning)
        if conditions[ALERT_MODE_FALLBACK]:
            self.alerts[ALERT_MODE_FALLBACK].text = truncate(
                'Autopilot mode %s, %s unavailable' % (values.get('ap.mode'), values.get('ap.preferred_mode')))
        return conditions, types

    def alert_messages(self, alert, now, text):
        common = {'alertType': Lookup(alert.type),
                  'alertCategory': Lookup(alert.category),
                  'alertSystem': ALERT_SYSTEM,
                  'alertSubSystem': 0,
                  'alertId': alert.id,
                  'dataSourceNetworkIdName': self.own_name(),
                  'dataSourceInstance': 0,
                  'dataSourceIndexSource': 0,
                  'alertOccurrenceNumber': alert.occurrence % 256}
        alert_fields = dict(common)
        alert_fields.update({'temporarySilenceStatus': Lookup(YES if alert.state(now) == STATE_SILENCED else NO),
                             'acknowledgeStatus': Lookup(YES if alert.acknowledged else NO),
                             'escalationStatus': Lookup(NO),
                             'temporarySilenceSupport': Lookup(YES),
                             'acknowledgeSupport': Lookup(YES),
                             'escalationSupport': Lookup(NO),
                             'reserved_134': None,
                             'acknowledgeSourceNetworkIdName': 0,
                             'triggerCondition': Lookup(1),  # auto
                             'thresholdStatus': Lookup(1 if alert.active else 0),
                             'alertPriority': 0,
                             'alertState': Lookup(alert.state(now))})
        messages = [make_message(126983, alert_fields, priority=2)]
        if text:
            text_fields = dict(common)
            text_fields.update({'languageId': Lookup(0),
                                'alertTextDescription': alert.text,
                                'alertLocationTextDescription': ''})
            messages.append(make_message(126985, text_fields))
        return messages

    def poll_alerts(self, now):
        if not self.settings['output'].value:
            return
        conditions, types = self.alert_conditions(now)
        for id, active in conditions.items():
            alert = self.alerts[id]
            alert.type = types[id]
            if active and not alert.active:
                alert.active = True
                alert.occurrence += 1
                alert.acknowledged = False
                alert.silenced_until = 0
                self.outbox += self.alert_messages(alert, now, True)
                alert.last_time = alert.last_text_time = now
            elif not active and alert.active:
                alert.active = False
                self.outbox += self.alert_messages(alert, now, False)
            elif active and now - alert.last_time >= STATUS_PERIOD:
                text = now - alert.last_text_time >= ALERT_TEXT_PERIOD
                self.outbox += self.alert_messages(alert, now, text)
                alert.last_time = now
                if text:
                    alert.last_text_time = now

    def acknowledge_all(self):
        for alert in self.alerts.values():
            if alert.active:
                alert.acknowledged = True
                alert.last_time = 0  # report the new state promptly
        return True

    def alert_response(self, message):
        try:
            if message.get_field_by_id('alertSystem').value != ALERT_SYSTEM:
                return
            alert = self.alerts.get(message.get_field_by_id('alertId').value)
            source = message.get_field_by_id('dataSourceNetworkIdName').raw_value
            occurrence = message.get_field_by_id('alertOccurrenceNumber').value
            response = message.get_field_by_id('responseCommand').raw_value
        except ValueError:
            return
        own = self.own_name()
        if not alert or not alert.active or (own and source != own) or \
           occurrence != alert.occurrence % 256:
            return
        if not self.permitted(MONITOR, message):
            return
        if response == RESPONSE_ACKNOWLEDGE:
            alert.acknowledged = True
        elif response == RESPONSE_SILENCE:
            alert.silenced_until = self.now() + SILENCE_TIME
        alert.last_time = 0

    # ---- bridge interface -----------------------------------------------------

    def attach(self, gateway):
        self.gateway = gateway
        if gateway is not None:
            gateway.installation_description1 = self.settings['description'].value or ''
            gateway.installation_description2 = self.description2

    def on_value(self, name, value):
        '''called by the bridge for every value received from pypilot'''
        self.values[name] = value
        if name in self.requested and requested_match(value, self.requested[name][0]):
            del self.requested[name]
        pending = self.pending_engage
        if pending:
            if name == 'ap.mode' and value == pending['mode']:
                pending.setdefault('mode_time', self.now())
            elif name == 'ap.heading' and 'mode_time' in pending and \
                    self.now() - pending['mode_time'] >= ENGAGE_SETTLE:
                # the server echoes the mode before the autopilot has computed a heading
                # in it, so only take headings from a while after the change
                self.pending_engage = None
                command = pending['command']
                self.set('ap.heading_command', value if command is None else command)
                self.set('ap.enabled', True)
        for pending in list(self.pending_texts):
            if name == pending['name'] and \
                    (pending['write'] is None or values_match(value, pending['write'])):
                self.finish_text(pending)

    def handle_message(self, message):
        '''called by the bridge for received PGNs, True if it was ours'''
        if self.level() == OFF:
            return False
        if message.PGN == 127502:
            self.switch_bank_control(message)
            return True
        if message.PGN == 126984:
            self.alert_response(message)
            return True
        return False

    def handle_group_function(self, message):
        '''126208 handler for the library, given the decoded message. Returns
        True if we answered it, False to let the library answer (it NAKs).'''
        if self.level() == OFF:
            return False
        pgn = field(message, 'pgn')
        if message.id == 'nmeaRequestGroupFunction':  # answer it if it's for something we send
            return pgn is not None and self.handle_iso_request(message, pgn)
        if message.id != 'nmeaCommandGroupFunction' or pgn not in COMMAND_PGNS:
            return False
        params = command_parameters(message)
        if pgn == 127237:
            pgn_error, errors = self.command_127237(params, message)
        else:
            pgn_error, errors = self.command_126998(params, message)
        self.outbox.append(build_ack(message.source, pgn, pgn_error, errors))
        return True

    def handle_iso_request(self, message, pgn):
        '''ISO request (59904) handler for the library, True if answered'''
        if self.level() == OFF:
            return False
        if pgn == 127237 and self.settings['output'].value:
            self.send_127237()
            return True
        if pgn == 127501 and self.bank() is not None:
            self.send_127501()
            return True
        if pgn == 126998:
            self.send_126998()
            return True
        return False

    def status_due(self, pgn, last, now):
        '''whether a changed status can be sent now'''
        if self.prompt_status.get(pgn, 0) > now:
            del self.prompt_status[pgn]  # once: further changes are rate limited
            return True
        return now - last >= STATUS_MIN_PERIOD

    def poll(self):
        '''periodic work; returns the messages to send'''
        now = self.now()
        if self.jog_end and now >= self.jog_end:
            self.stop_jog()
        if self.pending_engage and now - self.pending_engage['time'] > ENGAGE_TIMEOUT:
            mode = self.pending_engage['mode']
            self.pending_engage = None
            self.reject('engage in %s mode timed out' % mode)
        for channel, t in list(self.switches_on.items()):
            if now - t > SWITCH_REARM_TIME:
                del self.switches_on[channel]
        self.poll_text(now)

        if self.settings['output'].value:
            t, key = self.last_127237
            if now - t >= STATUS_PERIOD or (key != self.status_key() and self.status_due(127237, t, now)):
                self.send_127237()
        if self.bank() is not None:
            t, state = self.last_127501
            if now - t >= STATUS_PERIOD or (state != self.indicators(now) and self.status_due(127501, t, now)):
                self.send_127501()
        self.poll_alerts(now)

        outbox, self.outbox = self.outbox, []
        return outbox
