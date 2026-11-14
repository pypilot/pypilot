"""
Smoke test of N2KBridge's wiring of N2KControl: settings registration,
PGN filters, library hooks and sending, without a CAN bus or pypilot server.
"""
import asyncio
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

nmea2000 = pytest.importorskip('nmea2000')

import n2k
import n2k_control as nc
from test_n2k_control import command, command_payload, encode, switch_message, Source


class FakeClient:
    def __init__(self):
        self.registered = {}
        self.sets = []
        self.watches = {}
        self.received = {}
        self.received_data = True

    def register(self, value):
        self.registered[value.name] = value
        value.client = self
        return value

    def set(self, name, value):
        self.sets.append((name, value))

    def send(self, msg):
        pass

    def watch(self, name, value=True):
        self.watches[name] = value

    def receive(self):
        received, self.received = self.received, {}
        return received


class FakeGateway:
    ready = True
    own_name = 0x1234

    def __init__(self):
        self.sent = []
        self.group_function_handler = None
        self.iso_request_handler = None

    def set_group_function_handler(self, handler):
        self.group_function_handler = handler

    def set_iso_request_handler(self, handler):
        self.iso_request_handler = handler

    async def send(self, message):
        encode(message)  # everything the bridge sends must be encodable
        self.sent.append(message)


def make_bridge(gateway):
    bridge = n2k.N2KBridge.__new__(n2k.N2KBridge)
    bridge.client = FakeClient()
    asyncio.run(bridge.setup())  # transport defaults to none, so no gateway yet
    bridge.gateway = gateway
    bridge.attach_control()
    return bridge


def test_n2k_disabled_without_multiprocessing(monkeypatch, capsys):
    server = SimpleNamespace(multiprocessing=False)
    sensors = SimpleNamespace(client=SimpleNamespace(server=server))
    bridge = Mock(side_effect=AssertionError('must not construct a bridge'))
    monkeypatch.setattr(n2k, 'N2KBridge', bridge)
    instance = n2k.N2K(sensors)
    assert instance.n2k_bridge is None
    instance.poll()
    bridge.assert_not_called()
    assert 'N2K: disabled, server multiprocessing is off' in capsys.readouterr().out


def test_n2k_process_pipe_polling(monkeypatch):
    server = SimpleNamespace(multiprocessing=True)
    sensors = SimpleNamespace(client=SimpleNamespace(server=server), write=Mock())
    bridge = Mock()
    bridge.pipe_out.fileno.return_value = 42
    bridge.pipe_out.recv.side_effect = [{'wind': {'speed': 5}}, False]
    monkeypatch.setattr(n2k, 'N2KBridge', Mock(return_value=bridge))
    poller = Mock()
    poller.poll.side_effect = [[(42, n2k.select.POLLIN)], []]
    monkeypatch.setattr(n2k.select, 'poll', Mock(return_value=poller))
    instance = n2k.N2K(sensors)
    instance.poll()
    sensors.write.assert_called_once_with('wind', {'speed': 5}, 'can')
    bridge.poll.assert_not_called()


def test_settings_watches_and_filters():
    bridge = make_bridge(FakeGateway())
    registered = bridge.client.registered
    for name in ['n2k.control', 'n2k.switch.bank', 'n2k.control.allowed',
                 'n2k.control.jog_pulse', 'n2k.installation_description']:
        assert name in registered
    assert registered['n2k.control'].value == 'monitor'
    for name in nc.WATCHES:
        assert bridge.client.watches[name] is True
    assert {126208, 127502, 126984, 59904} <= set(bridge.transport_include_pgns())
    assert 127237 in bridge.get_transmit_pgns()
    assert 127501 not in bridge.get_transmit_pgns()  # no switch bank until configured

    registered['n2k.control'].value = 'off'
    assert 126208 not in bridge.transport_include_pgns()


def test_device_advertises_transmit_and_receive_pgns(monkeypatch):
    options = {}

    class StartingGateway(FakeGateway):
        def set_receive_callback(self, callback):
            pass

        def set_status_callback(self, callback):
            pass

        async def start(self):
            pass

    def for_python_can(**kwargs):
        options.update(kwargs)
        return StartingGateway()

    monkeypatch.setattr(nmea2000.N2KDevice, 'for_python_can', for_python_can)
    bridge = make_bridge(FakeGateway())
    bridge.gateway = None
    bridge.n2k_transport.value = 'socketcan'
    asyncio.run(bridge.init_transport())

    assert options['transmit_pgns'] == bridge.get_transmit_pgns()
    assert options['receive_pgns'] == bridge.transport_include_pgns()
    assert {130306, 129029, 126208, 127502, 126984} <= set(options['receive_pgns'])


def test_hooks_answer_commands():
    gateway = FakeGateway()
    bridge = make_bridge(gateway)
    bridge.client.registered['n2k.control'].value = 'steer'
    bridge.last_values.update({'ap.modes': ['compass'], 'ap.heading': 10.0})
    handled = asyncio.run(gateway.group_function_handler(
        command(command_payload(127237, [(5, bytes([4]))]))))
    assert handled
    assert ('ap.enabled', True) in bridge.client.sets
    assert [m.PGN for m in gateway.sent if m.PGN == 126208] == [126208]
    assert asyncio.run(gateway.iso_request_handler(Source(), 127237))
    assert not asyncio.run(gateway.iso_request_handler(Source(), 130306))


def test_switch_message_and_periodic_output():
    gateway = FakeGateway()
    bridge = make_bridge(gateway)
    bridge.client.registered['n2k.control'].value = 'steer'
    bridge.client.registered['n2k.switch.bank'].value = 3
    # a new bank changes the advertised PGNs, which normally restarts the transport
    assert bridge.current_transport_config() != bridge.transport_config
    bridge.transport_config = bridge.current_transport_config()
    bridge.last_values.update({'ap.enabled': True})
    asyncio.run(bridge.parse_pgn(switch_message(instance=3, ch1=nc.SWITCH_ON)))
    assert ('ap.enabled', False) in bridge.client.sets

    bridge.client.received = {'ap.mode': 'compass'}
    asyncio.run(bridge.poll(0))
    pgns = {m.PGN for m in gateway.sent}
    assert {127237, 127501} <= pgns


def test_missing_library_hooks_are_reported(capsys):
    class OldGateway(FakeGateway):
        set_group_function_handler = None
        set_iso_request_handler = None
    make_bridge(OldGateway())
    assert 'nmea2000 library has no' in capsys.readouterr().out


def test_gps_timestamp_is_epoch_seconds():
    import datetime
    message = nmea2000.NMEA2000Message(PGN=129033, fields=[
        nmea2000.NMEA2000Field(id='date', value=datetime.date(2026, 10, 2)),
        nmea2000.NMEA2000Field(id='time', value=datetime.time(16, 26, 3, 500000))])
    assert n2k.gps_timestamp(message) == 1790958363.5
    assert n2k.gps_timestamp(nmea2000.NMEA2000Message(PGN=129033, fields=[])) is None


def startup_bridge(monkeypatch):
    clock = [1000.0]
    monkeypatch.setattr(n2k.time, 'monotonic', lambda: clock[0])
    bridge = n2k.N2KBridge.__new__(n2k.N2KBridge)
    bridge.client = FakeClient()
    bridge.client.received_data = False
    asyncio.run(bridge.setup())
    starts = []

    async def init_transport():
        starts.append(bridge.n2k_transport.value)
        bridge.transport_config = bridge.current_transport_config()
    bridge.init_transport = init_transport

    def poll(dt):
        clock[0] += dt
        asyncio.run(bridge.ensure_transport())
    return bridge, starts, poll


def test_transport_waits_for_persistent_settings(monkeypatch):
    bridge, starts, poll = startup_bridge(monkeypatch)
    for _ in range(20):  # no data from the server yet
        poll(.1)
    assert starts == []

    bridge.client.received_data = True
    poll(.1)
    bridge.n2k_transport.set('socketcan')  # persistent setting arrives
    bridge.n2k_interface.set('can1')
    for _ in range(9):
        poll(.1)
    assert starts == []  # still settling
    for _ in range(20):
        poll(.1)
    assert starts == ['socketcan']  # started once, with the stored settings

    bridge.n2k_interface.set('can0')  # later changes restart immediately
    poll(.1)
    assert starts == ['socketcan', 'socketcan']


def test_transport_starts_without_server_data(monkeypatch):
    bridge, starts, poll = startup_bridge(monkeypatch)
    for _ in range(int(bridge.SETTINGS_MAX_WAIT / .1) + 1):
        poll(.1)
    assert starts == ['none']
