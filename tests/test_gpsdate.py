import pytest
from scripts import gpsdate


class FakeStream:
    def __init__(self, lines):
        self.lines = lines

    def __enter__(self):
        return iter(self.lines)

    def __exit__(self, exc_type, exc_value, traceback):
        return False


class FakeConnection:
    def __init__(self, lines):
        self.lines = lines
        self.timeout = None
        self.sent = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        return False

    def settimeout(self, timeout):
        self.timeout = timeout

    def sendall(self, data):
        self.sent = data

    def makefile(self, *args, **kwargs):
        return FakeStream(self.lines)


def test_read_gps_time_uses_json_watch_and_returns_tpv_time(monkeypatch):
    connection = FakeConnection(
        [
            "not json\n",
            '{"class":"VERSION"}\n',
            '{"class":"TPV","time":"2026-07-15T14:32:07.000Z"}\n',
        ]
    )
    calls = []

    def create_connection(address, timeout):
        calls.append((address, timeout))
        return connection

    monkeypatch.setattr(gpsdate.socket, "create_connection", create_connection)

    assert gpsdate.read_gps_time() == "2026-07-15T14:32:07.000Z"
    assert calls == [(gpsdate.GPSD_ADDRESS, 5)]
    assert connection.timeout == gpsdate.GPS_TIMEOUT
    assert connection.sent == b'?WATCH={"enable":true,"json":true};\n'


def test_read_gps_time_rejects_stream_without_timestamp(monkeypatch):
    connection = FakeConnection(['{"class":"TPV","mode":1}\n'])
    monkeypatch.setattr(gpsdate.socket, "create_connection", lambda address, timeout: connection)

    with pytest.raises(RuntimeError, match="without a GPS timestamp"):
        gpsdate.read_gps_time()


def test_set_system_time_converts_gpsd_timestamp(monkeypatch):
    calls = []

    def run(command, check):
        calls.append((command, check))

    monkeypatch.setattr(gpsdate.subprocess, "run", run)

    gpsdate.set_system_time("2026-07-15T14:32:07.000Z")

    assert calls == [(["date", "-u", "-s", "2026-07-15 14:32:07"], True)]
