"""
hat/gpio.py has to work with both the libgpiod 1.x python bindings
(python3-libgpiod on debian bookworm) and the 2.x bindings.  These
tests swap in a fake gpiod module for each API and press one key.
"""
import importlib.util
import os
import sys
import threading
import time
import types

import pytest

_GPIO_PY = os.path.join(os.path.dirname(__file__), '..', 'hat', 'gpio.py')


def load_gpio_module():
    spec = importlib.util.spec_from_file_location('hat_gpio_under_test', _GPIO_PY)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Script:
    """hands out one batch of edges per wait, then times out"""
    def __init__(self, batches):
        self.batches = list(batches)
        self.done = threading.Event()

    def next(self):
        if self.batches:
            return self.batches.pop(0)
        self.done.set()
        time.sleep(.01)
        return None


def fake_gpiod_v1(script):
    gpiod = types.ModuleType('gpiod')
    gpiod.LINE_REQ_EV_BOTH_EDGES = 'both'
    gpiod.LINE_REQ_FLAG_BIAS_PULL_UP = 'pullup'
    gpiod.requested = {}
    levels = {}

    class Line:
        def __init__(self, offset):
            self._offset = offset

        def offset(self):
            return self._offset

        def event_read_multiple(self):
            return []

        def get_value(self):
            return levels[self._offset]

    class LineBulk:
        def __init__(self, offsets):
            self.lines = {o: Line(o) for o in offsets}

        def request(self, consumer, type, flags=0):
            gpiod.requested.update(consumer=consumer, type=type, flags=flags,
                                   offsets=sorted(self.lines))

        def event_wait(self, sec):
            batch = script.next()
            if batch is None:
                return None
            levels.update(batch)
            return [self.lines[o] for o, _level in batch]

    class Chip:
        def __init__(self, path):
            gpiod.requested['chip'] = path

        def get_lines(self, offsets):
            return LineBulk(offsets)

    gpiod.Chip = Chip
    return gpiod


def fake_gpiod_v2(script):
    gpiod = types.ModuleType('gpiod')
    gpiod.requested = {}
    gpiod.line = types.SimpleNamespace(
        Direction=types.SimpleNamespace(INPUT='input'),
        Edge=types.SimpleNamespace(BOTH='both'),
        Bias=types.SimpleNamespace(PULL_UP='pullup'))

    class LineSettings:
        def __init__(self, **kwargs):
            self.kwargs = kwargs

    class EdgeEvent:
        Type = types.SimpleNamespace(RISING_EDGE='rising', FALLING_EDGE='falling')

        def __init__(self, line_offset, event_type):
            self.line_offset = line_offset
            self.event_type = event_type

    class Request:
        def wait_edge_events(self, timeout):
            self.pending = script.next()
            return self.pending is not None

        def read_edge_events(self):
            return [EdgeEvent(o, t) for o, t in self.pending]

    def request_lines(path, consumer, config):
        gpiod.requested.update(chip=path, consumer=consumer, config=config)
        return Request()

    gpiod.LineSettings = LineSettings
    gpiod.request_lines = request_lines
    return gpiod


@pytest.fixture(autouse=True)
def in_process_pipe(monkeypatch):
    # the os pipe needs the linebuffer C extension, which conftest stubs out
    from pypilot import nonblockingpipe
    real = nonblockingpipe.NonBlockingPipe
    monkeypatch.setattr(nonblockingpipe, 'NonBlockingPipe', lambda name, use_mp: real(name, False))


def run_keypress(monkeypatch, gpiod, script):
    monkeypatch.setitem(sys.modules, 'gpiod', gpiod)
    module = load_gpio_module()
    g = module.gpio()
    assert script.done.wait(5), 'gpio thread did not finish the script'
    events = []
    for _ in range(3):
        events += g.poll()
    g.thread_running = False
    g.thread.join(5)
    assert not g.thread.is_alive()
    return g, events


def test_libgpiod_v1_bindings(monkeypatch):
    # key on gpio 17 pressed (pulled low) then released (high again)
    script = Script([[(17, 0)], [(17, 1)]])
    gpiod = fake_gpiod_v1(script)
    g, events = run_keypress(monkeypatch, gpiod, script)

    assert gpiod.requested == {'chip': '/dev/gpiochip0', 'consumer': 'keys', 'type': 'both',
                               'flags': 'pullup', 'offsets': sorted(g.pins)}
    assert events == [('gpio17', 1), ('gpio17', 0)]


def test_libgpiod_v2_bindings(monkeypatch):
    script = Script([[(17, 'falling')], [(17, 'rising')]])
    gpiod = fake_gpiod_v2(script)
    g, events = run_keypress(monkeypatch, gpiod, script)

    assert gpiod.requested['chip'] == '/dev/gpiochip0'
    assert sorted(gpiod.requested['config']) == sorted(g.pins)
    assert events == [('gpio17', 1), ('gpio17', 0)]


def test_no_gpiod(monkeypatch):
    monkeypatch.setitem(sys.modules, 'gpiod', None)  # import gpiod raises ImportError
    module = load_gpio_module()
    g = module.gpio()
    g.thread.join(5)
    assert g.poll() == []


if __name__ == '__main__':
    sys.exit(pytest.main([__file__]))
