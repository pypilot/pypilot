"""
Shared pytest fixtures for the pypilot test suite.

pypilot's modules import sibling modules by bare name (`import values`,
`from client import ...`), so this conftest prepends the `pypilot/`
source directory to `sys.path` before any test module imports them.

Many modules also call `_()` from gettext, which is normally installed
as a builtin by `pypilot/gettext_loader.py`. For test purposes we install
an identity function so modules can import even when no locale data is
available.
"""
import atexit
import builtins
from contextlib import redirect_stderr, redirect_stdout
import io
import os
import signal
import sys
import types
from unittest.mock import Mock

import pytest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
_PKG_DIR = os.path.join(_REPO_ROOT, 'pypilot')

# Both the repo root (for `import pypilot.*`) and the inner pypilot
# directory (for bare `import values`, `from nmea import ...` used
# throughout the source) need to be on the path.
for p in (_REPO_ROOT, _PKG_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)

# Install a no-op `_` so modules that depend on gettext_loader can import
# without the locale data being compiled.
if not hasattr(builtins, '_'):
    builtins._ = lambda s: s

# pypilot.linebuffer.linebuffer is a C extension that is only built
# during `pip install`. For unit tests we don't need the optimized line
# buffer; if it is not importable, install a minimal shim into
# sys.modules so that modules which import it (nonblockingpipe,
# bufferedsocket) succeed.
try:
    from pypilot.linebuffer import linebuffer as _real_linebuffer  # noqa: F401
except Exception:
    linebuffer_shim = types.ModuleType('pypilot.linebuffer.linebuffer')

    class _LineBuffer:
        def __init__(self, fd): self.fd = fd
        def recv(self): return 0
        def line(self): return ''
        def readline_nmea(self): return ''

    linebuffer_shim.LineBuffer = _LineBuffer
    # Keep the real pypilot package; only register the submodule shims.
    pkg_shim = sys.modules.get('pypilot.linebuffer')
    if pkg_shim is None:
        pkg_shim = types.ModuleType('pypilot.linebuffer')
        pkg_shim.__path__ = [os.path.join(_PKG_DIR, 'linebuffer')]
    pkg_shim.linebuffer = linebuffer_shim
    sys.modules['pypilot.linebuffer'] = pkg_shim
    sys.modules['pypilot.linebuffer.linebuffer'] = linebuffer_shim


@pytest.fixture
def make_autopilot(monkeypatch):
    # autopilot.py detaches stdout and stderr on import. Give it its own streams.
    with redirect_stdout(io.TextIOWrapper(io.BytesIO())), redirect_stderr(io.TextIOWrapper(io.BytesIO())):
        import autopilot
    from pilots.absolute import AbsolutePilot
    from pilots.basic import BasicPilot
    from pilots.simple import SimplePilot
    from pilots.wind import WindPilot
    from rudder import Rudder
    from values import Value

    class TestController(autopilot.Autopilot):
        def __del__(self):
            # The fixture closes the server; a delayed finalizer can run during another test.
            pass

    def make(client=None):
        if client is None:
            client = types.SimpleNamespace(server=autopilot.pypilotServer(), register=lambda value: value)

        with monkeypatch.context() as patch:
            patch.setattr(autopilot, 'pypilotClient', lambda *args, **kwargs: client)
            patch.setattr(autopilot, 'pypilotServer', lambda: client.server)
            patch.setattr(client.server, 'poll', lambda: None)
            patch.setattr(autopilot, 'BoatIMU', Mock())
            patch.setattr(autopilot, 'Sensors', Mock())
            patch.setattr(autopilot.servo, 'Servo', Mock())
            patch.setattr(autopilot.pilots, 'default', [BasicPilot, AbsolutePilot, WindPilot, SimplePilot])
            patch.setattr(os, 'system', lambda cmd: 0)
            patch.setattr(signal, 'signal', lambda *args: None)
            patch.setattr(atexit, 'register', lambda callback: None)
            ap = TestController()

        for name in ('gps', 'apb', 'wind', 'truewind', 'water'):
            getattr(ap.sensors, name).source = Value('source', 'none')
        ap.sensors.rudder = Rudder(client)
        ap.boatimu.SensorValues = {'compass': Value('compass', [1, 2, 3])}
        ap.update_modes()
        ap.heading.set(90)
        ap.heading_error_int_time = 0
        ap.compute_heading_error(1)
        assert ap.heading_command.value == 90 and ap.heading_command.last_user_command is None
        return ap

    return make


@pytest.fixture
def ap(make_autopilot):
    return make_autopilot()
