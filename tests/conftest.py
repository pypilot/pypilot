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
import builtins
import os
import sys
import types

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
