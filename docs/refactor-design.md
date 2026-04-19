# Draft: refactor design notes

This document accompanies the draft PR that introduces a pytest
scaffold, a starter test suite, and this design note. The refactors
described below are **not** implemented in the draft PR because they
touch hot code paths and warrant focused review before they land. The
test suite in `tests/` is the regression baseline the refactors would
build on.

## 1. Unified pipe abstraction

**Problem.** `pypilot/nonblockingpipe.py` currently hosts four pipe
implementations — OS pipes (`PipeNonBlockingPipeEnd`),
multiprocessing pipes (`MultiprocessingNonBlockingPipeEnd`), socket
pairs (`SocketNonBlockingPipeEnd`), and a pure-Python fallback
(`NoMPLineBufferedPipeEnd`) — with different buffering and backpressure
semantics. `send()` can silently return `False` on buffer full when
`sendfailok` is set, and none of the callers in `server.py` or
`nmea.py` currently check that return value. This is a silent data
loss path.

**Proposed direction.** Introduce a formal `PipeEnd` ABC with:

```python
class PipeEnd(abc.ABC):
    @abc.abstractmethod
    def send(self, data: bytes) -> None:
        """Send data, raising IOError if the buffer is full. No silent fail."""
    @abc.abstractmethod
    def recv(self) -> Optional[bytes]:
        """Return one message or None if empty."""
    @abc.abstractmethod
    def fileno(self) -> int: ...
    @abc.abstractmethod
    def close(self) -> None: ...
```

Each existing backend becomes a concrete subclass, with `send()`
raising instead of returning `False`. Call-sites decide per-site
whether to drop-and-log or propagate. Remove the `sendfailok` flag
entirely.

**Risk.** Changing return semantics is a breaking change for any
external caller we missed. The test suite would need an end-to-end
test of the server/pipe round-trip before we land it.

## 2. Value validate/coerce contract

**Problem.** `pypilot/values.py` defines 14 subclasses with
inconsistent `set()`/`get_msg()` contracts:

- `RangeProperty.set()` silently returns on non-numeric input.
- `EnumProperty.set()` accepts floating-point equivalents (`"10.0"` ≡ `10`).
- `HeadingOffset` uses `.update()` not `.set()`.
- `StringValue.get_msg()` quotes; `Value.get_msg()` does not when
  the value is not a string.

**Proposed direction.** Introduce two methods on `Value`:

```python
class Value(abc.ABC):
    def validate(self, value) -> tuple[bool, str]: ...
    def coerce(self, value): ...
    def set(self, value):
        valid, msg = self.validate(value)
        if not valid:
            raise ValueError(f'{self.name}: {msg}')
        self._value = self.coerce(value)
```

Each subclass provides `validate`/`coerce`. The SignalK mapper can
then rely on a consistent contract instead of sniffing `isinstance`
for every subclass. Tests in `tests/test_values.py` provide the
regression baseline.

**Risk.** Changing `set()` from silent-ignore to raise is a
breaking change for clients that rely on the old "silently ignore"
behavior. Might need to stage behind an `info['strict']` flag.

## 3. Pytest scaffolding

This draft PR implements only this piece. See `tests/` and
`pytest.ini`. 28 tests cover:

- NMEA checksum and parser correctness (APB bounds, MWV units,
  T/R distinction).
- SignalK delta parsing (basic, duplicate-skip, initial-message
  regression, malformed-input tolerance, PUT behavior, backoff).
- Value hierarchy semantics (get_msg formatting, RangeProperty
  validation, EnumProperty membership, ResettableValue,
  RoundedValue list handling, BooleanProperty coercion).

`tests/conftest.py` sets up `sys.path` and stubs out the compiled
linebuffer C extension so the tests run against an unbuilt source
tree.
