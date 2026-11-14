"""
Tests for the Value class hierarchy in pypilot.values.

These exercise the set/get_msg round-trips and the validation semantics
each subclass promises. The test suite here is intentionally tight so
that a future refactor of the validate/coerce contract (see PR design
notes) has a regression baseline.
"""
import pytest


@pytest.fixture
def values():
    import values as v
    return v


def test_value_get_msg_str_quoting(values):
    v = values.Value('x', 'hello')
    assert v.get_msg() == '"hello"'


def test_value_get_msg_bool(values):
    assert values.Value('x', True).get_msg() == 'true'
    assert values.Value('x', False).get_msg() == 'false'


def test_value_get_msg_number(values):
    assert values.Value('x', 3.14).get_msg() == '3.14'


def test_range_property_accepts_in_range(values):
    p = values.RangeProperty('p', 5.0, 0.0, 10.0)
    p.set(7.5)
    assert p.value == 7.5


def test_range_property_rejects_out_of_range(values):
    p = values.RangeProperty('p', 5.0, 0.0, 10.0)
    p.set(99)
    assert p.value == 5.0  # unchanged


def test_range_property_rejects_non_numeric(values):
    p = values.RangeProperty('p', 5.0, 0.0, 10.0)
    p.set("not-a-number")
    assert p.value == 5.0  # unchanged


def test_range_property_step(values):
    p = values.RangeProperty('p', -1, -1, 252, step=1)
    assert p.info['step'] == 1
    p.set('5.3')
    assert p.value == 5 and isinstance(p.value, int)
    assert p.get_msg() == '5'
    p.set(253)
    assert p.value == 5  # unchanged
    assert 'step' not in values.RangeProperty('q', 5.0, 0.0, 10.0).info


@pytest.mark.parametrize('step', [None, 1, 1.5])
@pytest.mark.parametrize('invalid', [None, float('nan'), float('inf'), -float('inf'), 'NaN', 'Infinity'])
def test_range_property_rejects_null_and_nonfinite(values, step, invalid):
    p = values.RangeProperty('p', 3, 0, 10, step=step)
    p.set(invalid)
    assert p.value == 3
    p.set(6)
    assert p.value == 6


def test_range_property_fractional_step_and_minimum(values):
    p = values.RangeProperty('p', .25, .25, 10, step=1.5)
    p.set(2)
    assert p.value == 1.75


def test_enum_property_accepts_member(values):
    e = values.EnumProperty('e', 'a', ['a', 'b', 'c'])
    e.set('b')
    assert e.value == 'b'


def test_enum_property_rejects_non_member(values):
    e = values.EnumProperty('e', 'a', ['a', 'b', 'c'])
    e.set('zzz')
    assert e.value == 'a'  # unchanged


def test_boolean_property_coerces(values):
    b = values.BooleanProperty('b', False)
    b.set(1)
    assert b.value is True
    b.set(0)
    assert b.value is False


def test_resettable_value_on_falsy_resets_to_initial(values):
    r = values.ResettableValue('r', 42)
    r.set(100)
    assert r.value == 100
    r.set(False)
    assert r.value == 42


def test_rounded_value_format(values):
    rv = values.RoundedValue('rv', 3.14159)
    assert rv.get_msg() == '3.1416'


def test_rounded_value_list(values):
    rv = values.RoundedValue('rv', [1.0, 2.5])
    assert rv.get_msg() == '[1.0000, 2.5000]'
