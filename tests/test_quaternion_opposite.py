import math
import pytest
from pypilot import quaternion


@pytest.mark.parametrize('a', [[0, 0, 1], [1, 0, 0], [0, 1, 0], [1, 2, 3]])
def test_opposite_vectors_produce_unit_rotation(a):
    b = [-x for x in a]
    q = quaternion.vec2vec2quat(a, b)
    assert sum(x*x for x in q) == pytest.approx(1)
    assert quaternion.rotvecquat(a, q) == pytest.approx(b)


@pytest.mark.parametrize('a,b', [([0, 0, 1], [0, 0, 1]),
                                ([1, 0, 0], [0, 1, 0])])
def test_parallel_and_general_alignment_unchanged(a, b):
    q = quaternion.vec2vec2quat(a, b)
    assert quaternion.rotvecquat(a, q) == pytest.approx(b)
