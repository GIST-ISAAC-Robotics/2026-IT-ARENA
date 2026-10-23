import numpy as np
import pytest

from review_corridor_handoffs import geometric_difference


def line(offset=0.):
    return np.column_stack((np.linspace(0., 1., 51), np.full(51, offset)))


def test_numeric_difference_is_not_timing_authorization():
    a = geometric_difference(line(), line(.04))
    assert a['evaluated'] and a['symmetric_p80_m'] == pytest.approx(.04)
    assert 'continuity_certified' not in a and 'driving_authorized' not in a


def test_short_shared_path_is_unavailable_not_zero_error():
    result = geometric_difference(line(), line()[:8])
    assert not result['evaluated'] and 'symmetric_p80_m' not in result


def test_invalid_path_is_not_passed():
    with pytest.raises(ValueError):
        geometric_difference(line(), [[float('nan'), 0.], [1., 0.]])
