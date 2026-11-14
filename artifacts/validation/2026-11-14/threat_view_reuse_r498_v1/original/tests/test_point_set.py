"""外곽의 얇은 표본을 합치거나 순서를 바꾸지 않는 정렬 최적화 대조."""
import numpy as np
import pytest

from arena_autonomy.point_set import unique_xy


@pytest.mark.parametrize('seed', range(8))
def test_exact_order_and_bytes_of_finite_poses(seed):
    rng = np.random.default_rng(seed)
    values = rng.normal(size=(2400, 2))
    angle = .3*seed
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    values = np.vstack((values@rotation, values[:500]@rotation, np.zeros((12, 2))))
    rng.shuffle(values)
    original = values.copy()
    result = unique_xy(values)
    expected = np.unique(values, axis=0)
    assert result.shape == expected.shape and result.dtype == expected.dtype
    assert result.tobytes() == expected.tobytes()
    assert values.tobytes() == original.tobytes()


def test_adjacent_float_values_remain_distinct_and_noncontiguous_input_works():
    x = np.array([0., 1e-300, 1., 1e100])
    values = np.column_stack((np.repeat(x, 2), np.tile([1., np.nextafter(1., 2.)], 4)))
    assert len(unique_xy(values)) == 8
    np.testing.assert_array_equal(unique_xy(values[::-1]), np.unique(values, axis=0))


@pytest.mark.parametrize('values', [np.empty((0, 2)), np.array([[1., 2.]]),
    np.array([[0., -0.], [-0., 0.], [0., 0.], [-1., -0.]]),
    np.array([[float('nan'), 1.], [float('inf'), 0.], [1., 0.], [float('nan'), 1.]]),
    np.array([[2, 1], [1, 2], [2, 1]], dtype=np.int64),
    np.array([[2., 1.], [1., 2.], [2., 1.]], dtype=np.float32)])
def test_existing_representative_semantics_for_boundaries(values):
    result, expected = unique_xy(values), np.unique(values, axis=0)
    assert result.shape == expected.shape and result.dtype == expected.dtype
    assert result.tobytes() == expected.tobytes()
