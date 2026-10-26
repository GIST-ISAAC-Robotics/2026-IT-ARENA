"""전광선 기준식 대조. 원점/동률/가림/무응답/순환 증거를 보존한다."""
from dataclasses import replace
import math
import numpy as np
import pytest
from arena_autonomy.scan_topology import cyclic_count, adjacent_indices
def _both_sides_returned(scene, points, *, cycle=None):
    """depth_support와 같은 양옆 광선 규칙에서 두 광선이 모두 반환인지(무응답·마스크가 아닌지)."""
    cycle_count = cyclic_count(scene, cycle)
    origins = np.array([r.ray_origin_xy_m for r in scene.rays])
    directions = np.array([r.direction_xy for r in scene.rays])
    status = np.array([r.status for r in scene.rays])
    indices = np.array([r.source_index for r in scene.rays])
    out = np.zeros(len(points), dtype=bool)
    for k in range(0, len(points), 128):
        d = points[k:k+128, None, :]-origins[None, :, :]
        angle = np.arctan2(directions[None, :, 0]*d[:, :, 1]-directions[None, :, 1]*d[:, :, 0],
                           directions[None, :, 0]*d[:, :, 0]+directions[None, :, 1]*d[:, :, 1])
        low = np.where(angle >= -1e-10, np.maximum(angle, 0.), np.inf)
        high = np.where(angle <= 1e-10, np.maximum(-angle, 0.), np.inf)
        lo, hi = np.argmin(low, axis=1), np.argmin(high, axis=1)
        rows = np.arange(len(d))
        out[k:k+128] = (np.isfinite(low[rows, lo]) & np.isfinite(high[rows, hi])
            & (low[rows, lo]+high[rows, hi] <= .025)
            & adjacent_indices(indices[lo], indices[hi], cycle_count=cycle_count)
            & (status[lo] == 'return') & (status[hi] == 'return'))
    return out

from test_scan_topology import fixture
from arena_autonomy.m3_execution_trial import _both_sides_returned as both_returned_window


@pytest.mark.parametrize('moving', [False, True])
@pytest.mark.parametrize('mask', [0., 30.])
@pytest.mark.parametrize('seed', [3, 19, 599])
def test_matches_all_ray_reference_for_origins_unknowns_and_source_seam(moving, mask, seed):
    rng = np.random.default_rng(seed)
    values = rng.uniform(.1, 10., 500)
    values[[0, 1, 8, 249, 499]] = [np.nan, np.inf, 0., np.nan, np.inf]
    cycle = fixture(moving=moving, mask=mask, values=values.tolist())
    origins = np.array([r.ray_origin_xy_m for r in cycle.scene.rays])
    points = np.vstack((rng.uniform(-4., 4., (701, 2)), origins,
                        origins+1e-12, origins-1e-10, origins+[0., 1e-9]))
    for evidence in (None, cycle):
        expected = _both_sides_returned(cycle.scene, points, cycle=evidence)
        actual = both_returned_window(cycle.scene, points, cycle=evidence)
        assert np.array_equal(expected, actual)


@pytest.mark.parametrize('increment', [math.pi/500, -math.tau/500, math.tau/499])
def test_partial_reversed_or_duplicate_endpoints_keep_original_indices(increment):
    if increment < 0:
        with pytest.raises(ValueError, match='invalid scan metadata'):
            fixture(increment=increment, moving=True)
        return
    cycle = fixture(increment=increment, moving=True)
    points = np.random.default_rng(13).uniform(-2., 2., (900, 2))
    for c in (None, cycle):
        assert np.array_equal(_both_sides_returned(cycle.scene, points, cycle=c),
                              both_returned_window(cycle.scene, points, cycle=c))


def test_near_angle_threshold_and_tied_directions():
    cycle = fixture()
    scene = cycle.scene
    rays = list(scene.rays)
    rays[250] = replace(rays[250], direction_xy=rays[249].direction_xy)
    scene = replace(scene, rays=tuple(rays))
    angle = np.r_[np.linspace(-math.pi, math.pi, 1500), [-1e-10, 0., 1e-10, .025, -.025]]
    points = np.column_stack((.06+np.cos(angle), np.sin(angle)))
    assert np.array_equal(_both_sides_returned(scene, points), both_returned_window(scene, points))
    assert both_returned_window(scene, []).shape == (0,)
