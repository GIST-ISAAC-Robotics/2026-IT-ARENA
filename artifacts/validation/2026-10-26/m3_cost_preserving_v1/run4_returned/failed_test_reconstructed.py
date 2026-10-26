from dataclasses import replace
import math
import numpy as np
import pytest
from arena_autonomy.m3_execution_trial import _both_sides_returned
from test_scan_topology import fixture
from returned_candidate import both_returned_window


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
