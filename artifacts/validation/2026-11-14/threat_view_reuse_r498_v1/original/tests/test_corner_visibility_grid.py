import math
from types import SimpleNamespace

import numpy as np
import pytest
from shapely.geometry import box
from shapely.strtree import STRtree

from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from probe_corner_visibility_grid import (arc_transforms, cast_segments, footprint_audit, horizontal_segments,
                                         scene_at, shifted_pose, steady_scenes)
from observation_witness import arc_body


def test_triangle_slice_crossing_vertex_and_two_triangles():
    triangles = [[[1, -1, 0], [1, 1, 0], [1, 1, 2]], [[1, -1, 0], [1, 1, 2], [1, -1, 2]]]
    seg = horizontal_segments(triangles, 1.)
    np.testing.assert_allclose(cast_segments(seg, [0, 0], [0.]), [1.])
    vertex = horizontal_segments([[[1, 0, 1], [1, -1, 0], [1, 2, 2]]], 1.)
    # 한 꼭짓점이 측정면 위에 있을 때도 다른 변과의 교차를 잃지 않는다.
    assert vertex.shape == (1, 2, 2)
    np.testing.assert_allclose(vertex[0], [[1., 0.], [1., .5]])


def test_coplanar_or_absent_measurement_plane_rejected():
    with pytest.raises(ValueError):
        horizontal_segments([[[0, 0, 1], [1, 0, 1], [0, 1, 1]]], 1.)
    with pytest.raises(ValueError):
        horizontal_segments([[[0, 0, 0], [1, 0, 0], [0, 1, 0]]], 1.)


def test_rays_nearest_positive_parallel_and_empty_directions():
    segments = np.array([[[2., -1.], [2., 1.]], [[1., -1.], [1., 1.]], [[-3., -1.], [-3., 1.]]])
    distances = cast_segments(segments, [0, 0], [0., math.pi, math.pi/2])
    np.testing.assert_allclose(distances[:2], [1., 3.])
    assert math.isinf(distances[2])
    assert len(cast_segments(segments, [0, 0], [])) == 0


def test_shift_is_relative_to_original_tangent_not_rotated_candidate():
    np.testing.assert_allclose(shifted_pose(2., 3., math.pi/2, .06, .1), [1.94, 3., math.pi/2+.1])


def test_static_scene_retains_500_rays_age_unknown_and_rear_mask():
    segments = np.array([[[13., -100.], [13., 100.]]])
    raw = SimpleNamespace(angle_min=-math.pi, angle_increment=math.tau/500, range_min=.05,
                          range_max=12., time_increment=.1/500)
    motion = MotionConfig()
    scene, ranges = scene_at([0., 0., 0.], segments, raw, motion, .118, 30.)
    assert len(scene.rays) == 500 and math.isinf(ranges[250])
    assert scene.rays[250].status == 'unknown'
    assert scene.rays[0].status == 'masked'
    assert scene.source_first_s == .5
    assert scene.t_ref_s == pytest.approx(.618)
    assert scene.rays[499].acquired_at_s == pytest.approx(.5998)


def test_static_body_and_future_sweep_are_separate():
    cfg = FollowConfig()
    road, obstacles = box(-.5, -.225, .5, .225), [box(.3, -.2, .35, .2)]
    tree = STRtree(obstacles)
    assert footprint_audit([0, 0, 0], 0., 0., cfg, road, obstacles, tree)['sampled_geometry_pass']
    result = footprint_audit([0, 0, 0], 0., .3, cfg, road, obstacles, tree)
    assert not result['sampled_geometry_pass'] and result['maximum_wall_overlap_m2'] > 0
    assert not result['continuous_verified']


def test_batched_origins_are_not_replaced_with_one_pose():
    seg = [[[2., -2.], [2., 2.]]]
    np.testing.assert_allclose(cast_segments(seg, [[0., 0.], [.5, 0.]], [0., 0.]), [2., 1.5])


@pytest.mark.parametrize('steer', [0., -.12, .12])
def test_acquisition_transforms_match_independent_icr_body(steer):
    cfg = FollowConfig()
    distances = np.array([-.045, -.020, 0., .03])
    result = arc_transforms(distances, steer, cfg)
    for travel, transform in zip(distances, result):
        center = arc_body(steer, travel, cfg).mean(axis=0)
        np.testing.assert_allclose(transform[:2], center, atol=1e-14, rtol=0.)
    assert result[0, 0] < 0. and result[0, 2] == pytest.approx(math.tan(steer)/cfg.wheelbase_m*distances[0])


def test_steady_history_preserves_per_ray_times_origins_and_expiry():
    raw = SimpleNamespace(angle_min=-math.pi, angle_increment=math.tau/500, range_min=.05,
                          range_max=12., time_increment=.1/500)
    cfg, motion = FollowConfig(), MotionConfig()
    segments = [[[2., -2.], [2., 2.]], [[-2., -2.], [-2., 2.]], [[-2., 2.], [2., 2.]], [[-2., -2.], [2., -2.]]]
    sources = steady_scenes([0., 0., 0.], segments, raw, motion, cfg, .118, 30., .12)
    assert len(sources) == 2
    assert [s.source_first_s for s in sources] == [.5, .4]
    assert all(s.t_ref_s == pytest.approx(.618) for s in sources)
    assert sources[0].rays[0].ray_origin_xy_m != sources[0].rays[499].ray_origin_xy_m
    assert sources[0].rays[0].status == 'masked'
    for source in sources:
        assert source.t_ref_s-source.source_first_s < .3
        # 원래 월드의 벽에 닿는다는 것을 현재 좌표로 직접 확인한다.
        for ray in source.rays:
            if ray.status == 'return':
                assert min(abs(abs(v)-2.) for v in ray.hit_xy_m) < 1e-10


def test_negative_travel_audits_history_not_only_endpoint():
    cfg, obstacles = FollowConfig(), [box(-.22, -.2, -.21, .2)]
    result = footprint_audit([0., 0., 0.], 0., -.5, cfg, box(-1, -1, 1, 1), obstacles, STRtree(obstacles))
    assert result['samples'] == 51 and not result['sampled_geometry_pass']


@pytest.mark.parametrize('origin,angles', [([math.nan, 0], [0]), ([0, 0], [math.inf]), ([0], [0])])
def test_invalid_ray_query_rejected(origin, angles):
    with pytest.raises(ValueError):
        cast_segments([[[1, -1], [1, 1]]], origin, angles)
