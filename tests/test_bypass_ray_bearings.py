"""추월 최초 벽 기준의 실제 광선 정렬. 합성 관측이며 추월 주행 검증은 아니다."""
import math
from dataclasses import replace
import numpy as np
import pytest
from arena_autonomy.bypass_entry_trial import observed_straight_wall
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_scene import LocalScene, SensorRay
from test_lidar_wall_bearings import TWO_DEPTH_WALLS, cast


@pytest.mark.parametrize('origin', [(0., 0.), (.066, .002)])
def test_observed_entry_wall_keeps_ray_pairs_and_all_raw_points(origin):
    actor = [(.6, -.075, .8, -.075), (.8, -.075, .8, .075),
             (.8, .075, .6, .075), (.6, .075, .6, -.075)]
    points, bearings = cast(TWO_DEPTH_WALLS+actor, origin)
    rays = tuple(SensorRay(origin, tuple(point), .6, 'return', 'laser_frame', 0,
        index, (math.cos(angle), math.sin(angle)))
        for index, (point, angle) in enumerate(zip(points, bearings)))
    rays += (SensorRay(origin, None, .6, 'unknown', 'laser_frame', 0,
                       len(rays), (-1., 0.)),)
    scene = LocalScene(.7, .6, .6, 'laser_frame', 0, rays)
    before = scene.rays
    config = replace(FollowConfig(), wall_offset_m=.505,
                     max_steering_rad=.28475436568260193, wheelbase_m=.128, length_m=.207)
    fit, detail = observed_straight_wall(scene, config, 'left')
    assert fit is not None
    assert fit[0] == pytest.approx(.3, abs=1e-12)
    assert detail['residual_m'] < 1e-12
    assert scene.rays == before
    np.testing.assert_array_equal(scene.occupied_endpoints_xy_m, points)
    assert any(.59 < p[0] < .81 and abs(p[1]) <= .076 for p in scene.occupied_endpoints_xy_m)
    assert scene.rays[-1].status == 'unknown'
