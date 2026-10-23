"""깊이 질의의 비용 최적화와 독립된 기존 감사 구현을 대조한다."""
from dataclasses import replace
import math

import numpy as np
import pytest

from arena_autonomy.observation_support import depth_support
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_visibility import bracket_depth
from test_local_follow import history, scan


@pytest.mark.parametrize('speed,yaw', [(0.,0.), (.2,.7), (.9,-1.3), (1.0,2.)])
@pytest.mark.parametrize('seed', [17, 23, 91])
def test_moving_origins_and_unknowns_match_frozen_audit(speed, yaw, seed):
    rng = np.random.default_rng(seed)
    raw = scan(.5, rng.uniform(.06, 12., 500).tolist())
    for index in rng.choice(500, 30, replace=False): raw.ranges[index] = math.inf
    scene = motion_scene(raw, history(speed,yaw), .635, timing_verified=True, rear_half_angle_deg=30.)
    q = rng.uniform([-.3,-.7], [2.,.7], (240,2))
    # 광선 일치·미세 각도 양쪽·센서 원점을 함께 대조한다.
    boundary = []
    for ray in scene.rays[200:210]:
        origin = np.array(ray.ray_origin_xy_m)
        d = np.array(ray.direction_xy)
        for shift in (0., -1e-11, 1e-11, -1e-9, 1e-9):
            c,s = math.cos(shift),math.sin(shift)
            boundary.append(origin+.5*np.array([c*d[0]-s*d[1],s*d[0]+c*d[1]]))
        boundary.append(origin)
    q = np.vstack((q,boundary))
    padding = rng.uniform(0., .2, 500)
    expected = [row['supported'] for row in bracket_depth(scene,q,padding)]
    np.testing.assert_array_equal(depth_support(scene,q,padding), expected)


def test_empty_queries_preserve_empty_result():
    observed = motion_scene(scan(),history(),.635,timing_verified=True)
    assert depth_support(observed,[],np.zeros(500)).shape == (0,)
