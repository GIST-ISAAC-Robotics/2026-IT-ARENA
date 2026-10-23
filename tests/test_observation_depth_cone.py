"""atan2 사전 제외의 각도 경계/광선 상태를 전체 각도 계산과 대조한다."""
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from arena_autonomy.local_scene import LocalScene, SensorRay
from arena_autonomy.observation_support import depth_support
from audit_corridor_visibility import bracket_depth


spec = importlib.util.spec_from_file_location('depth_cone_candidate',
    Path(__file__).resolve().parents[1]/'artifacts/validation/2026-10-17/g1_recent_depth_cone_v1/candidate.py')
candidate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(candidate)


def angular_fixture(a, b, state='return'):
    rays = []
    for i in range(500):
        angle = a if i == 249 else b if i == 250 else math.pi
        direction = (math.cos(angle), math.sin(angle))
        status = state if i == 249 else 'return'
        hit = (3*direction[0], 3*direction[1]) if status == 'return' else None
        rays.append(SensorRay((0., 0.), hit, .5+i*.0002, status, 'laser_frame', 0, i, direction))
    return LocalScene(.635, .5, .5998, 'laser_frame', 0, tuple(rays))


@pytest.mark.parametrize('a,b', [(-.01249, .01249), (-.0125, .0125), (-.01251, .01251),
    (-.025+1e-12, 0.), (-.025-1e-12, 0.), (-.02999, .00001), (-.031, .00001),
    (-1e-10, 1e-10), (-1e-10-1e-14, 1e-10+1e-14), (1e-11, 1e-11),
    (-1e-11, -1e-11), (math.pi, -math.pi), (-1.5, 1.5), (-3., 3.)])
@pytest.mark.parametrize('state', ['return', 'unknown', 'masked'])
def test_angle_limits_and_missing_candidates_match_complete_angle_reference(a, b, state):
    scene = angular_fixture(a, b, state)
    q = np.array([[1., 0.], [2.9, 0.], [.1, 0.], [1., .01], [1., -.01]])
    pad = np.full(500, .1)
    expected = [r['supported'] for r in bracket_depth(scene, q, pad)]
    for fn in (depth_support, candidate.depth_support):
        np.testing.assert_array_equal(fn(scene, q, pad), expected)


def test_cone_is_only_necessary_condition_not_angular_limit_relaxation():
    assert math.atan(.03) > .025
    scene = angular_fixture(-.014, .014)
    for fn in (depth_support, candidate.depth_support):
        assert not fn(scene, [[1., 0.]], np.zeros(500))[0]


@pytest.mark.parametrize('seed', [17, 23, 91])
def test_different_origins_and_padding_do_not_assume_one_ray_origin(seed):
    from arena_autonomy.scene_motion import motion_scene
    from test_local_follow import history, scan
    rng = np.random.default_rng(seed)
    raw = scan(.5, rng.uniform(.1, 12., 500).tolist())
    for i in rng.choice(500, 40, replace=False):
        raw.ranges[i] = math.inf
    scene = motion_scene(raw, history(.9, -1.3), .635, timing_verified=True, rear_half_angle_deg=30.)
    q = rng.uniform([-.4, -.8], [2.5, .8], (700, 2))
    pad = rng.uniform(.0, .2, 500)
    expected = [r['supported'] for r in bracket_depth(scene, q, pad)]
    for fn in (depth_support, candidate.depth_support):
        np.testing.assert_array_equal(fn(scene, q, pad), expected)
