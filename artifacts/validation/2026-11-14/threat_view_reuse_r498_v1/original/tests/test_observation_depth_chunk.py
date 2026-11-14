"""작은 행 묶음의 깊이 계산은 광선/질의 선택과 순서를 바꾸지 않는다."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from arena_autonomy.observation_support import depth_support
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_visibility import bracket_depth
from test_local_follow import history, scan


spec = importlib.util.spec_from_file_location('depth_chunk_candidate',
    Path(__file__).resolve().parents[1]/'artifacts/validation/2026-10-17/g1_recent_depth_chunk_v1/candidate.py')
candidate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(candidate)


@pytest.mark.parametrize('size', [64, 128, 256])
@pytest.mark.parametrize('count', [0, 1, 63, 64, 65, 127, 128, 129, 255, 256, 257, 1598])
@pytest.mark.parametrize('velocity,yaw', [(0., 0.), (.9, -1.3)])
def test_chunk_boundaries_do_not_drop_or_reorder_queries(size, count, velocity, yaw):
    rng = np.random.default_rng(1017)
    raw = scan(.5, rng.uniform(.1, 12., 500).tolist())
    for i in rng.choice(500, 40, replace=False):
        raw.ranges[i] = float('inf')
    scene = motion_scene(raw, history(velocity, yaw), .635, timing_verified=True, rear_half_angle_deg=30.)
    queries = rng.uniform([-.3, -.7], [2., .7], (count, 2))
    pad = rng.uniform(0., .2, 500)
    before, original_pad = queries.copy(), pad.copy()
    expected = np.array([r['supported'] for r in bracket_depth(scene, queries, pad)], dtype=bool)
    np.testing.assert_array_equal(candidate.depth_support(scene, queries, pad, size),
                                  expected)
    np.testing.assert_array_equal(depth_support(scene, queries, pad), expected)
    np.testing.assert_array_equal(queries, before)
    np.testing.assert_array_equal(pad, original_pad)
