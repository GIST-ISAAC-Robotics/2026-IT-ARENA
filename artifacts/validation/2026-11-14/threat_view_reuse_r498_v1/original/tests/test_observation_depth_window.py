"""각도 창의 동등성 및 별도 원점 특이점 fail-closed 계약."""
from dataclasses import replace
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest

from arena_autonomy.observation_support import depth_support
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_visibility import bracket_depth
from test_local_follow import history, scan
from test_observation_depth_chunk import candidate as full_reference

spec = importlib.util.spec_from_file_location('depth_window_candidate',
    Path(__file__).resolve().parents[1]/'artifacts/validation/2026-10-19/g1_depth_window_v1/candidate.py')
candidate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(candidate)


def origin_near(scene, queries):
    # 독립 scalar hypot으로 수치 특이점을 판단한다. 창/벡터 선택식 재사용 없음.
    return np.array([any(math.dist(q, r.ray_origin_xy_m) <= 1e-9 for r in scene.rays)
                     for q in queries], dtype=bool)


def guarded_reference(scene, queries, padding):
    expected = full_reference.depth_support(scene, queries, padding)
    expected[origin_near(scene, queries)] = False
    return expected


@pytest.mark.parametrize('speed,yaw', [(0., 0.), (.2, .7), (.9, -1.3), (1., 2.)])
@pytest.mark.parametrize('seed', [19, 73, 1019])
def test_actual_origins_boundaries_unknown_and_source_seam(speed, yaw, seed):
    rng = np.random.default_rng(seed)
    raw = scan(.5, rng.uniform(.06, 12., 500).tolist())
    for i in rng.choice(500, 30, replace=False):
        raw.ranges[i] = math.inf
    scene = motion_scene(raw, history(speed, yaw), .635, timing_verified=True, rear_half_angle_deg=30.)
    queries = list(rng.uniform([-.3, -.7], [2., .7], (240, 2)))
    for ray in [*scene.rays[:3], *scene.rays[248:253], *scene.rays[-3:]]:
        o, d = np.array(ray.ray_origin_xy_m), np.array(ray.direction_xy)
        queries.append(o)
        for angle in (0., -1e-11, 1e-11, -1e-9, 1e-9, -.025, .025):
            c, s = math.cos(angle), math.sin(angle)
            queries.append(o+.2*np.array([c*d[0]-s*d[1], s*d[0]+c*d[1]]))
    pad = rng.uniform(0., .2, 500)
    # 창 최적화와 별도 계약 변경을 구별한다. 보존 후보는 원래 의미와 비교한다.
    original = full_reference.depth_support(scene, queries, pad)
    np.testing.assert_array_equal(candidate.depth_support(scene, queries, pad), original)
    expected = guarded_reference(scene, queries, pad)
    np.testing.assert_array_equal(depth_support(scene, queries, pad), expected)
    audited = np.array([r['supported'] for r in bracket_depth(scene, queries, pad)])
    np.testing.assert_array_equal(expected, audited)


def test_exact_origin_signed_zero_no_longer_changes_current_decision():
    scene = motion_scene(scan(.5), history(.9, -1.3), .635, timing_verified=True, rear_half_angle_deg=30.)
    queries = [r.ray_origin_xy_m for r in scene.rays[-3:]]
    pad = np.zeros(500)
    # 운영의 명시적 합은 (-0)+(-0)=-0, 감사의 reduce 합은 +0이 된다.
    # 원점의 정의되지 않는 방위가 pi / 0으로 갈려 선택 광선도 달라진다.
    np.testing.assert_array_equal(full_reference.depth_support(scene, queries, pad), [True]*3)
    np.testing.assert_array_equal(candidate.depth_support(scene, queries, pad), [True]*3)
    np.testing.assert_array_equal(depth_support(scene, queries, pad), [False]*3)
    audit = bracket_depth(scene, queries, pad)
    assert [r['supported'] for r in audit] == [False]*3
    assert {r['reason'] for r in audit} == {'query_at_ray_origin'}


@pytest.mark.parametrize('count', [0, 1, 127, 128, 129, 257, 1701])
def test_chunk_boundaries_input_immutability_and_bounding_disk_fallback(count):
    rng = np.random.default_rng(19)
    scene = motion_scene(scan(.5), history(.9, -1.3), .635, timing_verified=True)
    q = rng.uniform([-.04, -.03], [.08, .03], (count, 2))
    before, pad = q.copy(), np.full(500, .05)
    expected = guarded_reference(scene, q, pad)
    np.testing.assert_array_equal(depth_support(scene, q, pad), expected)
    np.testing.assert_array_equal(candidate.depth_support(scene, q, pad), expected)
    np.testing.assert_array_equal(q, before)
    np.testing.assert_array_equal(pad, np.full(500, .05))


def test_ties_select_original_array_order_not_angular_sort_order():
    scene = motion_scene(scan(.5), history(), .635, timing_verified=True, rear_half_angle_deg=0.)
    rays = list(scene.rays)
    # 배열 앞쪽의 동일 방향 unknown이 뒤의 반환보다 먼저 골라져야 한다.
    rays[249] = replace(rays[249], direction_xy=rays[250].direction_xy, hit_xy_m=None, status='unknown')
    changed = replace(scene, rays=tuple(rays))
    q, pad = [[.5, 0.]], np.full(500, .05)
    np.testing.assert_array_equal(candidate.depth_support(changed, q, pad), depth_support(changed, q, pad))
    assert not candidate.depth_support(changed, q, pad)[0]


@pytest.mark.parametrize('spread', [0., .001, .1, 1., 10.])
@pytest.mark.parametrize('seed', [8, 23, 1019])
def test_arbitrary_origins_angle_seams_and_bounding_circle_edges(spread, seed):
    rng = np.random.default_rng(seed)
    source = motion_scene(scan(.5), history(), .635, timing_verified=True, rear_half_angle_deg=0.)
    origins = rng.uniform(-spread, spread, (500, 2))
    angles = rng.uniform(-math.pi, math.pi, 500)
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    distances = rng.uniform(.05, 12., 500)
    rays = []
    for i, r in enumerate(source.rays):
        status = 'unknown' if i % 17 == 0 else 'masked' if i % 29 == 0 else 'return'
        rays.append(replace(r, ray_origin_xy_m=tuple(origins[i]), direction_xy=tuple(directions[i]),
            hit_xy_m=tuple(origins[i]+distances[i]*directions[i]) if status == 'return' else None, status=status))
    changed = replace(source, rays=tuple(rays))
    center = origins.mean(axis=0)
    radius = np.linalg.norm(origins-center, axis=1).max()
    phi = np.linspace(-math.pi, math.pi, 129)
    rim = np.column_stack((np.cos(phi), np.sin(phi)))
    q = np.vstack((rng.uniform(-3., 3., (259, 2)),
        *[center+rim*(radius+shift) for shift in (-1e-10, 0., 1e-12, 1e-10)], origins[::37]))
    pad = rng.uniform(0., .2, 500)
    np.testing.assert_array_equal(depth_support(changed, q, pad), guarded_reference(changed, q, pad))


def test_disordered_large_origin_disk_uses_all_rays_when_needed():
    scene = motion_scene(scan(.5), history(), .635, timing_verified=True, rear_half_angle_deg=0.)
    rays = list(scene.rays)
    # 일정 운동/원점 정렬에 의존하지 않는 상계인지 검사한다.
    rays[0] = replace(rays[0], ray_origin_xy_m=(1., 1.))
    changed = replace(scene, rays=tuple(rays))
    q, pad = [[.1, .1], [.5, 0.], [2., 1.], [0., 0.]], np.full(500, .05)
    np.testing.assert_array_equal(candidate.depth_support(changed, q, pad), depth_support(changed, q, pad))


@pytest.mark.parametrize('angle', [0., math.pi/2, math.pi, -math.pi/2])
@pytest.mark.parametrize('distance', [0., .9e-9, 1e-9, 1.1e-9, 2e-9])
def test_origin_guard_near_boundary_preserves_all_other_decisions(angle, distance):
    scene = motion_scene(scan(.5), history(), .635, timing_verified=True, rear_half_angle_deg=0.)
    # 원점 0인 광선이 원점 원의 가장 바깥쪽에 있어도 epsilon 범위는 전체 계산한다.
    rays = list(scene.rays)
    rays[0] = replace(rays[0], ray_origin_xy_m=(0., 0.))
    scene = replace(scene, rays=tuple(rays))
    query = [[distance*math.cos(angle), distance*math.sin(angle)]]
    pad = np.zeros(500)
    expected = guarded_reference(scene, query, pad)
    np.testing.assert_array_equal(depth_support(scene, query, pad), expected)
    np.testing.assert_array_equal([r['supported'] for r in bracket_depth(scene, query, pad)], expected)
    if distance <= 1e-9:
        assert not expected[0]
