"""먼 옆 벽 연결 보완. 다른 면·빈틈·차량 사이를 잇지 않고 기본 출력은 그대로다."""
import json
import math
from pathlib import Path

import numpy as np
import pytest

from arena_autonomy import local_follow as lf
from arena_autonomy.local_path import local_path, ray_continuous_gaps
from test_low_speed_arc_reacquire import SOLO, launch_module, play_scene

FIXTURE = json.loads((Path(__file__).parent/'fixtures/solo_ray_wall_scan.json').read_text(encoding='utf-8'))
ORIGIN = np.array([.06, 0.])
HORIZON = 2.75
WALL = (-1., .40, 5., .40)
RIGHT = (-1., -.45, 5., -.45)
ROOM = [(-1., -3., -1., 3.), (-1., 3., 7., 3.), (7., 3., 7., -3.), (7., -3., -1., -3.)]


def cast(segments, noise=None):
    """같은 스캐너 배치의 이상적인 광선 추적. 무반환은 점을 만들지 않는다."""
    angles = -math.pi+np.arange(500)*math.tau/500
    best = np.full(500, np.inf)
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    for x0, y0, x1, y1 in segments:
        edge = np.array([x1-x0, y1-y0])
        offset = np.array([x0, y0])-ORIGIN
        denominator = directions[:, 0]*edge[1]-directions[:, 1]*edge[0]
        parallel = np.abs(denominator) < 1e-12
        safe = np.where(parallel, 1., denominator)
        distance = (offset[0]*edge[1]-offset[1]*edge[0])/safe
        along = (offset[0]*directions[:, 1]-offset[1]*directions[:, 0])/safe
        best = np.where((~parallel) & (distance > .05) & (along >= 0.) & (along <= 1.) & (distance < best), distance, best)
    rays = np.flatnonzero(np.isfinite(best) & (np.abs(angles) <= math.radians(150)))
    ranges = best[rays] if noise is None else best[rays]+noise[0].normal(0., noise[1], len(rays))
    return ORIGIN+directions[rays]*ranges[:, None], rays


def ahead(segments, rays=True, side='left', noise=None):
    points, ids = cast(segments, noise)
    path, info = local_path(points, side, .425, horizon=HORIZON, rays=ids if rays else None)
    assert path is not None
    return info['boundary_observed_ahead_m'], path


def test_recorded_scan_reaches_required_stop_length():
    points, rays = np.array(FIXTURE['points']), FIXTURE['rays']
    plain, plain_info = local_path(points, FIXTURE['side'], .425, FIXTURE['horizon_m'])
    joined, joined_info = local_path(points, FIXTURE['side'], .425, FIXTURE['horizon_m'], rays=rays)
    assert plain_info['boundary_observed_ahead_m'] == pytest.approx(2.2593, abs=1e-3)
    assert joined_info['boundary_observed_ahead_m'] > plain_info['boundary_observed_ahead_m']+.3
    near = plain[:, 0] <= 1.
    assert np.max(np.abs(np.interp(plain[near, 0], joined[:, 0], joined[:, 1])-plain[near, 1])) < .01


def test_clean_straight_wall_extends_past_the_distance_limit():
    plain, _ = ahead([WALL, RIGHT, *ROOM], rays=False)
    joined, _ = ahead([WALL, RIGHT, *ROOM])
    assert plain < 2.5 and joined > 2.6


def test_noisy_range_keeps_joining_because_only_normal_offset_counts():
    # 비스듬한 벽의 방사 잡음은 벽 수직 방향으로 작다. C1 실측 잡음이 아니다.
    noise = np.random.default_rng(7), .01
    plain = [ahead([WALL, RIGHT, *ROOM], rays=False, noise=noise)[0] for _ in range(10)]
    joined = [ahead([WALL, RIGHT, *ROOM], noise=noise)[0] for _ in range(10)]
    assert np.median(joined) > np.median(plain)+.15


@pytest.mark.parametrize('name,segments', [
    ('opening_shows_outer_wall', [(-1., .40, 2.45, .40), (3.05, .40, 5., .40), (2.45, 1.5, 3.05, 1.5), RIGHT, *ROOM]),
    ('recessed_parallel_surface', [(-1., .40, 2.45, .40), (2.45, .48, 5., .48), RIGHT, *ROOM]),
    ('car_rear_on_wall_rays', [WALL, RIGHT, (2.35, .25, 2.55, .25), (2.55, .25, 2.55, .39), (2.55, .39, 2.35, .39), (2.35, .39, 2.35, .25), *ROOM]),
    ('wall_turns_left', [(-1., .40, 2.45, .40), (2.45, .40, 2.45, 3.), RIGHT, *ROOM]),
    ('cross_wall', [(-1., .40, 2.45, .40), (2.45, .40, 2.45, -.45), RIGHT, *ROOM]),
])
def test_distinct_surface_is_not_joined(name, segments):
    plain, plain_path = ahead(segments, rays=False)
    joined, joined_path = ahead(segments)
    assert joined == plain and np.array_equal(joined_path, plain_path)


def test_no_return_through_opening_is_not_joined():
    # 바깥이 관측되지 않은 빈 공간. 반환 없는 광선은 연속 근거가 될 수 없다.
    segments = [(-1., .40, 2.45, .40), (3.05, .40, 5., .40), RIGHT, (-1., -3., -1., .40), (-1., -3., 7., -3.), (7., -3., 7., .40)]
    assert ahead(segments)[0] == ahead(segments, rays=False)[0]


@pytest.mark.parametrize('slope', [30., -30.])
def test_wall_turned_from_the_heading_is_not_extended(slope):
    tangent = (math.cos(math.radians(slope)), math.sin(math.radians(slope)))
    wall = (-.5, .40, -.5+6*tangent[0], .40+6*tangent[1])
    assert ahead([wall, RIGHT, *ROOM], rays=False)[0] == ahead([wall, RIGHT, *ROOM])[0]


def straight_case():
    points, rays = cast([WALL, RIGHT, *ROOM])
    order = np.argsort(-np.arctan2(points[:, 1], points[:, 0]))
    points, rays = points[order], rays[order]
    gaps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    limits = .10+.035*np.minimum(np.linalg.norm(points[:-1], axis=1), np.linalg.norm(points[1:], axis=1))
    return points, gaps, limits, rays


def test_missing_ray_blocks_each_window_it_touches():
    points, gaps, limits, rays = straight_case()
    joined = ray_continuous_gaps(points, gaps, limits, rays, HORIZON)
    index = int(np.flatnonzero(joined)[0])
    rays = rays.copy()
    rays[index+2:] += 1  # 결측 반환 하나. 점유 점 자체는 그대로 둔다.
    assert not ray_continuous_gaps(points, gaps, limits, rays, HORIZON)[index]


def test_reordered_rays_are_not_continuous():
    points, gaps, limits, rays = straight_case()
    index = int(np.flatnonzero(ray_continuous_gaps(points, gaps, limits, rays, HORIZON))[0])
    rays = rays.copy()
    rays[index], rays[index-1] = rays[index-1], rays[index]
    assert not ray_continuous_gaps(points, gaps, limits, rays, HORIZON)[index]


def test_join_stays_inside_the_search_horizon():
    points, gaps, limits, rays = straight_case()
    joined = ray_continuous_gaps(points, gaps, limits, rays, 2.)
    assert joined.sum() < ray_continuous_gaps(points, gaps, limits, rays, HORIZON).sum()
    points, rays = cast([WALL, RIGHT, *ROOM])
    assert local_path(points, 'left', .425, 2., rays=rays)[1]['boundary_observed_ahead_m'] <= 2.+1e-9


def test_unset_rays_leave_existing_output_identical():
    points, rays = np.array(FIXTURE['points']), FIXTURE['rays']
    first, _ = local_path(points, FIXTURE['side'], .425, FIXTURE['horizon_m'])
    second, _ = local_path(points, FIXTURE['side'], .425, FIXTURE['horizon_m'], rays=None)
    assert np.array_equal(first, second)
    # 근거리만 보는 기존 호출은 광선 번호를 줘도 연결할 단절이 없다.
    near = np.linalg.norm(points, axis=1) < 1.5
    assert np.array_equal(local_path(points[near], FIXTURE['side'], .425)[0],
                          local_path(points[near], FIXTURE['side'], .425, rays=np.array(rays)[near])[0])


def test_nonfinite_points_keep_ray_numbers_aligned():
    points, rays = cast([WALL, RIGHT, *ROOM])
    damaged = points.copy()
    damaged[3] = np.nan
    assert (local_path(damaged, 'left', .425, HORIZON, rays=rays)[1]['boundary_observed_ahead_m'] ==
            pytest.approx(local_path(points, 'left', .425, HORIZON, rays=rays)[1]['boundary_observed_ahead_m'], abs=.05))


def test_default_is_off_and_requires_solo_observed_route():
    assert lf.FollowConfig().ray_wall_continuity is False
    assert lf.FollowConfig(**SOLO, solo_route_only=True, ray_wall_continuity=True).ray_wall_continuity is True
    for changes in ({'solo_route_only': False}, {'blend_open_speed_by_length': False, 'preview_turn_speed': False}):
        with pytest.raises(ValueError):
            lf.FollowConfig(**{**SOLO, 'solo_route_only': True, 'ray_wall_continuity': True, **changes})
    with pytest.raises(ValueError):
        lf.FollowConfig(**SOLO, solo_route_only=True, ray_wall_continuity=1)


def test_profile_adds_only_this_option_and_traffic_stays_unconnected():
    profiles = launch_module().SPEED_PROFILES
    assert profiles['g1_open_10kmh_solo_gap05_arc_ray'] == {
        **profiles['g1_open_10kmh_solo_gap05_arc'], 'g1_ray_wall_continuity': True}
    assert profiles['g1_open_10kmh_solo_gap05_arc_preview_bridge_ray'] == {**profiles['g1_open_10kmh_solo_gap05_arc_preview_bridge'], 'g1_ray_wall_continuity': True}
    assert [k for k, v in profiles.items() if 'g1_ray_wall_continuity' in v] == ['g1_open_10kmh_solo_gap05_arc_ray', 'g1_open_10kmh_solo_gap05_arc_preview_bridge_ray']


def long_corridor():
    return [(-1., -.425, 6., -.425), (-1., .425, 6., .425), (-1., -.425, -1., .425), *ROOM]


def test_controller_passes_the_selected_subset_with_ray_numbers():
    solo = {'solo_route_only': True, 'headway_s': .5, 'corridor_arc_agreement': True}
    off = play_scene(long_corridor(), True, speed=1., config_changes=solo)
    on = play_scene(long_corridor(), True, speed=1., config_changes={**solo, 'ray_wall_continuity': True})
    assert [row.reason for row in off] == [row.reason for row in on]
    assert max(row.v for row in on) >= max(row.v for row in off) > 0


def test_linked_option_does_not_fill_missing_front():
    solo = {'solo_route_only': True, 'headway_s': .5, 'corridor_arc_agreement': True, 'ray_wall_continuity': True}
    rows = play_scene(long_corridor(), True, blind=((-.3, .3),), config_changes=solo)
    assert all(row.v == 0 for row in rows)
