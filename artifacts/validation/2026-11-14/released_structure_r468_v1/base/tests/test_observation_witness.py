"""평가 전용 witness 분해의 고정 반례. 운영 관측 판단을 바꾸지 않는다."""
import math
from dataclasses import replace

import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.observation_support import sweep_queries
from observation_witness import distance_class, distance_parts, entry_travel, failure_witnesses
from test_corridor_visibility import scene
from test_linear_observation import unknown_at

POSE = (.06, 0., 0.)


@pytest.mark.parametrize('steer', [-.3, -.01, 0., 1e-4, .2])
def test_existing_queries_lie_in_independent_icr_sweep(steer):
    cfg = FollowConfig()
    queries, _ = sweep_queries(unknown_at(90), steer, .27, cfg)
    entries = [entry_travel(q, steer, .27, cfg) for q in queries]
    assert queries.size and all(e is not None and e <= .27+1e-9 for e in entries)


def test_left_right_wedge_geometry_matches_but_sequential_ray_ages_differ():
    cfg = FollowConfig()
    left = failure_witnesses([unknown_at(90)], 0., .2, .06, .06, cfg, POSE)
    right = failure_witnesses([unknown_at(-90)], 0., .2, -.06, -.06, cfg, POSE)
    assert left[2] and right[2] and not left[0]['supported'] and not right[0]['supported']
    # +90도는 index 375, -90도는 125: 같은 회전에서 취득 시각이 50 ms 다르다.
    # 정적 쐐기 기하는 대칭이지만 이동 가능 거리까지 같은 표본은 아니다.
    side_rows = lambda rows: [r for r in rows if r.get('ray_status') == 'unknown']
    assert {r['role'] for rows in (left[1], right[1]) for r in side_rows(rows)} == {'side_inner'}
    # 기존 fixture에는 후방 60도 마스크도 있다. 바깥 뒤 모서리 swing-out을
    # 측방 무응답 실패와 혼합하지 않되, 그 실패 자체는 결과에 계속 남아야 한다.
    for rows in (left[1], right[1]):
        rear = [r for r in rows if r.get('ray_status') == 'masked']
        assert rear and all(r['role'] == 'side_outer_behind_rear_axle' for r in rear)
    wedge = lambda rows: {tuple(np.round(r['query_xy_m'], 9)) for r in rows
                          if r['cause'] == 'unknown_wedge'}
    assert wedge(left[1]) == {(x, -y) for x, y in wedge(right[1])}
    left_reach = [r for r in left[1] if r['cause'] == 'lateral_reach']
    right_reach = [r for r in right[1] if r['cause'] == 'lateral_reach']
    assert left_reach and right_reach
    assert {round(r['motion_budget_m'], 9) for r in left_reach} == {.0125}
    assert {round(r['motion_budget_m'], 9) for r in right_reach} == {.0375}
    assert len(right_reach) > len(left_reach)


def test_exact_straight_passes_but_tiny_inner_curvature_enters_unknown_wedge():
    cfg = FollowConfig()
    straight = failure_witnesses([unknown_at(90)], 0., .2, 0., 0., cfg, POSE)
    assert straight[0]['supported'] and straight[1] == []
    result, rows, consistent = failure_witnesses([unknown_at(90)], 0., .2, 1e-4, 1e-4, cfg, POSE)
    parts = distance_parts(0., .2, .1, cfg)
    assert consistent and not result['supported'] and rows
    side = [r for r in rows if r.get('ray_status') == 'unknown']
    assert side and all(r['role'] == 'side_inner' and r['cause'] in
                        ('unknown_wedge', 'unknown_bracket', 'lateral_reach') for r in side)
    assert any(r['cause'] == 'unknown_wedge' for r in side)
    assert min(r['entry_travel_m'] for r in side) <= parts['future_motion_m']
    assert {r['ray_index'] for r in side} == {375}
    assert any(r.get('ray_status') == 'masked' for r in rows)


@pytest.mark.parametrize('angle', [-90, 90])
def test_lateral_witness_has_a_physical_start_outside_body_and_in_time_budget(angle):
    cfg = FollowConfig()
    observation = unknown_at(angle)
    _, rows, consistent = failure_witnesses([observation], 0., .2, math.copysign(.06, angle),
                                           math.copysign(.06, angle), cfg, POSE)
    reached = [r for r in rows if r['cause'] == 'lateral_reach']
    assert consistent and reached
    for row in reached:
        assert row['lateral_witness_found']
        start = row['possible_start_xy_m']
        assert abs(start[0]) >= cfg.length_m/2-1e-12 or abs(start[1]) >= cfg.width_m/2-1e-12
        assert math.dist(start, row['query_xy_m']) == pytest.approx(row['separation_m'], abs=1e-12)
        assert row['separation_m'] <= row['motion_budget_m']+1e-12
        assert row['motion_budget_m'] == pytest.approx(
            cfg.object_motion_bound_mps*(observation.t_ref_s-row['ray_acquired_at_s']))


@pytest.mark.parametrize('bad', ['front_unknown', 'stale', 'index_order', 'negative_speed', 'nan'])
def test_invalid_or_front_missing_inputs_keep_original_rejection(bad):
    cfg = FollowConfig()
    original = unknown_at(0) if bad == 'front_unknown' else scene()
    speed = 0.
    if bad == 'stale':
        original = replace(original, t_ref_s=.61)
    if bad == 'index_order':
        original = replace(original, rays=tuple(replace(r, source_index=499-r.source_index)
                                                for r in original.rays))
    if bad == 'negative_speed':
        speed = -.01
    if bad == 'nan':
        speed = math.nan
    result, rows, consistent = failure_witnesses([original], speed, .2, .06, 0., cfg, POSE)
    assert not result['supported'] and consistent and rows == []


def test_distance_parts_match_existing_requirement_and_classes():
    cfg = FollowConfig()
    parts = distance_parts(.1945, .2, .118, cfg)
    assert parts['total_m'] == pytest.approx(.2736)
    assert distance_class(.03, parts) == 'future_motion'
    assert distance_class(.05, parts) == 'age_term'
    assert distance_class(.1, parts) == 'braking_margin'
    assert distance_class(.2, parts) == 'standstill_gap'
    assert distance_class(None, parts) == 'outside_independent_sweep'


def test_current_return_overrides_before_witness():
    cfg = FollowConfig()
    result, rows, consistent = failure_witnesses([scene(.25)], 0., .2, 0., 0., cfg, POSE)
    assert result['reason'] == 'current_occupancy_overrides_history' and rows == [] and consistent
