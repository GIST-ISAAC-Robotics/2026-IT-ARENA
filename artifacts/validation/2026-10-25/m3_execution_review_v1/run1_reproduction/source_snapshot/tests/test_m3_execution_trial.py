"""M3 유한 실행·제동 전이 후보의 순수 반례. 운영 연결·폐루프·실차 안전 증명이 아니다."""
from dataclasses import replace
import math

import numpy as np
import pytest

import arena_autonomy.m3_execution_trial as mx
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.lateral_intent import (PathFrame, ObservationGrid, cell_coordinates, ego_state, offset_candidate,
                                           side_threats, committed_progress_m)
from arena_autonomy.m3_observed_follow import command_arc, command_observation
from arena_autonomy.observed_lateral_trial import observed_candidate
from arena_autonomy.occupancy_timing import per_ray_padding
from test_lateral_intent import GRASS, straight, car, box
from test_local_follow import history
from test_m3_observed_follow import timed_scene

EXEC = mx.ExecutionConfig()


def ctl(execution=None):
    return mx.M3ExecutionFollow(execution=execution, allow_simulation_trial=True)


def padding(scene):
    return per_ray_padding(scene, .055, .5)


def stub(controller, path, selected='hold', reason='follow_gap_hold'):
    """격리된 출력/관문 시험에서만 기본 G1과 후보 선택을 고정한다. 관측·점유 검사는 실제 함수다."""
    controller.base.command = lambda *a, **k: (0., 0., {'reason': reason, 'behavior': 'HOLD',
                                                        'path_points': path.tolist(), 'boundary_age_s': .1})
    controller.lateral.active = True

    def selected_step(*args):
        controller.lateral.active = True
        return {'mode': selected.upper()+'_TRIAL', 'path': path, 'selected': selected}
    controller.lateral.step = selected_step


def test_explicit_selection_and_typed_options():
    with pytest.raises(ValueError):
        mx.M3ExecutionFollow()
    with pytest.raises(ValueError):
        mx.ExecutionConfig(masked_history_support=1)
    with pytest.raises(ValueError):
        mx.ExecutionConfig(deceleration_mps2=0.)
    with pytest.raises(ValueError):
        mx.M3ExecutionFollow(FollowConfig(), allow_simulation_trial=True)


def test_solo_passthrough_matches_local_follow_without_building_side_grid(monkeypatch):
    monkeypatch.setattr(mx, 'ObservationGrid', lambda *a: pytest.fail('solo input built the side grid'))
    follow, baseline = ctl(), LocalFollow(FollowConfig(occupancy_timing_mode='per_ray_candidate'))
    h = history()
    for now in (.4, .5, .6, .7, .8):
        s, _ = timed_scene(now)
        expected = baseline.command(s, h, 0., 'left')
        got = follow.command(s, h, 0., 'left')
        assert got[:2] == expected[:2]
        assert (got[2].get('m3_lateral') or {}).get('mode') in (None, 'BASELINE')
        assert not got[2]['runtime_authorized'] and got[2]['hidden_future_risk_unresolved']
    assert got[0] > 0 and not follow.lateral.active


@pytest.mark.parametrize('extra', [[], [car(-.40, .13, 0.)], [car(-.40, -.13, 0.)], [car(-.1, .21, 0.)],
                                   [car(-.40, .13, 0.), car(-.40, -.13, 0.)]])
def test_side_presence_precheck_equals_side_threats(extra):
    scene, path = straight(0., extra=extra)
    frame = PathFrame(path)
    state = ego_state(frame, GRASS)
    full = ObservationGrid(scene, GRASS)
    s, l = cell_coordinates(full, frame)
    threats = side_threats(frame, state, full, GRASS, s, l, full.observed_free())
    light = mx.side_points(frame, state, mx.ReturnGrid(scene, GRASS), GRASS)
    assert light == (threats['left']['points'], threats['right']['points'])
    if not any(light):
        assert side_threats(frame, state, mx.ReturnGrid(scene, GRASS), GRASS, None, None, None) == threats


def test_candidate_is_anchored_at_actual_body_and_large_mismatch_is_invalid():
    # 곡선 경로에서 경로 좌표 왕복(구간 법선 투영/보간 법선 재구성)은 첫 자세를 μm 단위로 어긋나게 한다.
    t = np.linspace(-.3, 1.5, 60)
    radius = 1.2
    path = np.column_stack((radius*np.sin(t/radius), -.09+radius*(1-np.cos(t/radius))))
    frame = PathFrame(path)
    state = ego_state(frame, GRASS)
    raw = offset_candidate(frame, state, state['d0'], GRASS, 'hold')
    assert 0 < np.max(np.abs(raw['poses'][0])) < mx.START_TOLERANCE_M
    fixed = mx.anchored(raw)
    assert np.array_equal(fixed['poses'][0], np.zeros(3)) and fixed['valid']
    rear = fixed['poses'][:, :2]-GRASS.wheelbase_m/2*np.column_stack((np.cos(fixed['poses'][:, 2]),
                                                                    np.sin(fixed['poses'][:, 2])))
    np.testing.assert_allclose(rear, fixed['path'], atol=1e-12)
    shifted = {**raw, 'poses': raw['poses']+[.01, 0., 0.]}
    bad = mx.anchored(shifted)
    assert not bad['valid'] and 'start_pose_mismatch' in bad['problems']


def test_masked_only_never_includes_unknown_or_occluded_space():
    scene, _ = straight(0., extra=[box(.5, .7, -.05, .05)], dropout=lambda a: (a > 60.) & (a < 62.))
    pad = mx.ray_padding(scene, GRASS, .5)
    tail = np.array([[-.095, .0755], [-.095, -.0755]])           # 후방 마스크 쪽 꼬리 흔들림 띠
    dropout = np.array([[.06+.3*math.cos(math.radians(61)), .3*math.sin(math.radians(61))]])
    occluded = np.array([[.8, 0.]])                              # 현재 반환 뒤(가림)
    assert mx.masked_only(scene, tail, pad).all()
    assert not mx.masked_only(scene, dropout, pad).any()
    assert not mx.masked_only(scene, occluded, pad).any()


def two_scans(v, extra_now=(), extra_old=(), dropout=None):
    """같은 정지 물체를 이전(0.9 s)/현재(1.0 s) 차체 좌표에서 본 두 장면과 운동 이력."""
    shift = v*.1
    old, _ = timed_scene(.9, extra=[p+[shift, 0.] for p in extra_old], dropout=dropout)
    now, path = timed_scene(1., extra=list(extra_now), dropout=dropout)
    h = history(v=v)
    cache = mx.SceneHistory()
    cache.update(old, h)
    return now, path, cache.update(now, h)


def test_tail_swing_band_needs_history_after_motion_and_no_adjacent_object():
    tail = np.array([[-.095, .0755], [-.09, .0752], [-.095, -.0755]])
    strict = mx.ExecutionConfig(masked_history_support=False)
    # 정지 중에는 이전 장면도 같은 마스크라 지지되지 않는다.
    now, _, frames = two_scans(0.)
    assert len(frames) == 1
    assert not mx.support(now, frames, tail, GRASS, EXEC)['supported']
    # 7 cm 이동 뒤 이전 장면은 같은 띠를 실제 광선으로 봤다. 기본 꺼짐(strict)에서는 쓰지 않는다.
    now, _, frames = two_scans(.7)
    moved = mx.support(now, frames, tail, GRASS, EXEC)
    assert moved['supported'] and moved['history_supported'] == moved['masked_candidates'] > 0
    assert not mx.support(now, frames, tail, GRASS, strict)['supported']
    # 이전 장면에서 띠 바로 옆에 물체가 보였다면 이동 상계 여유 안이라 지지하지 않는다.
    adjacent = box(-.30, -.10, .085, .235)
    now, _, frames = two_scans(.7, extra_now=[adjacent], extra_old=[adjacent])
    assert not mx.support(now, frames, tail[:2], GRASS, EXEC)['supported']


def test_scene_history_ignores_same_scan_and_clears_on_regression_or_epoch():
    h = history(v=.5)
    cache = mx.SceneHistory()
    s1, _ = timed_scene(.9)
    assert cache.update(s1, h) == []
    assert cache.update(replace(s1, t_ref_s=.92), h) == []       # 같은 원본 스캔만 반복
    s2, _ = timed_scene(1.)
    assert len(cache.update(s2, h)) == 1
    assert cache.update(s1, h) == [] and not cache.scenes   # 원본 스캔·기준 시각 역행
    cache.update(s1, h)
    epoch = replace(s2, source_epoch=1, rays=tuple(replace(r, source_epoch=1) for r in s2.rays))
    assert cache.update(epoch, h) == [] and len(cache.scenes) == 1


def obstacle_on_arc(angle, progress, outward=.03):
    poses = command_arc(angle, progress, GRASS)
    x, y, yaw = poses[-1]
    corner = np.array([x, y])+[math.cos(yaw)*.1-math.sin(yaw)*.075, math.sin(yaw)*.1+math.cos(yaw)*.075]
    center = corner+outward*np.array([-math.sin(yaw), math.cos(yaw)])
    return box(center[0]-.01, center[0]+.01, center[1]-.01, center[1]+.01)


def test_envelope_checks_issued_and_previous_arcs_only_until_stop():
    obstacle = obstacle_on_arc(.3, .28)
    scene, path, frames = two_scans(.7, extra_now=[obstacle], extra_old=[obstacle])
    distance = mx.envelope_distance(.35, .1, .02, EXEC)
    assert distance == pytest.approx(.35*(.02+.1+.1)+.35**2/4)
    near = mx.arc_checks(scene, frames, path, (0., .3), distance, GRASS, EXEC)
    straight_only = mx.arc_checks(scene, frames, path, (0., 0.), distance, GRASS, EXEC)
    far = mx.arc_checks(scene, frames, path, (0., .3), .30, GRASS, EXEC)
    assert straight_only['observation_ok'] and near['observation_ok']
    assert near['observation']['history_supported'] > 0          # 0.3 rad 꼬리 흔들림 띠는 이력으로만 지지
    assert not far['observation_ok']
    # 이력 없이(정지 출발과 같은 조건)는 같은 조향의 꼬리 흔들림 띠를 지지하지 않는다.
    assert not mx.arc_checks(scene, [], path, (0., .3), distance, GRASS, EXEC)['observation_ok']
    # 동결 판본은 0.5 s 확약 동안 같은 각을 유지한다고 보고 같은 물체를 거절했다(기록용 대조).
    assert not command_observation(scene, .35, 0., .3, GRASS)['supported']
    # 이전각 원호의 정지 구간 안에서만 지나가는 얇은 무응답 방향은 이번 발행각(직진)과 무관하게 거절한다.
    scene2, path2, frames2 = two_scans(.7, dropout=lambda a: (a > 69.5) & (a < 70.5))
    assert not mx.arc_checks(scene2, frames2, path2, (0., .3), distance, GRASS, EXEC)['observation_ok']
    assert mx.arc_checks(scene2, frames2, path2, (0., 0.), distance, GRASS, EXEC)['observation_ok']


def trial_context(scene, path):
    frame = PathFrame(path)
    state = ego_state(frame, GRASS)
    grid = mx.ReturnGrid(scene, GRASS)
    from arena_autonomy.lateral_intent import corridor_reference
    corridor = corridor_reference(frame, grid.returns, state['s0']-.6, state['s0']+1.2, GRASS)
    return frame, state, {'frame': frame, 'grid': grid, 'corridor': corridor,
                          'age': scene.t_ref_s-scene.source_first_s, 'follow_padding': padding(scene)}


def test_front_object_is_deferred_to_gap_logic_but_side_object_is_not():
    scene, path = straight(0., extra=[box(.42, .62, -.075, .075)])
    frame, state, context = trial_context(scene, path)
    hold = mx.anchored(offset_candidate(frame, state, state['d0'], GRASS, 'hold'))
    frozen = observed_candidate(scene, offset_candidate(frame, state, state['d0'], GRASS, 'hold'), context, GRASS, .35)
    new = mx.plan_check(scene, [], hold, context, GRASS, EXEC, .35)
    assert 'current_sweep_unobserved' in frozen['reasons']
    assert new['eligible_for_trial']
    assert new['in_path_free_progress_m'] < committed_progress_m(.35, context['age'], GRASS)
    beside, path = straight(0., extra=[box(-.05, .15, .10, .25)])
    frame, state, context = trial_context(beside, path)
    hold = mx.anchored(offset_candidate(frame, state, state['d0'], GRASS, 'hold'))
    out = mx.plan_check(beside, [], hold, context, GRASS, EXEC, .35)
    assert not out['eligible_for_trial'] and 'lateral_clearance_short' in out['reasons']


@pytest.mark.parametrize('sign', [1., -1.])
def test_single_side_threat_only_yields_away_and_never_centers(sign):
    d0 = -sign*.09
    scene, path = straight(d0, extra=[car(-.40, sign*.13, d0)])
    trial = mx.ExecutionLateralTrial(GRASS, EXEC)
    out = trial.step(scene, [], path, .35, padding(scene))
    side = 'left' if sign > 0 else 'right'
    assert out['threats'][side]['present'] and not out['runtime_authorized']
    assert 'center' not in out['checks']
    if 'yield' in out['checks']:
        assert sign*(out['checks']['yield']['target_m']-d0) < 0
    if out['selected'] in ('yield', 'hold'):
        target = out['checks'][out['selected']]['target_m']
        assert sign*(target-d0) <= 1e-9


def test_both_sides_occupied_offers_no_lateral_offset():
    scene, path = straight(0., extra=[car(-.40, .16, 0.), car(-.40, -.16, 0.)])
    out = mx.ExecutionLateralTrial(GRASS, EXEC).step(scene, [], path, .35, padding(scene))
    assert out['threats']['left']['present'] and out['threats']['right']['present']
    assert set(out['checks']) <= {'hold', 'straight'} and 'yield' not in out['checks']


def test_current_lateral_budget_violation_blocks_every_candidate():
    # 이미 옆차와 4 cm인 자세에서는 모든 후보의 첫 자세가 측방 예산을 못 지킨다. 간격을 줄여 통과시키지 않는다.
    scene, path = straight(0., extra=[car(0., .19, 0.)])
    out = mx.ExecutionLateralTrial(GRASS, EXEC).step(scene, [], path, .35, padding(scene))
    assert out['mode'] == 'HOLD_STOP_TRIAL' and out['path'] is None
    assert all('lateral_clearance_short' in c['reasons'] for c in out['checks'].values())


def test_path_loss_keeps_intent_and_holds_previous_steering():
    follow, s = ctl(), timed_scene(.4)[0]
    follow.base.command = lambda *a, **k: (.2, -.05, {'reason': 'local_follow', 'behavior': 'FOLLOW',
                                                      'path_points': []})
    follow.lateral.active = True
    follow.lateral.step = lambda *a: {'mode': 'HOLD_PATH', 'path': None, 'selected': None}
    out = follow.command(s, history(), .35, 'left', previous_steering=.08)
    assert out[:2] == (0., .08) and out[2]['reason'] == 'm3_candidate_unavailable'


@pytest.mark.parametrize('reason', ['follow_reverse_motion', 'follow_front_unobserved', 'follow_boundary_unconfirmed',
                                    'follow_scene_stale'])
def test_existing_g1_gates_pass_through(reason):
    follow, s = ctl(), timed_scene(.4)[0]
    follow.base.command = lambda *a, **k: (0., 0., {'reason': reason, 'behavior': 'HOLD'})
    follow.lateral.step = lambda *a: pytest.fail('gated input reached lateral planner')
    out = follow.command(s, history(), -.01 if reason == 'follow_reverse_motion' else 0., 'left')
    assert out[:2] == (0., 0.) and not out[2]['m3_selected'] and follow.held


def test_epoch_or_time_regression_resets_trial_and_history():
    follow = ctl()
    _, path = timed_scene(.4)
    stub(follow, path)
    follow.command(timed_scene(.4)[0], history(), 0., 'left')
    follow.command(timed_scene(.5)[0], history(), 0., 'left')
    assert follow.recent.scenes
    s = timed_scene(.6)[0]
    epoch = replace(s, source_epoch=1, rays=tuple(replace(r, source_epoch=1) for r in s.rays))
    out = follow.command(epoch, history(), 0., 'left')
    assert len(follow.recent.scenes) == 1 and out[0] == 0 and follow.held


def test_resume_requires_two_distinct_scans_not_repeated_same_scan():
    follow, h = ctl(), history()
    _, path = timed_scene(.4)
    stub(follow, path)
    for now in (.4, .42, .44, .46):
        out = follow.command(replace(timed_scene(.4)[0], t_ref_s=now), h, 0., 'left')
        assert out[0] == 0
    assert out[2]['reason'] == 'm3_release_wait'
    out = follow.command(timed_scene(.5)[0], h, 0., 'left')
    assert 0 < out[0] <= .35 and out[2]['reason'] == 'm3_execution_trial'
    env = out[2]['m3_envelope']
    assert env['distance_m'] == pytest.approx(mx.envelope_distance(out[2]['envelope_speed_mps'], .1, .02, EXEC))
    assert not env['continuous_sweep_certified'] and not out[2]['body_standstill_verified']


def test_straight_start_presteers_only_while_stopped_and_never_moves_first():
    follow, h = ctl(mx.ExecutionConfig(masked_history_support=False)), history()
    _, path = timed_scene(.4)
    stub(follow, path, selected='straight')
    out = follow.command(timed_scene(.4)[0], h, .2, 'left', previous_steering=.2)
    assert out[:2] == (0., .2) and out[2]['reason'] == 'm3_command_unobserved'   # 이동 중엔 정렬하지 않음
    previous, now, reasons = .2, .5, []
    for _ in range(8):
        out = follow.command(timed_scene(now)[0], h, 0., 'left', previous_steering=previous)
        reasons.append(out[2]['reason'])
        assert out[0] == 0 or previous == 0.
        assert abs(out[1]-previous) <= 3*.02+1e-12
        previous, now = out[1], now+.1
        if out[0] > 0:
            break
    assert reasons[0] == 'm3_straight_presteer' and 'm3_release_wait' in reasons
    assert out[0] > 0 and out[1] == 0. and out[2]['m3_candidate'] == 'straight'


def test_presteer_rejected_when_steered_wheel_outline_is_unobserved():
    # 앞바퀴 바로 옆 물체(최대 조향 바퀴 돌출 + 점유 여유 안)는 정지 중 조향 정렬도 막는다.
    scene, _ = timed_scene(.4, extra=[box(.03, .10, .085, .10)])
    out = mx.presteer_support(scene, [], (.37, .31), GRASS, EXEC)
    assert out['queries'] > 0 and not out['supported']


@pytest.mark.parametrize('lead_rear', [.14, .45, .9])
def test_front_car_with_side_car_never_faster_than_g1_and_stops_when_close(lead_rear):
    # 실제 G1 경계·점유 경로를 쓴다. 앞차가 가까우면 기존 간격 HOLD, 멀어도 G1 목표보다 빠르지 않다.
    extra = [car(-.40, .13, -.09), box(lead_rear, lead_rear+.2, -.075, .075)]
    follow, baseline, h = ctl(), LocalFollow(FollowConfig(occupancy_timing_mode='per_ray_candidate')), history()
    modes = set()
    for now in (.4, .5, .6, .7, .8):
        s, _ = timed_scene(now, d0=-.09, extra=extra)
        base = baseline.command(s, h, 0., 'left')
        out = follow.command(s, h, 0., 'left')
        modes.add((out[2].get('m3_lateral') or {}).get('mode'))
        if out[2]['m3_selected']:
            assert out[0] <= .35 and (out[0] <= base[0]+1e-12 or base[2]['reason'] == 'follow_release_wait')
        if lead_rear < .2:
            assert out[0] == 0.
    assert lead_rear < .2 or modes-{None, 'BASELINE'}     # 아주 가까운 앞차는 G1 관문/간격이 먼저 정지
