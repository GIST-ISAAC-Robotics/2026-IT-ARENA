"""작은 중앙 보정 생략(선택)의 순서 경계와 반례. 직진도 모든 계획·발행 검사를 받으며 주행 성공 판정이 아니다.

10/25 run19 저장 판단에서 중앙 후보는 횡오차 7.56 mm 이하의 보정에 0이 아닌 미세 조향(약 0.0007 rad)을 냈고,
다음 주기 그 이전각 원호가 후방 마스크 꼬리 띠 검사에서 거절되어 전 후보 HOLD가 됐다. 이 옵션은 그 경우의
검사 순서만 바꾼다. 현재 횡오차만 보는 deadband·마지막 조향각 유지·위협 기록 생략은 하지 않는다.
"""
from dataclasses import replace
import math

import numpy as np
import pytest

import arena_autonomy.m3_execution_trial as mx
from arena_autonomy.lateral_intent import ego_state
from test_lateral_intent import GRASS, car
from test_local_follow import history
from test_m3_execution_trial import padding
from test_m3_observed_follow import timed_scene

ON = mx.ExecutionConfig(command_fallback=True, release_to_baseline=True, small_center_straight=True)
OFF = replace(ON, small_center_straight=False)
LIMIT = GRASS.budget.tracking_straight_m


def road_path(d0=0., psi=0., radius=None, start=-.3, end=1.6, n=80):
    """도로 기준 경로를 자차 좌표로 옮긴다. 자차는 도로 좌표 (0, d0)에서 방향 오차 psi. radius는 왼쪽 곡선."""
    s = np.linspace(start, end, n)
    if radius is None:
        road = np.column_stack((s, np.zeros_like(s)))
    else:
        road = np.column_stack((radius*np.sin(s/radius), radius*(1-np.cos(s/radius))))
    c, sn = math.cos(-psi), math.sin(-psi)
    return (road-[0., d0])@np.array([[c, -sn], [sn, c]]).T


def small(path):
    frame = mx.ExecutionFrame(path)
    state = ego_state(frame, GRASS)
    return mx.small_center_check(mx.straight_candidate(frame, state, GRASS), GRASS), state


def test_option_is_typed_default_off_and_requires_observed_release():
    assert mx.ExecutionConfig().small_center_straight is False
    with pytest.raises(ValueError):
        mx.ExecutionConfig(release_to_baseline=True, small_center_straight=1)
    with pytest.raises(ValueError, match='release'):
        mx.ExecutionConfig(small_center_straight=True)


@pytest.mark.parametrize('name, d0, psi, radius, expected', [
    ('run19_like_small', .004, 0., None, True),
    ('small_offset_and_heading', .004, .005, None, True),
    ('run17_like_90mm_return', .09, 0., None, False),
    ('heading_only', 0., .03, None, False),          # 현재 횡오차 0이어도 1.2 m 앞 편차 약 36 mm
    ('tight_curve', 0., 0., 1.2, False),             # 곡선 구간: 직진 우선 없음
    ('gentle_curve', 0., 0., 4.0, False),            # 곡률 임계 미만이어도 투영 편차가 예산 초과
])
def test_prediction_uses_heading_and_curvature_not_only_current_offset(name, d0, psi, radius, expected):
    check, state = small(road_path(d0, psi, radius))
    assert check['ok'] is expected, (name, check)
    assert check['limit_m'] == LIMIT and check['planned_length_m'] >= GRASS.min_ahead_m
    if name in ('heading_only', 'tight_curve', 'gentle_curve'):
        # 현재 후륜축 횡오차만 보는 deadband라면 통과했을 반례다.
        assert abs(state['d_rear0']) <= LIMIT and check['max_abs_offset_m'] > LIMIT
    if name == 'tight_curve':
        assert check['curved']


def test_repeated_straight_cannot_accumulate_drift_beyond_budget():
    """매 판단 현재 자세에서 다시 예측한다. 계속 직진한 누적 편차는 예산을 넘기 전에 중앙 우선으로 돌아간다."""
    psi, step = .008, .05
    flipped = None
    for k in range(80):
        x = k*step
        lateral = x*math.sin(psi)              # 도로 기준 실제 횡편차(직진 누적)
        path = road_path(lateral, psi, start=-.3-x, end=1.6)
        check, state = small(path)
        if check['ok']:
            # 직진 우선이 남아 있는 동안 현재·예측 편차 모두 예산 안이다.
            assert abs(state['d_rear0']) <= LIMIT and check['max_abs_offset_m'] <= LIMIT
        elif flipped is None:
            flipped = lateral
    assert flipped is not None and flipped < LIMIT
    # 같은 직진을 계속 유지했다면(이전 선택 고착/마지막 조향 유지) 3 m 뒤 편차는 예산을 넘는다.
    assert 3.0*math.sin(psi) > LIMIT


def active(execution, d0):
    trial = mx.ExecutionLateralTrial(GRASS, execution)
    scene, path = timed_scene(.3, d0)
    trial.step(scene, [], path, .35, padding(scene), history=history())
    trial.active, trial.clear_fresh = True, 1     # 위협 소멸 뒤 fresh 1회 경과(다음 fresh에서 중앙 촉발)
    return trial


def step_order(execution, d0, accept=lambda name: True):
    trial = active(execution, d0)
    scene, path = timed_scene(.4, d0)
    seen = []

    def check(name, candidate):
        seen.append(name)
        ok = accept(name)
        return {'accepted': ok, 'reason': None if ok else 'm3_command_unobserved'}

    return trial.step(scene, [], path, .35, padding(scene), check, history=history()), seen, trial


def test_small_center_phase_checks_straight_first_and_off_is_unchanged():
    on, seen_on, trial = step_order(ON, -.004)
    off, seen_off, _ = step_order(OFF, -.004)
    # 합성 직선로에서도 4 mm 중앙 보정의 계획 원호는 후방 마스크 꼬리 띠 때문에 미지지다(§8.10.2와 같은 원인).
    assert list(off['checks'])[0] == 'center' and 'small_center' not in off
    assert 'current_sweep_unobserved' in off['checks']['center']['reasons']
    assert list(on['checks']) == ['straight'] and seen_on == ['straight']
    assert on['selected'] == 'straight' and on['small_center']['ok']
    assert on['checks']['straight']['eligible_for_trial'] and trial.active   # 시험 종료 아님(관측 복귀만 해제)


def test_large_return_keeps_center_first_with_identical_selection():
    on, seen_on, _ = step_order(ON, -.09, accept=lambda name: name != 'center')
    off, seen_off, _ = step_order(OFF, -.09, accept=lambda name: name != 'center')
    assert not on['small_center']['ok']
    assert list(on['checks']) == list(off['checks']) and list(on['checks'])[0] == 'center'
    assert seen_on == seen_off and on['selected'] == off['selected'] and on['mode'] == off['mode']
    assert set(on) - set(off) == {'small_center'}


def test_rejected_straight_command_falls_back_to_existing_order():
    out, seen, _ = step_order(ON, -.004, accept=lambda name: name != 'straight')
    off, _, _ = step_order(OFF, -.004)
    assert seen[0] == 'straight' and out['selected'] != 'straight'
    assert list(out['checks'])[1:] == list(off['checks'])       # 나머지는 기존 순서(center → hold)
    assert out['selected'] == off['selected']
    assert not out['checks']['straight']['command']['accepted']


def test_unknown_ahead_is_not_exempted_for_small_correction():
    trial = active(ON, -.004)
    blind = lambda deg: np.abs(deg) < 12.     # 전방 무응답(unknown)
    scene, path = timed_scene(.4, -.004, dropout=blind)
    out = trial.step(scene, [], path, .35, padding(scene), lambda *a: {'accepted': True, 'reason': None},
                     history=history())
    assert out['small_center']['ok']
    assert 'current_sweep_unobserved' in out['checks']['straight']['reasons']
    assert out['selected'] != 'straight'


def passing(d0, n=13, lateral=.26):
    """왼쪽 옆차가 정지한 자차 기준 0.45 m/s로 앞서 나간다. 합성 벽 반환과 떨어진 compact 반환이 되도록
    run19(0.32 m)보다 안쪽에 둔다. 정답 actor 입력이 아니라 합성 광선 장면이다."""
    return [(round(1.0+.1*k, 3), [car(.30+.45*.1*k, lateral, d0)]) for k in range(n)]


def run_trial(execution, sequence, d0, mutate=None):
    trial = mx.ExecutionLateralTrial(GRASS, execution)
    h = history()
    outs = []
    for t, extra in sequence:
        scene, path = timed_scene(t, d0, extra=extra)
        if mutate is not None:
            scene = mutate(t, scene)
        outs.append(trial.step(scene, [], path, .35, padding(scene), history=h))
    return trial, outs


def test_threat_memory_and_release_gate_are_identical_on_and_off():
    d0 = -.004
    on_trial, on = run_trial(ON, passing(d0), d0)
    off_trial, off = run_trial(OFF, passing(d0), d0)
    assert any(o['threats']['left']['present'] for o in on)
    for a, b in zip(on, off):
        assert a['threats'] == b['threats']
        assert a.get('release_gate') == b.get('release_gate')
    assert set(on_trial.records) == set(off_trial.records)
    # 위협이 보이는 동안에는 중앙 단계가 아니므로 순서 변경이 없다.
    for a, b in zip(on, off):
        if a['threats']['left']['present']:
            assert 'small_center' not in a and a['mode'] == b['mode'] and a['selected'] == b['selected']
    assert any(o.get('small_center') for o in on)


def test_epoch_loss_still_holds_release_history_with_option():
    d0 = -.004

    def epoch_after(t, scene):
        if t < 1.25:
            return scene
        return replace(scene, source_epoch=1, rays=tuple(replace(r, source_epoch=1) for r in scene.rays))

    trial, outs = run_trial(ON, passing(d0, n=5), d0, epoch_after)
    assert outs[-1]['mode'] == 'HOLD_RELEASE_HISTORY' and trial.release_history_invalidated


def test_second_side_threat_keeps_existing_branch_and_both_records():
    """한쪽 옆차가 앞으로 지나간 뒤 다른 쪽 뒤옆에 새 물체가 보이면 중앙 단계가 아니다(복수 물체)."""
    d0 = -.004
    right = car(-.15, -.26, d0)
    seq = [(t, extra+([right] if t >= 1.5 else [])) for t, extra in passing(d0)]
    on_trial, on = run_trial(ON, seq, d0)
    off_trial, off = run_trial(OFF, seq, d0)
    assert any(o['threats']['right']['present'] for o in on)
    for a, b in zip(on, off):
        assert a.get('release_gate') == b.get('release_gate') and a['threats'] == b['threats']
        if a['threats']['right']['present'] or a['threats']['left']['present']:
            assert 'small_center' not in a and a['mode'] == b['mode'] and a['selected'] == b['selected']
    assert set(on_trial.records) == set(off_trial.records) == {'left', 'right'}
    assert on_trial.active and not any(o.get('released_to_baseline') for o in on)


@pytest.mark.parametrize('lead_x, reason', [(.60, 'm3_execution_trial'), (.45, 'm3_gap_hold')])
def test_stopped_lead_after_pass_keeps_same_longitudinal_limit(lead_x, reason):
    """옆차 통과 뒤 같은 차로 앞에 정지한 앞차(재정지). 직진 우선이어도 종방향 간격·제동은 기존과 같다."""
    from test_m3_execution_trial import ctl
    d0 = -.004
    seq = [(round(.4+.1*k, 3), []) for k in range(3)]
    for k in range(16):
        extra = [car(.30+.045*k, .26, d0)] if k < 5 else []
        seq.append((round(.7+.1*k, 3), extra+([car(lead_x, 0., d0)] if k >= 6 else [])))
    results = {}
    for name, execution in (('on', ON), ('off', OFF)):
        follow, h, rows = ctl(execution), history(v=0.), []
        for t, extra in seq:
            scene, _ = timed_scene(t, d0, extra=extra)
            rows.append((t, follow.command(scene, h, 0., 'left')))
        results[name] = rows
    late = [(a, b) for (t, a), (_, b) in zip(results['on'], results['off']) if t >= 1.4]
    assert any((a[2].get('m3_lateral') or {}).get('mode') == 'STRAIGHT_TRIAL' for a, _ in late)
    for a, b in late:
        assert a[0] == b[0] and a[2]['reason'] == b[2]['reason'] == reason
        assert a[2]['swept_clearance_m'] == b[2]['swept_clearance_m']
