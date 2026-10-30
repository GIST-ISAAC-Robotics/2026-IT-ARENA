"""후방 소실 직후 차선 유지 후보의 상태 계약 반례. 주입 view는 분기만 검증하며 센서 인식 증명이 아니다."""
import math
from types import SimpleNamespace

import pytest

import numpy as np

import arena_autonomy.m3_execution_trial as mx
import arena_autonomy.pass_release_trial as ps
import candidate as lk
import parallel_speed
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.traffic_path_recovery_trial import SuspendedPassageSelector

CHAIN = dict(partial_observation_policy='bounded_local_v4', boundary_evidence_mode='corner_distance_candidate',
             occupancy_timing_mode='per_ray_candidate', parallel_hold_trial=True, pass_release_trial=True,
             guard_curve_trial=True, traffic_recovery_trial=True, traffic_path_recovery_trial=True,
             corridor_fallback_enabled=True)
PATH = [[x/10, 0.] for x in range(11)]


def config(**kwargs):
    return FollowConfig(**{**CHAIN, 'rear_lane_keep_trial': True, **kwargs})


def baseline(speed=1., angle=0., reason='local_follow'):
    return speed, angle, {'reason': reason, 'path_points': PATH, 'swept_clearance_m': 1.4}


def parallel(speed=.35, padding=.10):
    return speed, 0., {'reason': 'parallel_hold_follow' if speed > 0 else 'parallel_hold_unchanged_gap_hold',
                       'parallel_hold_trial': {'path_points': PATH, 'maximum_padding_m': padding,
                                               'clearance_m': 1.4}}


def view(present=False, rear=-.2):
    # GUARD 곡선 후보가 진입 기준을 계산하므로 기존 G1 경로와 같은 실제 경로 좌표를 둔다.
    return {'frame': mx.ExecutionFrame(np.asarray(PATH)), 'state': None,
            'left': {'present': present, 'points': 4 if present else 0,
                     'rearmost_ds_m': rear if present else None, 'frontmost_ds_m': rear+.2 if present else None},
            'right': {'present': False, 'points': 0, 'rearmost_ds_m': None, 'frontmost_ds_m': None}}


@pytest.fixture
def setup(monkeypatch):
    state = {'view': view(), 'band': {'free': True, 'samples': 10, 'observed_free': 10, 'masked': 0, 'unobserved': 0},
             'window': {'free': True, 'samples': 40, 'observed_free': 40, 'masked': 0, 'unobserved': 0},
             'points': []}
    monkeypatch.setattr(ps, 'threat_view', lambda *a: state['view'])
    monkeypatch.setattr(ps, 'band_check', lambda *a: state['window'] if len(a) > 5 else state['band'])
    # RELEASED 이동 영역 검사는 원래 선택기 계약(별도 시험)이다. 여기서는 관측된 영역으로 둔다.
    monkeypatch.setattr(ps, 'motion_region', lambda *a, **k: {'distance_m': .1, 'queries': 4, 'unobserved': 0,
        'masked_left': 0, 'masked_right': 0, 'masked_tail_extent_m': 0., 'masked_front_samples': 0})
    monkeypatch.setattr(parallel_speed, 'proposal', lambda *a: {'eligible': False, 'reason': 'fixture_no_speed_proof'})
    ctl = lk.StraightLanePreferenceSelector(config())

    def call(t, *, speed=.3, previous=0., base=None, par=None, epoch=0):
        scene = SimpleNamespace(source_first_s=t-.1, t_ref_s=t, source_epoch=epoch,
                                occupied_endpoints_xy_m=state['points'], rays=[SimpleNamespace(acquired_at_s=t-.0002)])
        return ctl.select(scene, baseline() if base is None else base, parallel() if par is None else par,
                          speed, previous, .02)
    return ctl, state, call


def release_from_rear(state, call, rear=-.2):
    state['view'] = view(True, rear)
    call(1.)
    state['view'] = view()
    call(1.1)
    return call(1.2)


def test_rear_release_toward_hidden_side_keeps_current_lane(setup):
    ctl, state, call = setup
    release_from_rear(state, call)
    assert ctl.mode == 'RELEASED' and 'left' in ctl.lane_keep
    out = call(1.3, base=baseline(1., .2))
    assert out[:2] == (.35, 0.)
    assert out[2]['selector_reason'] == 'released_straight_lane_preference'
    assert out[2]['rear_lane_keep']['applied'] is True
    assert out[2]['rear_lane_keep']['unknown_filled'] is False
    assert out[2]['rear_lane_keep']['hidden_rear_risk_unresolved'] is True


def test_motion_away_or_straight_keeps_original_release(setup):
    ctl, state, call = setup
    release_from_rear(state, call)
    for angle in (0., -.2):
        out = call(1.3+abs(angle), base=baseline(1., angle))
        assert out[:2] == (1., angle) and out[2]['rear_lane_keep']['applied'] is False


def test_same_observed_straight_context_does_not_expire_by_time(setup):
    ctl, state, call = setup
    release_from_rear(state, call)
    assert call(3.3, base=baseline(1., .2))[0] == .35
    assert call(5.0, base=baseline(1., .2))[0] == .35


def test_curved_reference_ends_current_preference(setup):
    ctl, state, call = setup
    release_from_rear(state, call)
    par = parallel(0.)
    par[2]['parallel_hold_trial']['reason'] = 'reference_not_straight'
    assert call(1.3, base=baseline(1., .2), par=par)[:2] == (1., .2)
    assert not ctl.lane_keep
    assert call(1.4, base=baseline(1., .2))[:2] == (1., .2)


def test_wall_reference_change_ends_preference(setup):
    ctl, state, call = setup
    release_from_rear(state, call)
    base = baseline(1., .2); base[2]['preferred_wall'] = 'left'
    assert call(1.3, base=base)[0] == .35
    base[2]['preferred_wall'] = 'right'
    assert call(1.4, base=base)[:2] == (1., .2)
    assert not ctl.lane_keep


def test_forward_pass_never_holds_lane(setup):
    ctl, state, call = setup
    release_from_rear(state, call, rear=.25)
    assert ctl.sides['left']['vanish'] == 'benign' and not ctl.lane_keep
    assert call(1.3, base=baseline(1., .2))[:2] == (1., .2)


def test_sticky_rear_record_does_not_hold_after_later_forward_pass(setup):
    ctl, state, call = setup
    release_from_rear(state, call)
    # 같은 쪽 물체가 다시 보이고 앞으로 지나갔다. 원래 rear 기록은 남지만 이번 소실 방향은 앞이다.
    state['view'] = view(True, .25)
    call(1.3)
    state['view'] = view()
    call(1.4)
    call(1.5)
    assert ctl.sides['left']['vanish'] == 'rear' and ctl.mode == 'RELEASED'
    assert not ctl.lane_keep
    assert call(1.6, base=baseline(1., .2))[:2] == (1., .2)


def test_parallel_that_cannot_move_never_creates_new_stop(setup):
    ctl, state, call = setup
    release_from_rear(state, call)
    out = call(1.3, base=baseline(1., .2), par=parallel(0.))
    assert out[:2] == (1., .2)
    assert out[2]['rear_lane_keep']['reason'] == 'parallel_not_moving_keep_original'


def test_slower_parallel_ends_hold_for_this_release(setup):
    ctl, state, call = setup
    release_from_rear(state, call)
    out = call(1.3, base=baseline(1., .2), par=parallel(.2))
    assert out[:2] == (1., .2) and out[2]['rear_lane_keep']['reason'] == 'parallel_slower_end_hold'
    # 보류가 끝났으므로 평행 후보가 다시 빨라져도 원래 판단을 쓴다.
    assert call(1.4, base=baseline(1., .2))[:2] == (1., .2)


def test_near_return_on_opposite_side_ends_hold(setup):
    ctl, state, call = setup
    release_from_rear(state, call)
    # 오른쪽 옆면 밖 0.12 m의 앞쪽 반환(팽창 0.10 + 0.5 m/s x 0.1 s = 0.15 m 안).
    state['points'] = [[.8, -.195]]
    out = call(1.3, base=baseline(1., .2))
    assert out[:2] == (1., .2)
    assert out[2]['rear_lane_keep']['reason'] == 'opposite_near_occupancy_end_hold'
    assert not ctl.lane_keep


def test_lead_edge_beside_corridor_is_followed_not_an_opposite_trap(setup):
    ctl, state, call = setup
    release_from_rear(state, call)
    # 통로를 가로막는 앞차의 뒷면: 안쪽 반환과 같은 종방향 위치의 바깥 가장자리 반환.
    state['points'] = [[.8, y] for y in (-.12, -.10, -.05, 0., .05)]
    assert call(1.3, base=baseline(1., .2))[0] == .35


def test_away_turn_rear_swing_is_not_an_approach(setup):
    ctl, state, call = setup
    release_from_rear(state, call)
    out = call(1.3, base=baseline(1., -.37))
    assert out[:2] == (1., -.37) and out[2]['rear_lane_keep']['reason'] == 'not_toward_recent_rear_side'


@pytest.mark.parametrize('point', [[.8, -.30], [.8, .195], [3., -.195], [.8, -.05]])
def test_far_same_side_out_of_range_or_inside_corridor_returns_do_not_end_hold(setup, point):
    ctl, state, call = setup
    release_from_rear(state, call)
    state['points'] = [point]
    assert call(1.3, base=baseline(1., .2))[0] == .35


def test_reappearing_side_goes_back_to_original_guard(setup):
    ctl, state, call = setup
    release_from_rear(state, call)
    state['view'] = view(True, -.2)
    out = call(1.3, base=baseline(1., .2))
    assert ctl.mode == 'GUARD' and not ctl.lane_keep
    assert out[2]['selector_mode'] == 'GUARD' and out[2]['rear_lane_keep']['applied'] is False


def test_sensor_reset_unknown_release_also_holds_lane(setup):
    ctl, state, call = setup
    state['view'] = view(True, -.2)
    call(1.)
    ctl.invalidate('sensor_reset')
    state['view'] = view()
    call(1.1)
    call(1.2)
    assert ctl.sides['left']['vanish'] == 'unknown' and ctl.mode == 'RELEASED'
    assert set(ctl.lane_keep) == {'left', 'right'}
    assert call(1.3, base=baseline(1., .2))[0] == .35


@pytest.mark.parametrize('base', [baseline(0., .2, 'follow_gap_hold'), baseline(0., .2, 'follow_release_wait')])
def test_baseline_stop_is_unchanged(setup, base):
    ctl, state, call = setup
    release_from_rear(state, call)
    out = call(1.3, base=base, previous=.05)
    assert out[0] == 0.


@pytest.mark.parametrize('blocked', [False, True])
def test_speed_increase_requires_its_own_observed_region(setup, monkeypatch, blocked):
    ctl, state, call = setup
    release_from_rear(state, call)
    def proposal(scene, base, speed, previous, dt, cfg, cap):
        return dict(eligible=True, reason='parallel_current_position', target_speed_mps=min(.8, cap),
                    steering_rad=0., path_points=PATH, clearance_m=1.5, maximum_padding_m=.1)
    monkeypatch.setattr(parallel_speed, 'proposal', proposal)
    region = dict(distance_m=.5, queries=10, unobserved=1 if blocked else 0,
                  masked_left=0, masked_right=0, masked_tail_extent_m=0., masked_front_samples=0)
    parent_region = ps.motion_region
    calls = []
    def specific_region(*a, **k):
        calls.append(a[2])
        return region if math.isclose(a[2], .8) else parent_region(*a, **k)
    monkeypatch.setattr(ps, 'motion_region', specific_region)
    out = call(1.3, base=baseline(1., .2))
    assert .8 in calls
    assert out[0] == (.35 if blocked else .8)
    assert out[2]['rear_lane_keep']['observed_speed']['applied'] is (not blocked)


def test_speed_upgrade_never_exceeds_current_baseline(setup, monkeypatch):
    ctl, state, call = setup
    release_from_rear(state, call)
    caps = []
    def proposal(scene, base, speed, previous, dt, cfg, cap):
        caps.append(cap)
        return dict(eligible=True, reason='parallel_current_position', target_speed_mps=cap,
                    steering_rad=0., path_points=PATH, clearance_m=1.5, maximum_padding_m=.1)
    monkeypatch.setattr(parallel_speed, 'proposal', proposal)
    assert call(1.3, base=baseline(.5, .2))[0] == .5
    assert caps == [.5]


def test_large_masked_tail_cannot_approve_higher_speed(setup, monkeypatch):
    ctl, state, call = setup
    release_from_rear(state, call)
    monkeypatch.setattr(parallel_speed, 'proposal', lambda *a: dict(eligible=True, reason='parallel_current_position',
        target_speed_mps=.8, steering_rad=0., path_points=PATH, clearance_m=1.5, maximum_padding_m=.1))
    parent_region = ps.motion_region
    calls = []
    def specific_region(*a, **k):
        calls.append(a[2])
        if math.isclose(a[2], .8):
            return dict(distance_m=.5, queries=10, unobserved=0,
                masked_left=2, masked_right=0, masked_tail_extent_m=.011, masked_front_samples=0)
        return parent_region(*a, **k)
    monkeypatch.setattr(ps, 'motion_region', specific_region)
    out = call(1.3, base=baseline(1., .2))
    assert .8 in calls
    assert out[0] == .35
    assert not out[2]['rear_lane_keep']['observed_speed']['applied']



def test_target_headroom_does_not_clip_measured_speed(setup, monkeypatch):
    ctl, state, call = setup
    release_from_rear(state, call)
    captured = []
    def proposal(scene, base, speed, previous, dt, cfg, cap):
        captured.append((speed, cap))
        return dict(eligible=True, reason='parallel_current_position', target_speed_mps=cap,
                    steering_rad=0., path_points=PATH, clearance_m=1.5, maximum_padding_m=.1)
    monkeypatch.setattr(parallel_speed, 'proposal', proposal)
    out = call(1.3, speed=.9967076692639, base=baseline(1., .2))
    quantum = math.pi*.065/(28*15*.02)
    assert lk.ENCODER_SPEED_QUANTUM_MPS == pytest.approx(quantum)
    assert captured == pytest.approx([(.9967076692639, 1.-quantum)])
    assert out[0] == pytest.approx(1.-quantum)
    assert out[2]['rear_lane_keep']['observed_speed']['measured_speed_unclipped_mps'] == .9967076692639


@pytest.fixture
def measured_case(monkeypatch):
    state = dict(clearance=1.4, unobserved=0, captured=[])
    monkeypatch.setattr(parallel_speed, 'per_ray_padding', lambda *a: np.array([.1]))
    monkeypatch.setattr(parallel_speed, 'swept_limit', lambda *a, **k: state['clearance'])
    def region(scene, speed, target, *a, **k):
        state['captured'].append((speed, target))
        return dict(distance_m=.6, queries=100, unobserved=state['unobserved'],
                    masked_left=0, masked_right=0, masked_tail_extent_m=0., masked_front_samples=0)
    monkeypatch.setattr(ps, 'motion_region', region)
    base = baseline()
    base[2]['partial_observation'] = dict(allowed=True, reason='fully_observed')
    scene = SimpleNamespace(t_ref_s=.1, source_first_s=0., source_epoch=0,
                            occupied_endpoints_xy_m=[[1.4, 0.]])
    return scene, base, state


@pytest.mark.parametrize('speed', [1.0210176124, 1.10, 1.25])
def test_measured_speed_above_target_is_used_in_its_own_region(measured_case, speed):
    scene, base, state = measured_case
    result = parallel_speed.proposal(scene, base, speed, 0., .02, config(), .975)
    assert result['eligible']
    assert result['target_speed_mps'] <= .975
    assert state['captured'] == [(speed, result['target_speed_mps'])]
    assert result['measured_speed_mps'] == speed


def test_measurement_overshoot_cannot_borrow_unobserved_space(measured_case):
    scene, base, state = measured_case
    state['unobserved'] = 1
    result = parallel_speed.proposal(scene, base, 1.021, 0., .02, config())
    assert not result['eligible'] and result['reason'] == 'measured_region_unobserved'


def test_measured_stopping_distance_checked_before_allowing_positive_parallel(measured_case):
    scene, base, state = measured_case
    state['clearance'] = .3
    result = parallel_speed.proposal(scene, base, 1.021, 0., .02, config())
    assert not result['eligible'] and result['reason'] == 'measured_stop_space_insufficient'
    assert not state['captured']


@pytest.mark.parametrize('speed', [3., 1000., -1., math.nan, math.inf])
def test_invalid_or_out_of_horizon_input_never_allocates_motion_region(measured_case, speed):
    scene, base, state = measured_case
    assert not parallel_speed.proposal(scene, base, speed, 0., .02, config())['eligible']
    assert not state['captured']


def test_nominal_speed_keeps_original_proposal(measured_case):
    from arena_autonomy.parallel_lane_trial import proposal
    scene, base, state = measured_case
    import arena_autonomy.parallel_lane_trial as original
    # 공유 점유 계산을 동일하게 제공하고 목표 .35의 실제 출력/증거를 대조한다.
    from unittest.mock import patch
    with patch.object(original, 'per_ray_padding', parallel_speed.per_ray_padding), patch.object(original, 'swept_limit', parallel_speed.swept_limit):
        expected = proposal(scene, base, .7, .0, .02, config())
    assert parallel_speed.proposal(scene, base, .7, .0, .02, config()) == expected
    assert not state['captured']


def test_overshoot_still_requires_fresh_restart_confirmations(measured_case):
    from parallel_state import MeasuredParallelTrial
    scene, base, state = measured_case
    trial = MeasuredParallelTrial(config())
    first = trial.apply(scene, base, 1.021, 0., .02)
    assert first[0] == 0 and first[2]['reason'] == 'parallel_hold_release_wait'
    scene.source_first_s, scene.t_ref_s = .1, .2
    second = trial.apply(scene, base, 1.021, 0., .02)
    assert 0 < second[0] <= .35
