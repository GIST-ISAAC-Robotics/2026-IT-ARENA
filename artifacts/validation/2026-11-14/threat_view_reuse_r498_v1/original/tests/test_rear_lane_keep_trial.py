"""후방 소실 직후 차선 유지 후보의 상태 계약 반례. 주입 view는 분기만 검증하며 센서 인식 증명이 아니다."""
import math
from types import SimpleNamespace

import pytest

import numpy as np

import arena_autonomy.m3_execution_trial as mx
import arena_autonomy.pass_release_trial as ps
import arena_autonomy.rear_lane_keep_trial as lk
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
    ctl = lk.RearLaneKeepSelector(config())

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


def test_flag_requires_explicit_path_recovery_and_defaults_off():
    assert FollowConfig().rear_lane_keep_trial is False
    with pytest.raises(ValueError):
        FollowConfig(**{**CHAIN, 'traffic_path_recovery_trial': False, 'rear_lane_keep_trial': True})
    with pytest.raises(ValueError):
        FollowConfig(**{**CHAIN, 'rear_lane_keep_trial': 1})
    assert type(LocalFollow(FollowConfig(**CHAIN)).pass_selector) is SuspendedPassageSelector
    assert type(LocalFollow(config()).pass_selector) is lk.RearLaneKeepSelector


@pytest.mark.parametrize('hold', [0., -1., 2.01, math.nan, math.inf, True])
def test_hold_must_stay_within_motion_history(hold):
    with pytest.raises(ValueError):
        lk.LaneKeepConfig(hold)


def test_rear_release_toward_hidden_side_keeps_current_lane(setup):
    ctl, state, call = setup
    release_from_rear(state, call)
    assert ctl.mode == 'RELEASED' and 'left' in ctl.lane_keep
    out = call(1.3, base=baseline(1., .2))
    assert out[:2] == (.35, 0.)
    assert out[2]['selector_reason'] == 'released_recent_rear_lane_keep'
    assert out[2]['rear_lane_keep']['applied'] is True
    assert out[2]['rear_lane_keep']['unknown_filled'] is False
    assert out[2]['rear_lane_keep']['hidden_rear_risk_unresolved'] is True


def test_motion_away_or_straight_keeps_original_release(setup):
    ctl, state, call = setup
    release_from_rear(state, call)
    for angle in (0., -.2):
        out = call(1.3+abs(angle), base=baseline(1., angle))
        assert out[:2] == (1., angle) and out[2]['rear_lane_keep']['applied'] is False


def test_hold_expires_after_two_seconds(setup):
    ctl, state, call = setup
    release_from_rear(state, call)
    assert call(3.2, base=baseline(1., .2))[0] == .35
    out = call(3.3, base=baseline(1., .2))
    assert out[:2] == (1., .2) and not ctl.lane_keep


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
