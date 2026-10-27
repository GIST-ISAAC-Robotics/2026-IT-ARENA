"""GUARD 곡선 선택 계약. 상태/분기 주입과 실제 기하 계산을 구별한다."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

import arena_autonomy.pass_release_trial as ps
from arena_autonomy.local_follow import FollowConfig


@pytest.fixture
def guard(monkeypatch):
    cfg = FollowConfig(partial_observation_policy='bounded_local_v4', parallel_hold_trial=True,
                       pass_release_trial=True, guard_curve_trial=True,
                       boundary_evidence_mode='corner_distance_candidate', occupancy_timing_mode='per_ray_candidate')
    state = {'offset': 0., 'heading': 0., 'regions': [], 'region': {'unobserved': 0, 'masked_left': 0, 'masked_right': 0}}

    def view(*_):
        xs = np.linspace(-1., 2., 61)
        path = np.c_[xs, state['offset'] + np.tan(state['heading'])*xs]
        return {'frame': ps.mx.ExecutionFrame(path), 'state': None,
                'left': {'present': True, 'points': 4, 'rearmost_ds_m': -.1, 'frontmost_ds_m': .1},
                'right': {'present': False, 'points': 0, 'rearmost_ds_m': None, 'frontmost_ds_m': None}}

    def region(scene, speed, target, steering, previous, dt, *rest):
        state['regions'].append((speed, target, steering, previous, dt))
        return state['region']

    monkeypatch.setattr(ps, 'threat_view', view)
    monkeypatch.setattr(ps, 'motion_region', region)
    ctl = ps.PassReleaseSelector(cfg)

    def call(t=1., *, baseline=.8, angle=-.1, p_speed=0., p_reason='parallel_hold_reference_not_straight',
             b_reason='local_follow', speed=.2, previous=0., epoch=0, age=.1):
        scene = SimpleNamespace(source_first_s=t-age, t_ref_s=t, source_epoch=epoch)
        path = [[0., state['offset']], [1., state['offset']]]
        b = (baseline, angle, {'reason': b_reason, 'path_points': path, 'behavior': 'FOLLOW'})
        p = (p_speed, 0., {'reason': p_reason, 'behavior': 'HOLD'})
        return ctl.select(scene, b, p, speed, previous, .02)

    return ctl, state, call


def test_guard_state_and_obstacle_record_survive_curved_candidate(guard):
    ctl, state, call = guard
    out = call()
    assert out[0] == .35 and out[2]['selector_choice'] == 'guard_baseline'
    assert ctl.mode == 'GUARD' and ctl.sides['left']['present'] and not ctl.sides['left']['passed']
    assert ctl.sides['left']['vanish'] is None
    assert out[2]['selector_guard_candidate']['accepted'] and len(state['regions']) == 1
    assert not out[2]['selector_unknown_filled']


@pytest.mark.parametrize('reason', ['parallel_hold_unchanged_gap_hold', 'parallel_hold_lateral_motion_not_small',
                                  'parallel_hold_short_reference', 'parallel_hold_partial_observation_outside_trial'])
def test_only_known_geometry_failure_uses_candidate(guard, reason):
    _, state, call = guard
    out = call(p_reason=reason)
    assert out[0] == 0. and out[2]['selector_choice'] == 'parallel' and not state['regions']


def test_positive_parallel_is_kept_and_release_wait_requires_previous_candidate(guard):
    _, state, call = guard
    assert call(p_speed=.2)[0] == .2
    assert not state['regions']
    assert call(1.1, p_reason='parallel_hold_release_wait')[0] == 0.
    assert call(1.2)[0] == .35
    assert call(1.3, p_reason='parallel_hold_release_wait')[0] == .35
    assert call(1.4, p_reason='parallel_hold_unchanged_gap_hold')[0] == 0.
    assert call(1.5, p_reason='parallel_hold_release_wait')[0] == 0.


@pytest.mark.parametrize('reason', ['follow_gap_hold', 'follow_release_wait', 'follow_scene_stale'])
def test_baseline_stop_is_not_replaced(guard, reason):
    _, state, call = guard
    assert call(baseline=0., b_reason=reason)[0] == 0.
    assert not state['regions']


def test_lower_target_and_current_speed_are_both_preserved(guard):
    _, state, call = guard
    out = call(baseline=.12, speed=.8, angle=0.)
    assert out[0] == .12
    assert out[2]['selector_guard_candidate']['prediction_speed_mps'] == .8
    assert state['regions'][-1][:2] == (.8, .12)


def test_small_motion_does_not_skip_unknown_or_occupied_region(guard):
    _, state, call = guard
    state['region']['unobserved'] = 1
    out = call(angle=0.)
    assert out[0] == 0. and 'large_motion_region_unobserved' in out[2]['selector_reason']
    assert len(state['regions']) == 1


@pytest.mark.parametrize('extent,front,allowed', [(.0002, 0, True), (.011, 0, False), (.0002, 1, False)])
def test_hidden_tail_is_bounded_and_never_free(guard, extent, front, allowed):
    _, state, call = guard
    state['region'].update(masked_right=2, masked_tail_extent_m=extent, masked_front_samples=front)
    out = call()
    assert (out[0] > 0.) == allowed
    if allowed:
        assert out[2]['selector_masked_tail_risk_accepted'] and not out[2]['selector_unknown_filled']


def test_cumulative_toward_side_motion_is_blocked_but_recovery_is_possible(guard):
    _, state, call = guard
    call(p_speed=.2, angle=0.)  # 진입 기준, 후보 미사용
    outputs = []
    for i, offset in enumerate((-.002, -.006, -.012)):
        state['offset'] = offset
        outputs.append(call(1.1+i*.1, angle=.02))
    assert outputs[0][0] > 0.
    assert outputs[-1][0] == 0. and outputs[-1][2]['selector_reason'] == 'guard_curve_rejected_toward_left'
    # 앞은 멀어져도 뒤 모서리가 접근하면 여전히 거절한다.
    assert call(1.5, angle=-.1)[0] == 0.
    # 차체가 경로와 비스듬하여 모든 모서리의 최대 외곽이 증가하지 않는 회복은 고착시키지 않는다.
    state['heading'] = .15
    out = call(1.6, angle=-.1)
    assert out[0] > 0. and out[2]['selector_guard_candidate']['guarded_sides']['left']['not_increasing_present_extent']


@pytest.mark.parametrize('fault', ['reverse', 'nan', 'epoch', 'rewind', 'stale'])
def test_input_faults_precede_curve_fallback(guard, fault):
    ctl, _, call = guard
    call()
    kwargs = {'reverse': {'speed': -.01}, 'nan': {'speed': float('nan')}, 'epoch': {'epoch': 1},
              'rewind': {'t': .9}, 'stale': {'age': .3}}[fault]
    out = call(**({'t': 1.1} | kwargs))
    assert out[0] == 0. and out[2]['selector_choice'] == 'hold_fault'
    if fault != 'reverse':
        assert ctl.guard_anchor == {} and all(v['vanish'] == 'unknown' for v in ctl.sides.values())


def test_curve_flag_off_and_invalid_options(guard):
    ctl, state, call = guard
    ctl.follow_config = replace(ctl.follow_config, guard_curve_trial=False)
    assert call()[0] == 0. and not state['regions']
    assert not FollowConfig().guard_curve_trial
    with pytest.raises(ValueError):
        FollowConfig(guard_curve_trial=True)
    with pytest.raises(ValueError):
        ps.SelectorConfig(guard_speed_cap_mps=.36)


def test_explicit_cli_and_evidence_cannot_claim_release_from_guard():
    from g1_partial_trial import guard_curve_arguments, pass_release_evidence
    assert guard_curve_arguments(False, False) == []
    with pytest.raises(ValueError):
        guard_curve_arguments(True, False)
    assert guard_curve_arguments(True, True) == ['g1_guard_curve_trial:=true']
    row = {'pass_release_trial': True, 'selector_choice': 'guard_baseline', 'selector_mode': 'GUARD',
           'selector_unknown_filled': False, 'selector_guard_candidate': {'accepted': True}}
    assert pass_release_evidence([{'pass_release_trial': row}], guard_curve=True)['passed']
    assert not pass_release_evidence([{'pass_release_trial': row}])['passed']
    row['selector_mode'] = 'RELEASED'
    assert not pass_release_evidence([{'pass_release_trial': row}], guard_curve=True)['passed']


@pytest.mark.parametrize('sign', [-1., 1.])
@pytest.mark.parametrize('side_y,allowed', [(.12, False), (.22, True)])
def test_real_ray_side_obstacle_blocks_motion_region_even_with_positive_baseline(sign, side_y, allowed):
    # 상태/관측 분류를 주입하지 않는다. 작은 직진도 가깝게 관측된 상자와 겹치는 여유 영역은 거절해야 한다.
    # 기존 G1의 정상 출력이라는 입력을 주되, 이 선택기의 독립적인 이동 영역 검사를 확인한다.
    from test_pass_release_trial import box_scene, baseline
    cfg = FollowConfig(partial_observation_policy='bounded_local_v4', parallel_hold_trial=True,
                       pass_release_trial=True, guard_curve_trial=True,
                       boundary_evidence_mode='corner_distance_candidate', occupancy_timing_mode='per_ray_candidate')
    ctl = ps.PassReleaseSelector(cfg)
    y0, y1 = sorted((sign*side_y, sign*(side_y+.15)))
    scene = box_scene(1., [(-.05, .15, y0, y1)])
    out = ctl.select(scene, baseline(), (0., 0., {'reason': 'parallel_hold_reference_not_straight'}),
                     .2, 0., .02)
    assert ctl.mode == 'GUARD'
    assert ctl.sides['left' if sign > 0 else 'right']['present']
    assert (out[0] > 0.) == allowed
    info = out[2]['selector_guard_candidate']
    assert (info['region']['unobserved'] == 0) == allowed
    assert not out[2]['selector_unknown_filled']
