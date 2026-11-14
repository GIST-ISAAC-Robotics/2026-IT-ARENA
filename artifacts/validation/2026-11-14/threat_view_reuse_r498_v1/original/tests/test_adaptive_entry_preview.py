"""짧은 목표점 후보의 현재 제어 모듈 표적 회귀."""

import math
import numpy as np
import pytest
from arena_autonomy.adaptive_entry_preview import shorter_pursuit, compare_shorter_previews
from arena_autonomy.local_follow import FollowConfig, pursue_path
from arena_autonomy.lidar_motion import MotionHistory, MotionUnavailable
from arena_autonomy.bypass_handoff import HandoffExecutionConfig


def quiet_history():
    h = MotionHistory()
    for k in range(26):
        h.wheels.add(k*.02, 0.)
    return h


def blocked(reason='known_occupied_command_hold'):
    return dict(reason='no_entry_command_candidate', selected=None,
                candidates=[dict(reason=reason, preview_unclipped_steering_rad=-.30)])


def run(compare, primary=None, **kwargs):
    return compare_shorter_previews(blocked() if primary is None else primary, compare,
        **dict(dict(history=quiet_history(), now=.50, speed=0., previous=0., dt=.02), **kwargs))


@pytest.mark.parametrize('look', [.24, .18, .12])
def test_target_uses_same_path_and_preserves_path_length_and_original_speed_budget(look):
    cfg = FollowConfig()
    x = np.linspace(0, 1.2, 61)
    path = np.column_stack((x, -.1*(1-np.cos(np.pi*np.minimum(x/.8, 1)))))
    saved = path.copy()
    oldcap, _, remaining = pursue_path(path, .1, cfg, .12, speed_cap=.35)
    cap, steer, newremaining = shorter_pursuit(path, .1, cfg, .12, lookahead_m=look, speed_cap=.35)
    np.testing.assert_array_equal(path, saved)
    assert newremaining == remaining and 0 < cap <= oldcap <= .35
    assert -cfg.max_steering_rad <= steer < 0


def test_short_preview_does_not_turn_short_unobserved_path_into_a_valid_path():
    path = np.array([[0., 0.], [.01, 0.], [.02, 0.]])
    with pytest.raises(MotionUnavailable, match='path_too_short'):
        shorter_pursuit(path, 0., FollowConfig(), .1, lookahead_m=.12)


def test_existing_command_and_non_occupancy_rejections_are_unchanged():
    def unexpected(*_):
        raise AssertionError('fallback must not run')
    for primary in (dict(blocked(), selected=dict(speed_mps=.1)),
                    blocked('unobserved_motion_region'),
                    dict(blocked(), reason='invalid_input'),
                    dict(blocked(), candidates=[])):
        assert run(unexpected, primary) is primary


def test_align_only_keeps_motion_rejected_and_limits_steering_change():
    calls = []
    def compare(look, prev):
        calls.append((look, prev))
        return dict(blocked(), selected=dict(speed_mps=.35)) if prev == -.30 else blocked()
    output = run(compare)
    assert calls == [(.24, 0.), (.24, -.30)]
    assert output['selected'] is None
    assert output['preview_alignment']['requested_rad'] == pytest.approx(-.06)
    assert output['preview_alignment']['new_motion_authorized'] is False


@pytest.mark.parametrize('speed', [.0813, -.0813, float('nan')])
def test_moving_reverse_or_invalid_speed_cannot_use_stationary_alignment(speed):
    def compare(look, prev):
        return dict(blocked(), selected=dict(speed_mps=.35)) if prev == -.30 else blocked()
    assert 'preview_alignment' not in run(compare, speed=speed)


@pytest.mark.parametrize('kind', ['missing', 'stale', 'gap', 'pulse'])
def test_encoder_window_is_not_replaced_by_current_zero_sample(kind):
    h = quiet_history()
    if kind == 'missing':
        h.wheels.times.clear(); h.wheels.values.clear()
    elif kind == 'stale':
        h.wheels.times = h.wheels.times[:17]; h.wheels.values = h.wheels.values[:17]
    elif kind == 'gap':
        del h.wheels.times[18:22]; del h.wheels.values[18:22]
    else:
        h.wheels.values[21] = .0813
    def compare(look, prev):
        return dict(blocked(), selected=dict(speed_mps=.35)) if prev == -.30 else blocked()
    assert 'preview_alignment' not in run(compare, history=h)


def test_positive_command_requires_comparison_with_actual_previous_angle():
    calls = []
    def compare(look, prev):
        calls.append((look, prev))
        return dict(blocked(), selected=dict(speed_mps=.15, steering_rad=prev))
    output = run(compare, speed=.0813, previous=-.28)
    assert calls == [(.24, -.28)]
    assert output['selected']['speed_mps'] == .15
    assert 'preview_alignment' not in output


@pytest.mark.parametrize('reason', ['known_occupied_command_hold', 'unobserved_motion_region'])
def test_all_rejected_alternatives_remain_rejected(reason):
    output = run(lambda *_: blocked(reason))
    assert output['selected'] is None and 'preview_alignment' not in output
    assert not output['adaptive_preview']['applied']


def test_feature_is_off_and_requires_explicit_corridor_policy():
    assert HandoffExecutionConfig().adaptive_entry_preview is False
    with pytest.raises(ValueError, match='explicit corridor'):
        HandoffExecutionConfig(adaptive_entry_preview=True)


def test_sampled_support_never_becomes_motion_permission(monkeypatch):
    from arena_autonomy.adaptive_entry_preview import sampled_region_rejection
    import arena_autonomy.pass_release_trial as pr
    import arena_autonomy.observation_support as os
    import arena_autonomy.m3_execution_trial as mx
    monkeypatch.setattr(pr, 'motion_queries', lambda *a: (.2, np.ones((80, 2))))
    monkeypatch.setattr(mx, 'ray_padding', lambda *a: np.zeros(500))
    monkeypatch.setattr(os, 'depth_support', lambda scene, q, pad: np.ones(len(q), dtype=bool))
    assert sampled_region_rejection(None, 0., .35, -.3, -.3, .02, FollowConfig(), None, None) is None
