"""빠른 추월의 명령/측정 분리. 합성 검사이며 실제 추월 성공 증명이 아니다."""
from dataclasses import replace
from unittest.mock import patch

import numpy as np
import pytest

from arena_autonomy.bypass_entry_trial import _compare_entries, entry_motion_region
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.front_patch_motion import ObservedForwardPace
from arena_autonomy.moving_pass import MovingEntryConfig
from arena_autonomy.moving_pass import available_pass_offsets
from arena_autonomy.bypass_entry_trial import straight_reference
from arena_autonomy.bypass_handoff import front_patch
from arena_autonomy.pass_release_trial import lateral_config
from probe_bypass_entry import sensor_scene
from test_bypass_entry_guide import started
from test_moving_pass import moving_controller, history, scene


def faster_controller():
    old = moving_controller()
    return BypassHandoffFollow(old.config, replace(old.execution, moving_pass_profile='faster_v1'),
                               allow_simulation_trial=True)


def faster_entry():
    return MovingEntryConfig(profile='faster_v1', speed_cap_mps=1.2)


def test_explicit_faster_profile_keeps_legacy_caps_and_increases_motion_budget():
    with pytest.raises(ValueError):
        HandoffExecutionConfig(moving_pass_profile='faster_v1')
    assert not MovingEntryConfig().measured_speed_envelope
    assert not MovingEntryConfig(profile='medium_v1', speed_cap_mps=.9).measured_speed_envelope
    assert faster_entry().measured_speed_envelope
    ctrl = faster_controller()
    assert ctrl.config.max_speed_mps == 1.
    assert ctrl.config.object_motion_bound_mps == 1.2
    assert ctrl.normal.config.object_motion_bound_mps == 1.2
    assert ctrl.moving_pass.tracker.config.object_motion_bound_mps == 1.2
    assert ctrl.moving_pass.guide.entry.transitions_m == (.9, 1.2, 1.5)
    with pytest.raises(ValueError):
        MovingEntryConfig(profile='faster_v1', speed_cap_mps=1.2001)


def test_measurement_above_normal_command_cap_keeps_intent_and_checks_actual_speed():
    guide, h, first = started()
    guide.entry = faster_entry()
    before, direction = guide.path.copy(), guide.intent.sign
    s, _ = sensor_scene(targets=((1.2, 0.),), now=.6, first=.5)
    with patch('arena_autonomy.bypass_entry_trial.entry_motion_region', wraps=entry_motion_region) as check:
        out = guide.evaluate(s, h, 1.08, first['selected']['steering_rad'], .02)
    assert out['reason'] != 'entry_guide_input_reset'
    np.testing.assert_allclose(guide.path, before, atol=1e-12)
    assert guide.intent.sign == direction and check.called
    assert all(call.args[1] == 1.08 for call in check.call_args_list)
    assert all(row.get('target_speed_mps', 0.) <= 1.2 for row in out['candidates'])


def test_excessive_measured_braking_distance_is_not_a_path_reset_or_clipped_measurement():
    guide, h, first = started()
    guide.entry = faster_entry()
    before = guide.path.copy()
    s, _ = sensor_scene(targets=((1.2, 0.),), now=.6, first=.5)
    with patch('arena_autonomy.bypass_entry_trial.entry_motion_region') as check:
        out = guide.evaluate(s, h, 10., 0., .02)
    assert out['reason'] == 'measured_envelope_outside_swept_horizon'
    assert out['measured_distance_m'] > 2.5 and out['selected'] is None
    np.testing.assert_array_equal(guide.path, before)
    check.assert_not_called()


@pytest.mark.parametrize('speed', [float('nan'), float('inf'), True, -1.])
def test_invalid_sensor_values_still_reset(speed):
    guide, h, _ = started(); guide.entry = faster_entry()
    s, _ = sensor_scene(targets=((1.2, 0.),), now=.6, first=.5)
    assert guide.evaluate(s, h, speed, 0., .02)['reason'] == 'entry_guide_input_reset'
    assert guide.path is None


def test_epoch_change_still_resets_at_valid_high_speed():
    guide, h, _ = started(); guide.entry = faster_entry()
    s, _ = sensor_scene(targets=((1.2, 0.),), now=.6, first=.5, epoch=1)
    assert guide.evaluate(s, h, 1.08, 0., .02)['reason'] == 'entry_guide_input_reset'


@pytest.mark.parametrize('lead,ready', [(.7, True), (.8, True), (.86, False)])
def test_faster_admission_uses_observed_upper_and_actual_selected_advantage(lead, ready):
    ctrl = faster_controller(); h = history(); pace = ObservedForwardPace()
    for k in range(5):
        t = .1 + k*.1
        pace.observe(t, t+.1, [1.+lead*k*.1, 0.], h)
    ctrl.monitor.forward_pace = pace
    out = ctrl.moving_pass.eligibility(scene(.6), h)
    assert bool(out['ready']) == ready
    assert out['estimated_lead_upper_mps'] == pytest.approx(lead+.15)
    assert not out['future_speed_guaranteed']
    assert not ctrl.moving_pass.entry_advantage_sufficient(.99, .8)
    assert ctrl.moving_pass.entry_advantage_sufficient(1.1, .8)


def test_faster_unknown_and_occupied_support_use_same_larger_motion_budget():
    import arena_autonomy.m3_execution_trial as mx
    ctrl = faster_controller(); s, _ = sensor_scene(targets=((1.2, 0.),))
    with patch.object(mx, 'ray_padding', wraps=mx.ray_padding) as padding:
        entry_motion_region(s, 1.08, .3, 0., 0., .02, ctrl.config, lateral_config(ctrl.config))
    assert padding.called
    assert all(call.args[2] == 1.2 for call in padding.call_args_list)


def test_swept_horizon_rejects_large_input_before_geometry_and_comparison_does_not_authorize():
    ctrl = faster_controller(); s, _ = sensor_scene()
    out = _compare_entries(s, s.t_ref_s, 10., 0., .02, config=ctrl.config, entry=faster_entry())
    assert out['reason'] == 'measured_envelope_outside_swept_horizon'
    assert not out['control_authorized'] and not out['unknown_filled'] and out['selected'] is None


def test_faster_offsets_are_bounded_separately_without_changing_legacy_geometry():
    with pytest.raises(ValueError):
        MovingEntryConfig(offsets_m=(0., .34))
    assert replace(faster_entry(), offsets_m=(0., .34)).maximum_offset_m == .35
    with pytest.raises(ValueError):
        replace(faster_entry(), offsets_m=(0., .351))


def test_fixed_outer_proposal_keeps_dynamic_inner_budget_and_still_needs_two_wall_scans():
    ctrl = faster_controller()
    s, _ = sensor_scene(targets=((1.2, -.12),))
    _, reference = straight_reference(s, ctrl.config)
    patch_points, _ = front_patch(s, 'contiguous_three_v0')
    assert reference['walls'] and patch_points is not None
    old, old_width = available_pass_offsets(s, ctrl.config, reference['walls'], patch_points)
    new, new_width = available_pass_offsets(s, ctrl.config, reference['walls'], patch_points,
        fixed_outer_candidate=True, maximum_offset=.35)
    for side in new_width:
        assert new_width[side]['high_m'] == old_width[side]['high_m']
        assert new_width[side]['low_m'] < old_width[side]['low_m']
    # 후보 공간이 있더라도 첫 관측 하나로 정적 외벽 허가를 만들지 않는다.
    out = ctrl.moving_pass.guide.evaluate(s, history(), 0., 0., .02)
    assert out.get('selected') is None
    assert ctrl.moving_pass.guide.path is None
