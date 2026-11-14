"""복귀 후 실제 명령 외곽 진단. 출력/STOP과 분리하고 기존 기하·관측을 사용한다."""
from dataclasses import replace
import math

import numpy as np
import pytest

from arena_autonomy.m3_return_guard import assess_return_command, ReturnGuardAudit
from arena_autonomy.m3_execution_trial import ExecutionConfig, envelope_distance
from test_lateral_intent import GRASS, car
from test_m3_observed_follow import timed_scene


def check(*, speed=.35, target=.35, angle=0., previous=0., dt=.02, extra=None, dropout=None):
    scene, path = timed_scene(1., 0., extra=extra or [], dropout=dropout)
    return assess_return_command(scene, [], path, target_speed=target, steering=angle,
        measured_speed=speed, previous_steering=previous, control_dt=dt, cfg=GRASS, execution=ExecutionConfig())


def test_clear_straight_uses_actual_output_not_another_pursuit_command():
    got = check()
    assert got['sampled_sweep_ok'], got
    assert got['envelope']['distance_m'] == pytest.approx(envelope_distance(.35, .1, .02, ExecutionConfig()))
    assert not got['runtime_authorized'] and not got['output_changed']
    assert not got['continuous_sweep_certified'] and not got['body_standstill_verified']


def test_zero_target_does_not_erase_moving_braking_sweep():
    moving = check(target=0., speed=.35)
    stopped = check(target=0., speed=0.)
    assert moving['envelope']['distance_m'] > stopped['envelope']['distance_m'] == 0.
    assert moving['stop_command_must_not_be_delayed']


def test_previous_angle_and_issued_angle_are_both_inspected():
    got = check(angle=0., previous=.2)
    assert [x['steering_rad'] for x in got['envelope']['geometry']] == [0., .2]
    assert not got['sampled_sweep_ok']  # 현재 후방 사각에서 회전 외곽을 확인할 수 없다.


def test_side_vehicle_is_not_ignored_by_prior_state_release():
    got = check(extra=[car(0., .16, 0.)])
    assert not got['sampled_sweep_ok']
    assert any('lateral_clearance_short' in x['reasons'] for x in got['envelope']['geometry'])


def test_new_unknown_is_not_cleared_by_prior_release():
    got = check(dropout=lambda deg: np.abs(deg) < 20.)
    assert not got['sampled_sweep_ok'] and 'return_sweep_unobserved' in got['reasons']


@pytest.mark.parametrize('kwargs', [dict(speed=-.1), dict(target=-.1), dict(speed=math.nan),
    dict(previous=math.inf), dict(angle=1.), dict(dt=0.), dict(dt=.3), dict(target=True)])
def test_invalid_command_never_returns_supported(kwargs):
    got = check(**kwargs)
    assert not got['sampled_sweep_ok'] and got['reason'] == 'return_command_input_invalid'


@pytest.mark.parametrize('path', [None, [], [[0., 0.]], [[0., 0.], [math.nan, 1.]], [[0., 0., 0.]]])
def test_invalid_reference_path_is_not_extrapolated(path):
    scene, _ = timed_scene(1., 0.)
    got = assess_return_command(scene, [], path, target_speed=.35, steering=0., measured_speed=.35,
        previous_steering=0., control_dt=.02, cfg=GRASS, execution=ExecutionConfig())
    assert not got['sampled_sweep_ok'] and got['reason'] == 'return_reference_path_invalid'


def test_release_latch_survives_absent_field_and_new_error_until_explicit_reset():
    audit = ReturnGuardAudit()
    assert not audit.observe_release({'m3_lateral': {'mode': 'BASELINE'}})
    assert audit.observe_release({'m3_lateral': {'released_to_baseline': True}})
    assert audit.observe_release({'reason': 'follow_input_stop'})
    assert audit.observe_release({'m3_lateral': {'mode': 'BASELINE'}})
    assert audit.release_events == 1
    audit.reset()
    assert not audit.ever_released
