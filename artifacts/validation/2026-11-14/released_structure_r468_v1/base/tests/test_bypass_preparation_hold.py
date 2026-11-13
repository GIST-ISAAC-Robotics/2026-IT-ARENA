"""준비 대기의 기억은 추가 감속만 하며 현재 관측·출처·고장 경계를 보존한다."""
from dataclasses import asdict, replace
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_events
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from test_bypass_handoff import context, trace_node
from test_bypass_handoff_corridor_trial import subject as corridor


def subject():
    return BypassHandoffFollow(execution=replace(corridor().execution,
        preparation_hold_policy='bounded_hold_v1'), allow_simulation_trial=True)


@pytest.fixture
def held(monkeypatch):
    ctl = subject()
    ctl.monitor.motion_evidence = lambda: False
    patch = np.array([[1.43, -.02], [1.43, 0.], [1.43, .02]])
    scene = NS(source_frame_id='laser_frame', source_epoch=0, source_first_s=10., t_ref_s=10.14)
    monkeypatch.setattr('arena_autonomy.bypass_handoff.front_patch', lambda *_: (patch, 'current'))
    ctl._preparation_hold.observe(ctl, scene, (0., 0., dict(
        preparation=dict(applied=True), entry_space=dict(hold=True))))
    assert ctl._preparation_hold.anchor is not None
    scene.source_first_s = 10.5
    scene.t_ref_s = 10.64
    monkeypatch.setattr(ctl, '_follow_entry_space_current', lambda *args: (
        args[-1], dict(reason='two_wall_straight_reference_unavailable', applied=False)))
    monkeypatch.setattr('arena_autonomy.bypass_preparation_hold.transport_points', lambda points, *_: points)
    return ctl, scene, patch


def test_hold_is_explicit_and_legacy_mapping_stays_off():
    cfg = HandoffExecutionConfig()
    old = asdict(cfg)
    old.pop('preparation_hold_policy')
    assert HandoffExecutionConfig.from_mapping(old).preparation_hold_policy == 'off'
    assert corridor()._preparation_hold is None
    with pytest.raises(ValueError):
        replace(cfg, preparation_hold_policy='bounded_hold_v1')
    assert HandoffExecutionConfig.from_mapping(asdict(subject().execution)) == subject().execution


@pytest.mark.parametrize('value', [None, True, 1, '', 'bounded', 'unlimited'])
def test_unregistered_hold_rejected(value):
    with pytest.raises(ValueError):
        replace(corridor().execution, preparation_hold_policy=value)


def test_additional_hold_cannot_grant_bypass_or_free_space(held):
    ctl, scene, _ = held
    speed, detail = ctl._follow_entry_space(scene, (.4, 0., {}), 0., .4)
    assert speed == 0. and detail['current_front_evidence'] is True
    assert detail['bypass_permission'] is False and detail['free_space_assumed'] is False


@pytest.mark.parametrize('changes', [dict(source_epoch=1), dict(source_frame_id='other'),
    dict(source_first_s=9.9), dict(t_ref_s=11.1401, source_first_s=11.),
    dict(t_ref_s=10.1, source_first_s=10.), dict(t_ref_s=10.8)])
def test_expiry_staleness_clock_or_source_change_releases(held, changes):
    ctl, scene, _ = held
    for key, value in changes.items():
        setattr(scene, key, value)
    assert ctl._follow_entry_space(scene, (.4, 0., {}), 0., .4)[0] == .4
    assert ctl._preparation_hold.anchor is None


@pytest.mark.parametrize('replacement', ['moving', 'displaced', 'missing', 'nan'])
def test_current_patch_has_priority(held, monkeypatch, replacement):
    ctl, scene, patch = held
    if replacement == 'moving':
        ctl.monitor.motion_evidence = lambda: True
    else:
        new = {'displaced': patch + [.10, 0.], 'missing': None, 'nan': patch * float('nan')}[replacement]
        monkeypatch.setattr('arena_autonomy.bypass_handoff.front_patch', lambda *_: (new, 'changed'))
    assert ctl._follow_entry_space(scene, (.4, 0., {}), 0., .4)[0] == .4
    assert ctl._preparation_hold.anchor is None


def test_current_path_outside_patch_is_not_held(held, monkeypatch):
    ctl, scene, _ = held
    monkeypatch.setattr(ctl, '_follow_entry_space_current', lambda *args: (.2, dict(reason='patch_outside_g1_swept_arc')))
    assert ctl._follow_entry_space(scene, (.4, 0., {}), 0., .4)[0] == .2


def test_encoder_compensation_and_missing_history(held, monkeypatch):
    from arena_autonomy.scene_motion import transport_points
    ctl, scene, old = held
    history = MotionHistory(MotionConfig(wheel_radius_m=.0325, history_s=3.))
    for t in np.arange(9.9, 10.7, .01):
        history.add_mean_wheel(float(t), .35 / .0325)
        history.add_gyro(float(t), 0.)
    ctl._preparation_history = history
    current = old - [.175, 0.]
    monkeypatch.setattr('arena_autonomy.bypass_handoff.front_patch', lambda *_: (current, 'current'))
    monkeypatch.setattr('arena_autonomy.bypass_preparation_hold.transport_points', transport_points)
    speed, detail = ctl._follow_entry_space(scene, (.4, 0., {}), 0., .4)
    assert speed == 0. and detail['compensated_patch_displacement_m'] < 1e-8
    ctl._preparation_history = MotionHistory(MotionConfig())
    assert ctl._follow_entry_space(scene, (.4, 0., {}), 0., .4)[0] == .4
    assert ctl._preparation_hold.anchor is None


def test_reused_scan_cannot_extend_hold(held, monkeypatch):
    ctl, scene, _ = held
    output = (0., 0., dict(preparation=dict(applied=True), entry_space=dict(hold=True)))
    monkeypatch.setattr(ctl, '_command', lambda *a, **kw: output)
    ctl.command(scene, None, 0., 'left', 0., .02, raw_scan=object())
    armed = ctl._preparation_hold.anchor['time']
    scene.t_ref_s += .10
    ctl.command(scene, None, 0., 'left', 0., .02, raw_scan=object())
    assert ctl._preparation_hold.anchor['time'] == armed
    ctl.reset()
    assert ctl._preparation_hold.anchor is None


@pytest.mark.parametrize('reason', ['handoff_reverse_motion_hold', 'handoff_input_fault',
    'handoff_invalid_input', 'handoff_source_reset'])
def test_fault_result_clears_memory(held, monkeypatch, reason):
    ctl, scene, _ = held
    monkeypatch.setattr(ctl, '_command', lambda *a, **kw: (0., 0., dict(reason=reason)))
    assert ctl.command(scene, None, 0., 'left', 0., .02, raw_scan=object())[0] == 0.
    assert ctl._preparation_hold.anchor is None


@pytest.mark.parametrize('state', ['BYPASS', 'REACQUIRE'])
def test_admitted_bypass_is_not_modified(held, monkeypatch, state):
    ctl, scene, _ = held
    ctl.mode = state
    output = (.25, .03, dict(reason='handoff_bypass'))
    monkeypatch.setattr(ctl, '_command', lambda *a, **kw: output)
    assert ctl.command(scene, None, 0., 'left', 0., .02, raw_scan=object()) == output
    assert ctl._preparation_hold.anchor is None


def test_node_header_and_replay_include_explicit_policy(tmp_path):
    node = trace_node(tmp_path, g1_g2_preparation_policy='observed_entry_space_v1',
        g1_g2_persistence_window='trim_before_compare_v1', g1_g2_follow_entry_space=True,
        g1_g2_reacquire_policy='known_hold_v1', g1_g2_entry_policy='corridor_local_v1',
        g1_g2_corridor_policy='corridor_v2_signed_hold', g1_g2_preparation_hold='bounded_hold_v1')
    path = tmp_path / 'new.jsonl'
    trace = node.create_follow_trace(str(path))
    for now in (.5, .6, .7):
        scan, history, scene = context(now)
        output = node.follow.command(scene, history, 0., 'left', 0., .02, raw_scan=scan)
        trace.record_call(scan, history, now, 0, 0., 'left', 0., .02, True, 30., output)
    trace.close()
    events = [json.loads(line) for line in path.read_text().splitlines()]
    assert events[0]['data']['execution_config']['preparation_hold_policy'] == 'bounded_hold_v1'
    assert replay_events(events)['passed']
