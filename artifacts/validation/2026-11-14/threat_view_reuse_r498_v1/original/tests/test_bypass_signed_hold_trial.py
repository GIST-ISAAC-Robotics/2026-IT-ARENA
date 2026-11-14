from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest

from arena_autonomy.bypass_signed_hold_trial import SignedHoldGuide
from arena_autonomy.bypass_corridor_trial import CorridorGuide
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.scene_motion import motion_scene
from probe_bypass_entry_sequence import render_scan


def test_explicit_selection_preserves_old_classes():
    from arena_autonomy.bypass_entry_follow import BypassEntryFollow, BypassExecutionConfig
    from arena_autonomy.bypass_entry_trial import EntryGuide
    for name, cls in [('off', EntryGuide), ('corridor_v1', CorridorGuide),
                      ('corridor_v2_signed_hold', SignedHoldGuide)]:
        follow = BypassEntryFollow(execution=BypassExecutionConfig(corridor_policy=name), allow_simulation_trial=True)
        assert type(follow.guide) is cls
    assert BypassExecutionConfig().corridor_policy == 'off'


def inputs():
    physical = [dict(t=t, rear_pose=[-.0675, 0., 0.], speed=0., steering=0.) for t in (0., 1.2)]
    scan = render_scan(physical, .5, 0.)
    history = MotionHistory(MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675, lidar_x_m=.06))
    for t in np.arange(.2, 1.21, .01):
        history.add_mean_wheel(float(t), -1.)
        history.add_gyro(float(t), 0.)
    return motion_scene(scan, history, .5, timing_verified=True, rear_half_angle_deg=30.), history


def active():
    guide = SignedHoldGuide(stationary_outer_boundary=True, wall_reanchor=True, tracked_wall_fit=True)
    guide.path = np.array([[0., 0.], [.3, 0.], [.6, 0.]])
    guide.wall_line = np.array([[0., -.425], [1., -.425]])
    guide.reference_s, guide.last_source, guide.epoch = .45, .3, 0
    guide.side, guide.offset, guide.transition = 'right', -.26, .9
    return guide


def test_negative_feedback_keeps_signed_transport_but_never_commands():
    scene, history = inputs()
    guide = active()
    x = guide.path[0, 0]
    result = guide.evaluate(scene, history, -.0325, 0., .02)
    assert result['selected'] is None and result['reason'] == 'entry_reverse_motion_hold'
    assert result['measured_speed_mps'] == -.0325
    assert guide.path[0, 0] > x and guide.reference_s == scene.t_ref_s
    assert guide.reverse_wait and guide.wall_confirmations == 0


@pytest.mark.parametrize('speed,previous,dt', [(float('nan'), 0., .02), (False, 0., .02),
    (-1.1, 0., .02), (-.1, float('nan'), .02), (-.1, 0., -.02), (-.1, 3., .02)])
def test_invalid_input_still_erases_geometry(speed, previous, dt):
    scene, history = inputs()
    guide = active()
    result = guide.evaluate(scene, history, speed, previous, dt)
    assert result['selected'] is None and guide.path is None and not guide.reverse_wait


def test_new_epoch_and_stale_source_still_reset():
    scene, history = inputs()
    epoch = replace(scene, source_epoch=1, rays=tuple(replace(r, source_epoch=1) for r in scene.rays))
    for bad in (epoch, replace(scene, t_ref_s=1.)):
        guide = active()
        assert guide.evaluate(bad, history, -.1, 0., .02)['selected'] is None
        assert guide.path is None


def test_missing_motion_cannot_retain_geometry():
    scene, _ = inputs()
    guide = active()
    result = guide.evaluate(scene, MotionHistory(MotionConfig()), -.1, 0., .02)
    assert result['reason'] == 'entry_reverse_motion_transport_unavailable'
    assert guide.path is None and not guide.reverse_wait


def test_same_scan_cannot_supply_forward_confirmations(monkeypatch):
    scene, history = inputs()
    guide = active()
    guide.evaluate(scene, history, -.1, 0., .02)
    current = dict(reason='candidate', selected=dict(speed_mps=.1, steering_rad=0.),
        reference=dict(reason='current_outer_wall_rechecked_for_remembered_guide'),
        outer_wall_fresh_confirmations=2)
    monkeypatch.setattr(CorridorGuide, 'evaluate', lambda *a: current)
    assert guide.evaluate(scene, history, 0., 0., .02)['selected'] is None
    rays = tuple(replace(r, acquired_at_s=r.acquired_at_s+.1) for r in scene.rays)
    newer = replace(scene, source_first_s=rays[0].acquired_at_s, source_last_s=rays[-1].acquired_at_s,
        rays=rays, t_ref_s=.6)
    assert guide.evaluate(newer, history, 0., 0., .02)['selected'] is None
    assert guide.evaluate(newer, history, 0., 0., .02)['selected'] is None
    rays = tuple(replace(r, acquired_at_s=r.acquired_at_s+.1) for r in newer.rays)
    newest = replace(newer, source_first_s=rays[0].acquired_at_s, source_last_s=rays[-1].acquired_at_s,
        rays=rays, t_ref_s=.7)
    assert guide.evaluate(newest, history, 0., 0., .02)['signed_hold_recovered']


def test_current_obstacle_or_wall_failure_cancels_recovery(monkeypatch):
    scene, history = inputs()
    guide = active()
    guide.evaluate(scene, history, -.1, 0., .02)
    guide.forward_confirmations = 1
    monkeypatch.setattr(CorridorGuide, 'evaluate', lambda *a: dict(reason='occupied', selected=None))
    assert guide.evaluate(scene, history, 0., 0., .02)['selected'] is None
    assert guide.forward_confirmations == 1  # 같은 입력 재사용은 새 반증/새 확인으로 세지 않는다.
    rays = tuple(replace(r, acquired_at_s=r.acquired_at_s+.1) for r in scene.rays)
    newer = replace(scene, source_first_s=rays[0].acquired_at_s, source_last_s=rays[-1].acquired_at_s,
        rays=rays, t_ref_s=.6)
    assert guide.evaluate(newer, history, 0., 0., .02)['selected'] is None
    assert guide.forward_confirmations == 0  # 새 관측에서 막히면 연속 확인을 지운다.
    guide.reset()
    assert guide.path is None and not guide.reverse_wait
