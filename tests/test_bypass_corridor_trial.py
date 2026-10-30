"""G2 통로 명시 시험 후보의 반례, 기본 OFF, 기록 재생 연결을 확인한다. 주행 성공 판정은 별도다."""
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from arena_autonomy import bypass_entry_trial as trial
from arena_autonomy.bypass_corridor_trial import (BoundaryMonitor, CorridorGuide, CorridorPolicy, PlannedIntent,
    REGISTERED_POLICIES, abeam_age, boundary_face, choose_side, connected_wall_returns, straight_blocked, wall_frame)
from arena_autonomy.bypass_entry_follow import BypassEntryFollow, BypassExecutionConfig
from arena_autonomy.bypass_entry_trial import EntryGuide
from arena_autonomy.follow_trace import FollowTrace, equal_output, freeze, replay_events
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_pursuit import LocalPursuit
from arena_autonomy.scene_motion import motion_scene
from probe_bypass_entry_sequence import render_scan

P = REGISTERED_POLICIES['corridor_v1']


class S:
    def __init__(self, t, epoch=0):
        self.source_first_s, self.source_epoch, self.t_ref_s = t, epoch, t+.1


def face(n, along=(1., 1.2)):
    return dict(normal_m=float(n), method='test', along_range_m=along, endpoints_xy_m=None)


def same_interval(row):
    return (1., 1.2)


def feed(values, policy=P):
    monitor, out = BoundaryMonitor(policy), None
    for i, value in enumerate(values):
        out = monitor.update(S(i*.1), face(value), same_interval)
    return monitor, out


def rows(along, normal):
    along, normal = np.asarray(along, float), np.asarray(normal, float)
    return np.arange(len(along)), np.c_[along, .425-normal], along, normal


def test_registered_policy_is_the_declared_test_assumption():
    assert P.surface_error_budget_m == .06 and P.approach_budget_m == .03 and P.window_s == .4
    assert P.onset_acceleration_mps2 == 3. and P.snap_budget_m == .03
    assert set(REGISTERED_POLICIES) == {'corridor_v1'}


def test_evidence_needs_full_window_and_term_is_not_a_physical_bound():
    assert not feed([.35]*4)[1]['stable']
    out = feed([.35]*5)[1]
    assert out['stable'] and out['observed_rate_term_mps'] == pytest.approx(.225)
    assert out['rate_term_is_physical_bound'] is False
    assert feed([.35]*5, CorridorPolicy(surface_error_budget_m=0.))[1]['observed_rate_term_mps'] == pytest.approx(.075)


def test_approach_cancels_and_retreat_keeps_evidence():
    out = feed([.35, .35, .35, .35, .31])[1]
    assert not out['stable'] and out['approach_m'] == pytest.approx(.04)
    assert feed([.31, .32, .33, .34, .35])[1]['stable']


def test_same_scan_is_not_new_evidence():
    monitor = BoundaryMonitor(P)
    for _ in range(9):
        out = monitor.update(S(1.), face(.35), same_interval)
    assert not out['stable'] and len(monitor.rows) == 1


def test_gap_epoch_loss_and_other_face_reset_evidence():
    monitor = feed([.35]*5)[0]
    assert not monitor.update(S(.7), face(.35), same_interval)['stable'] and len(monitor.rows) == 1
    monitor = feed([.35]*5)[0]
    assert not monitor.update(S(.5, 1), face(.35), same_interval)['stable']
    monitor = feed([.35]*5)[0]
    assert not monitor.update(S(.5), None)['stable'] and monitor.rows == []
    monitor = feed([.35]*5)[0]
    assert not monitor.update(S(.5), face(.35, (2., 2.2)), same_interval)['stable']
    monitor = feed([.35]*5)[0]
    assert not monitor.update(S(.5), face(.35), lambda row: None)['stable']


def test_wall_frame_is_symmetric():
    config, wall = FollowConfig(), dict(slope=0., intercept_m=0.)
    points = np.array([[1., .35], [1., -.35]])
    assert wall_frame(points, config, 'left', wall)[1][0] == pytest.approx(.075)
    assert wall_frame(points, config, 'right', wall)[1][1] == pytest.approx(.075)


def test_side_face_uses_spread_median_and_rear_face_keeps_lowest():
    along = np.linspace(0., .2, 41)
    ids, points, along, normal = rows(along, .35+.03*np.sin(np.arange(41)*2.399))
    out = boundary_face(ids, points, along, normal, np.ones(41, bool), P)
    assert out['method'] == 'parallel_face_median' and out['normal_m'] == pytest.approx(.35, abs=.008)
    ids, points, along, normal = rows(np.full(30, 1.1)+np.linspace(-.01, .01, 30), np.linspace(.35, .50, 30))
    out = boundary_face(ids, points, along, normal, np.ones(30, bool), P)
    assert out['method'] == 'low_order' and out['normal_m'] == pytest.approx(.35)


def test_evidence_scope_excludes_returns_past_the_band_and_separate_objects():
    along, normal = np.r_[np.linspace(1., 1.2, 20), np.linspace(1.2, 1.3, 20)], np.r_[np.full(20, .35), np.linspace(.42, .60, 20)]
    ids, points, along, normal = rows(along, normal)
    assert set(boundary_face(ids, points, along, normal, np.ones(40, bool), P)['members']) <= set(range(20))
    along, normal = np.r_[np.linspace(1., 1.2, 20), np.linspace(1.8, 2.0, 20)], np.r_[np.full(20, .35), np.full(20, .45)]
    ids, points, along, normal = rows(along, normal)
    assert set(boundary_face(ids, points, along, normal, np.ones(40, bool), P)['members']) <= set(range(20))
    along, normal = np.r_[np.linspace(1., 1.2, 20), np.linspace(1.8, 2.0, 20)], np.full(40, .35)
    ids, points, along, normal = rows(along, normal)
    assert len(boundary_face(ids, points, along, normal, np.ones(40, bool), P)['members']) == 20


def test_outliers_stay_out_or_narrow_only():
    normal = np.full(41, .35)
    normal[20] = .60
    ids, points, along, normal = rows(np.linspace(0., .2, 41), normal)
    out = boundary_face(ids, points, along, normal, np.ones(41, bool), P)
    assert len(out['members']) == 40 and 20 not in set(out['members'])
    normal = np.full(41, .35)
    normal[0] = .07
    ids, points, along, normal = rows(np.linspace(0., .2, 41), normal)
    assert boundary_face(ids, points, along, normal, np.ones(41, bool), P)['normal_m'] == pytest.approx(.07)


def test_wall_returns_need_continuity_and_stay_near_the_line():
    along = np.linspace(-.3, 1.3, 80)
    normal = .03*np.sin(np.arange(80)*2.399)
    ids, points, along, normal = rows(along, normal)
    assert connected_wall_returns(ids, points, along, normal, P).all()
    keep = np.ones(80, bool)
    keep[[20, 40, 60]] = False
    assert not connected_wall_returns(ids[keep], points[keep], along[keep], normal[keep], P).any()
    moved = normal.copy()
    moved[30:45] = .15
    assert not connected_wall_returns(ids, points, along, moved, P)[30:45].any()


def test_side_choice_and_blocked_band():
    table = lambda left, right, a=.35, b=.35: dict(left=dict(width_m=left, inner_normal_m=a), right=dict(width_m=right, inner_normal_m=b))
    assert choose_side(table(.02, .045)) == 'right' and choose_side(table(.20, .02)) == 'left'
    config = FollowConfig()
    assert straight_blocked(table(0, 0), config) and straight_blocked(table(0, 0, .50, .20), config)
    assert not straight_blocked(table(0, 0, .60, .10), config) and not straight_blocked(table(0, 0, .78, .78), config)


def test_planned_offset_needs_a_current_eligible_row():
    intent = PlannedIntent()
    intent.forced_offset = -.27
    scene = SimpleNamespace(t_ref_s=1., source_first_s=.9, source_epoch=0)
    straight = dict(offset_m=0., transition_m=.8, eligible=True, target_speed_mps=.3, steering_rad=0., preview_return_progress_m=.8)
    blocked = dict(offset_m=-.27, transition_m=.8, eligible=False)
    selected = dict(offset_m=0., transition_m=.8, speed_mps=.3, steering_rad=0.)
    assert intent.select(scene, dict(candidates=[straight, blocked], reason='x', selected=selected))['selected']['offset_m'] == 0.
    assert intent.sign is None
    allowed = dict(blocked, eligible=True, target_speed_mps=.2, steering_rad=-.1, preview_return_progress_m=.5)
    assert intent.select(scene, dict(candidates=[straight, allowed], reason='x', selected=selected))['selected']['offset_m'] == -.27
    assert intent.sign == -1.


def test_abeam_age_follows_scan_order():
    ray_list = []
    for i in range(360):
        angle = -math.pi+i*math.tau/360
        ray_list.append(SimpleNamespace(status='return', direction_xy=(math.cos(angle), math.sin(angle)), acquired_at_s=i/3600))
    scene = SimpleNamespace(rays=ray_list, source_last_s=359/3600)
    assert abeam_age(scene, 'left', .08) == pytest.approx(.155, abs=.002)
    assert abeam_age(scene, 'right', .08) == pytest.approx(.105, abs=.002)


def inputs(now=.5, target_y=0.):
    physical = [dict(t=t, rear_pose=[-.0675, 0., 0.], speed=0., steering=0.) for t in (0., 1.2)]
    scan = render_scan(physical, now, target_y)
    history = MotionHistory(MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675, lidar_x_m=.06))
    for t in np.arange(.2, 1.21, .01):
        history.add_mean_wheel(float(t), 0.)
        history.add_gyro(float(t), 0.)
    return scan, history


def test_default_stays_off_and_selection_is_explicit():
    assert BypassExecutionConfig().corridor_policy == 'off'
    follow = BypassEntryFollow(allow_simulation_trial=True)
    assert type(follow.guide) is EntryGuide and follow.guide.padding_provider is None
    with pytest.raises(ValueError):
        BypassExecutionConfig(corridor_policy='on')
    with pytest.raises(ValueError):
        BypassExecutionConfig(corridor_policy=True)
    selected = BypassEntryFollow(execution=BypassExecutionConfig(corridor_policy='corridor_v1'), allow_simulation_trial=True)
    assert type(selected.guide) is CorridorGuide and selected.guide.padding_provider == selected.guide._padding


def test_old_g2_header_without_policy_still_replays_as_off():
    assert BypassExecutionConfig(**{key: value for key, value in vars(BypassExecutionConfig()).items()
                                    if key != 'corridor_policy'}).corridor_policy == 'off'


def test_default_padding_provider_matches_explicit_existing_function():
    scan, history = inputs()
    scene = motion_scene(scan, history, .5, timing_verified=True, rear_half_angle_deg=30.)
    implicit = trial._compare_entries(scene, .5, 0., 0., .02, stationary_boundaries=True)
    explicit = trial._compare_entries(scene, .5, 0., 0., .02, stationary_boundaries=True,
                                      padding_provider=trial.current_outer_wall_padding)
    assert equal_output(freeze(implicit), freeze(explicit))


def test_module_function_is_not_replaced_while_candidate_runs():
    original = trial.current_outer_wall_padding
    guide = BypassEntryFollow(execution=BypassExecutionConfig(corridor_policy='corridor_v1'), allow_simulation_trial=True).guide
    scan, history = inputs()
    guide.evaluate(motion_scene(scan, history, .5, timing_verified=True, rear_half_angle_deg=30.), history, 0., 0., .02)
    assert trial.current_outer_wall_padding is original


def test_reset_clears_corridor_evidence_and_selection():
    guide = BypassEntryFollow(execution=BypassExecutionConfig(corridor_policy='corridor_v1'), allow_simulation_trial=True).guide
    guide.monitors['left'].rows.append(dict(source=0., reference=.1, normal=.35, endpoints=None))
    guide.plan_detail = dict(chosen='left')
    guide.reset()
    assert guide.monitors['left'].rows == [] and guide.plan_detail is None and isinstance(guide.intent, PlannedIntent)


def test_corridor_trace_replays_and_status_stays_json_safe(tmp_path):
    from arena_autonomy.g2_entry_node import G2EntryNode
    scan, history = inputs()
    config, execution = FollowConfig(), BypassExecutionConfig(corridor_policy='corridor_v1')
    path = tmp_path/'g2.jsonl'
    node = G2EntryNode.__new__(G2EntryNode)
    node.motion = SimpleNamespace(history=history, last_meta={'reference_stamp_s': .5}, verified=True, counts={'resets': 0})
    node.follow = BypassEntryFollow(config, execution, allow_simulation_trial=True)
    node.follow_trace = FollowTrace(path, history.config, config, mode='g2_entry_trial', execution_config=execution)
    node.side, node.last_steering, node.settings, node.rear_blind_half_angle = 'left', 0., {'control_rate_hz': 50.}, 30.
    outputs = []
    for scan_time in (.5, .6, .7, .8, .9, 1., 1.1):
        for offset in (0., .02):
            node.scan = inputs(scan_time)[0]
            node.motion.last_meta['reference_stamp_s'] = scan_time+offset
            out = node.command(None)
            json.dumps({'state': 'X', **out[2]}, ensure_ascii=False, allow_nan=False)
            assert 0 <= out[0] <= .35
            outputs.append(out)
            node.last_steering = out[1]
    node.reset_follow('SENSOR_STOP')
    node.follow_trace.close()
    events = [json.loads(line) for line in path.read_text().splitlines()]
    assert events[0]['data']['execution_config']['corridor_policy'] == 'corridor_v1'
    report = replay_events(events)
    assert report['passed'] and report['matched'] == len(outputs)
    assert any(o[0] > 0 for o in outputs)


def test_default_launches_and_files_do_not_select_the_policy():
    root = Path(__file__).resolve().parents[1]
    for name in ('simulation.launch.py', 'demo.launch.py', 'local_pursuit.launch.py', 'g1_follow.launch.py'):
        assert 'g2_corridor_policy' not in (root/'src/arena_bringup/launch'/name).read_text()
    assert "declare_parameter('g2_corridor_policy','off')" in (root/'src/arena_autonomy/arena_autonomy/g2_entry_node.py').read_text(encoding='utf-8')
    assert LocalPursuit.__name__ == 'LocalPursuit'
