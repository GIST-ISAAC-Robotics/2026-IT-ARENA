"""실제 합성 광선으로 G1 부분 관측 허가와 GUARD의 연결을 대조한다. 폐루프 주행이 아니다."""
from dataclasses import replace
import json
import math
import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.follow_trace import FollowTrace, replay_events
from test_g1_partial_policy import corridor
from test_local_follow import history


def config(enabled=True):
    return FollowConfig(partial_observation_policy='bounded_local_v4',
        boundary_evidence_mode='corner_distance_candidate', occupancy_timing_mode='per_ray_candidate',
        corridor_fallback_enabled=True, parallel_hold_trial=True, pass_release_trial=True,
        guard_curve_trial=enabled)


def rays(first, side=1., missing=(), front=None, side_distance=.22):
    raw = corridor(first, obstacle=front)
    # 옆차의 가까운 평면 y=±거리, 길이20cm. 첫 반환을 갱신하므로 벽/앞차를 투과하지 않는다.
    angles = raw.angle_min+np.arange(500)*raw.angle_increment
    for i, angle in enumerate(angles):
        s = math.sin(angle)
        if s*side > 0:
            distance = side*side_distance/s
            x = .06+distance*math.cos(angle)
            if -.05 <= x <= .15:
                raw.ranges[i] = min(raw.ranges[i], distance)
    for i in missing:
        raw.ranges[i] = math.inf
    return raw


def call(ctl, raw, h, *, previous=0., speed=0.):
    now = raw.header.stamp.sec+raw.header.stamp.nanosec*1e-9+.1
    scene = motion_scene(raw, h, now, timing_verified=True)
    return ctl.command(scene, h, speed, 'left', previous, .02, raw_scan=raw)


def warmed(side=1., enabled=True):
    ctl, h = LocalFollow(config(enabled)), history()
    for t in (.2, .3, .4, .5, .6, .7, .8, .9):
        out = call(ctl, rays(t, side), h)
    assert out[0] > 0 and ctl.pass_selector.mode == 'GUARD'
    return ctl, h


@pytest.mark.parametrize('side,missing', [(1.,125),(-1.,375)])
def test_allowed_gap_uses_same_baseline_and_preserves_side_threat(side, missing):
    ctl, h = warmed(side)
    raw = rays(1., side, [missing])
    out = call(ctl, raw, h)
    assert 0 < out[0] <= .35
    info = out[2]['selector_guard_candidate']
    assert info['baseline_partial_reason'] == 'bounded_side_missing'
    assert info['path'] == 'baseline_g1' and info['steering_rad'] == out[1]
    assert info['accepted'] and info['region']['unobserved'] == 0
    assert out[2]['selector_choice'] == 'guard_baseline' and ctl.pass_selector.mode == 'GUARD'
    assert ctl.pass_selector.sides['left' if side>0 else 'right']['present']
    assert not out[2]['selector_unknown_filled'] and math.isinf(raw.ranges[missing])


def test_flag_off_preserves_old_stop_and_default_is_off():
    ctl, h = warmed(enabled=False)
    out = call(ctl, rays(1., missing=[125]), h)
    assert out[0] == 0 and out[2]['reason'] == 'parallel_hold_partial_observation_outside_trial'
    assert not FollowConfig().guard_curve_trial


@pytest.mark.parametrize('missing,front', [(range(118,130),None),([125],.25),([250],None)])
def test_wide_gap_front_stop_and_unobserved_actual_motion_stay_blocked(missing, front):
    ctl, h = warmed()
    out = call(ctl, rays(1., missing=missing, front=front), h)
    assert out[0] == 0
    if 250 in missing:
        # G1의 전방 소수 결측 허가만으로 GUARD 이동 영역의 미관측을 지우지 않는다.
        assert out[2]['partial_observation']['allowed']
        assert 'unobserved' in out[2]['selector_reason']


@pytest.mark.parametrize('fault', ['under_range','missing_raw','stale_raw','reverse','known_lost'])
def test_input_faults_and_known_lost_obstacle_are_not_hidden(fault):
    ctl, h = warmed()
    raw = rays(1., missing=[125])
    if fault == 'under_range':
        raw.ranges[125] = .01
    if fault == 'known_lost':
        ctl.partial_policy._latch('known_lost_test', .95)
    if fault in ('missing_raw','stale_raw'):
        scene = motion_scene(raw, h, 1.1, timing_verified=True)
        out = ctl.command(scene, h, 0., 'left', raw_scan=None if fault=='missing_raw' else rays(.9))
    else:
        out = call(ctl, raw, h, speed=-.01 if fault=='reverse' else 0.)
    assert out[0] == 0
    assert out[2]['selector_choice'] != 'guard_baseline'


def test_fresh_raw_trace_replays_new_coupling(tmp_path):
    ctl, h = LocalFollow(config()), history()
    path = tmp_path/'calls.jsonl'
    writer = FollowTrace(str(path), h.config, ctl.config)
    for i in range(12):
        t = .2+.1*i
        raw = rays(t, missing=[125] if i>=8 else [])
        out = call(ctl, raw, h)
        writer.record_call(raw,h,t+.1,0,0.,'left',0.,.02,True,30.,out)
    writer.close()
    assert out[0] > 0 and out[2]['selector_choice'] == 'guard_baseline'
    replay = replay_events(json.loads(line) for line in path.read_text().splitlines())
    assert replay['passed'] and replay['calls'] == 12
