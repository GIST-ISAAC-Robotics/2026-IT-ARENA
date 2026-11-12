"""저장 기하의 같은 입력과 위험 반례. 실제 구동 검증이 아니다."""
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace
from dataclasses import replace
import numpy as np
import pytest
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.pass_release_trial import PassReleaseSelector, motion_region

A = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('candidate_r164_test', A/'candidate.py')
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)


@pytest.fixture(scope='module')
def saved():
    trace = A.parent/'adjacent_live_r152_v1/two_actor_run2/run1/follow_decisions.jsonl'
    events = [json.loads(s) for s in trace.open()]
    header = events[0]['data']
    data, scan, history = restore_call(next(e for e in events if e['seq'] == 632)['data']['input'],
                                     MotionConfig(**header['motion_config']))
    scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
        epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
    follow = FollowConfig(**header['follow_config'])
    selector = PassReleaseSelector(follow)
    context = {}
    motion_region(scene, data['speed_mps'], .15, .1914960850180926, data['previous_steering_rad'],
                  data['dt_s'], follow, selector.lat, context=context)
    q = context['queries'][~context['free'] & ~context['masked']]
    return scene, scan, history, q, context['padding']


def check(saved, **kw):
    scene, scan, _, q, pad = saved
    args = dict(scene=scene, scan=scan, queries=q, padding=pad, length=.2, width=.15, speed=0., target=.15)
    args.update(kw)
    return c.body_gap(**args)


def test_observed_neighbors_outside_current_padded_envelope(saved):
    r = check(saved)
    assert r['accepted'] and not r['unknown_filled']
    assert r['exterior_max_m'] < .007
    assert r['minimum_neighbor_range_m'] < .5438955426216125
    assert r['local_chord_m'] <= .05


@pytest.mark.parametrize('speed', [.151, -.001, math.inf, math.nan])
def test_actual_speed_is_not_replaced_by_target(saved, speed):
    assert not check(saved, speed=speed)['accepted']


def test_larger_exterior_and_front_progress_are_excluded(saved):
    q = saved[3].copy(); q[:, 1] += .011
    assert not check(saved, queries=q)['accepted']
    q = saved[3].copy(); q[:, 0] = .101
    assert not check(saved, queries=q)['accepted']


def test_existing_age_padding_can_reject_neighbor(saved):
    assert not check(saved, padding=np.full(len(saved[4]), .6))['accepted']


def test_raw_gap_cannot_be_nan_or_near_return(saved):
    for replacement in (math.nan, .01, -math.inf):
        ranges = list(saved[1].ranges_m); ranges[111] = replacement
        scan = replace(saved[1], ranges_m=tuple(ranges))
        assert not check(saved, scan=scan)['accepted']


def test_other_front_missing_is_not_hidden(saved):
    scene = saved[0]
    index = next(i for i, r in enumerate(scene.rays) if r.status == 'return' and abs(math.atan2(r.direction_xy[1], r.direction_xy[0])) < .1)
    rays = list(scene.rays); rays[index] = replace(rays[index], status='unknown', hit_xy_m=None)
    assert not check(saved, scene=replace(scene, rays=tuple(rays)))['accepted']


def test_old_obstacle_and_expired_history_are_kept(saved):
    scene, _, history, q, _ = saved
    p = SimpleNamespace(latched=False, previous=(np.asarray([q[0]]), scene.t_ref_s, scene.source_first_s))
    assert not c.history_check(p, scene, history, q, scene.occupied_endpoints_xy_m)['accepted']
    p.previous = (np.empty((0,2)), scene.t_ref_s-.301, scene.source_first_s-.301)
    assert not c.history_check(p, scene, history, q, scene.occupied_endpoints_xy_m)['accepted']
    p.latched = True
    assert c.history_check(p, scene, history, q, scene.occupied_endpoints_xy_m)['reason'] == 'known_latch_preserved'
