"""멀리 있는 점의 회전 영향과 실제 근접/차체 변화의 분리. 물성·팽창 완화 없음."""
from dataclasses import replace
import math
import sys
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.local_scene import LocalScene, SensorRay
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.measured_parallel_trial import MeasuredParallelTrial
from arena_autonomy.fresh_parallel_trial import FreshMemoryParallelTrial, geometry_check, snapshot
import importlib.util
_spec=importlib.util.spec_from_file_location('_fresh_body_offline_test', ROOT/'artifacts/validation/2026-10-30/fresh_confirmation_body_v2/candidate_state.py')
_offline=importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_offline)
legacy=_offline.legacy

CFG = FollowConfig(partial_observation_policy='bounded_local_v4', boundary_evidence_mode='corner_distance_candidate',
 occupancy_timing_mode='per_ray_candidate', parallel_hold_trial=True, pass_release_trial=True,
 guard_curve_trial=True, traffic_recovery_trial=True, traffic_path_recovery_trial=True,
 corridor_fallback_enabled=True, rear_lane_keep_trial=True, rear_lane_keep_policy='observed_measured_v1')
BASELINE = (1., 0., dict(reason='local_follow', path_points=[[x/10, 0.] for x in range(11)],
                       preferred_wall='left', partial_observation=dict(allowed=True, reason='fully_observed')))


def scene(source=0., now=.10, angle=0., dx=0., dy=0., y=-.2152, front=1.4):
    pts = [(x, y, .02) for x in (-.09, -.06, -.03, 0., .03, .06, .09)]
    pts += [(front, 0., .05), (9., .8, .08)]
    c, s = math.cos(angle), math.sin(angle)
    rotation = np.array([[c,s],[-s,c]])
    rays = []
    for i, (x, y, t) in enumerate(pts):
        raw = np.array([x,y]); direction = raw/np.linalg.norm(raw)
        rays.append(SensorRay((dx,dy), tuple(raw@rotation+[dx,dy]), source+t, 'return', 'laser_frame',0,i,tuple(direction@rotation)))
    return LocalScene(now, rays[0].acquired_at_s, rays[-1].acquired_at_s, 'laser_frame',0,tuple(rays))


def apply(t, sc, previous=0., baseline=BASELINE):
    return t.apply(sc, baseline, 0., previous, .02)


def test_far_rotation_preserves_confirmation_but_current_hold_and_next_fresh_checks_remain():
    a, b = legacy.FreshMemoryParallelTrial(CFG), FreshMemoryParallelTrial(CFG)
    for t in (a,b): apply(t, scene())
    ra, rb = [apply(t, scene(now=.20, angle=.0015)) for t in (a,b)]
    assert ra[0] == rb[0] == 0.
    assert not ra[2]['fresh_confirmation_memory']['kept']
    assert rb[2]['fresh_confirmation_memory']['kept']
    assert rb[2]['fresh_confirmation_memory']['body_geometry']['body_corner_displacement_m'] < .0003
    assert apply(a,scene(source=.1,now=.20))[0] == 0.
    assert apply(b,scene(source=.1,now=.20))[0] > 0.


@pytest.mark.parametrize('dx,angle', [(.014,0.),(0.,.06),(.004,.025)])
def test_actual_body_motion_discards(dx,angle):
    t=FreshMemoryParallelTrial(CFG); apply(t,scene())
    g=geometry_check(snapshot(scene(now=.20,dx=dx,angle=angle)),snapshot(scene()),CFG.length_m,CFG.width_m)
    assert not g['small_body_motion']
    out=apply(t,scene(now=.20,dx=dx,angle=angle))
    if 'fresh_confirmation_memory' in out[2]:
        assert not out[2]['fresh_confirmation_memory']['kept']


@pytest.mark.parametrize('change', ['hit','direction','index','status','origin','frame'])
def test_changed_same_timestamp_scan_is_not_accepted(change):
    first=scene(); current=scene(now=.20,angle=.0015); rays=list(current.rays)
    r=rays[2]
    if change=='hit': r=replace(r,hit_xy_m=(r.hit_xy_m[0]+.0002,r.hit_xy_m[1]))
    elif change=='direction': r=replace(r,direction_xy=(1.,0.))
    elif change=='index': r=replace(r,source_index=100)
    elif change=='status': r=replace(r,status='unknown',hit_xy_m=None)
    elif change=='origin': r=replace(r,ray_origin_xy_m=(.001,0.))
    elif change=='frame':
        rays=[replace(x,source_frame_id='other') for x in rays]
        current=replace(current,source_frame_id='other',rays=tuple(rays)); r=None
    if r is not None:
        rays[2]=r; current=replace(current,rays=tuple(rays))
    g=geometry_check(snapshot(current),snapshot(first),CFG.length_m,CFG.width_m)
    assert not (g['same_raw_scan'] and g['rigid_scan'])
    t=FreshMemoryParallelTrial(CFG); apply(t,first)
    out=apply(t,current)
    assert not out[2]['fresh_confirmation_memory']['kept'] and t.confirmations==0


@pytest.mark.parametrize('front,y',[(.35,-.2152),(1.4,-.165),(.40,-.12)])
def test_new_fresh_obstacle_clears_immediately(front,y):
    t=FreshMemoryParallelTrial(CFG); apply(t,scene()); apply(t,scene(now=.20,angle=.0015))
    out=apply(t,scene(source=.1,now=.20,front=front,y=y))
    assert out[0]==0 and t.confirmations==0 and t.confirmed is None


@pytest.mark.parametrize('event',['gap','epoch','reverse','steering','wall','reset'])
def test_memory_invalidations(event):
    t=FreshMemoryParallelTrial(CFG); apply(t,scene()); apply(t,scene(now=.20,angle=.0015))
    if event=='gap': out=apply(t,scene(source=.3,now=.4))
    elif event=='epoch':
        sc=scene(source=.1,now=.2); sc=replace(sc,source_epoch=1,rays=tuple(replace(r,source_epoch=1) for r in sc.rays)); out=apply(t,sc)
    elif event=='reverse': out=apply(t,scene(now=.18))
    elif event=='steering': out=apply(t,scene(now=.20),previous=.02)
    elif event=='wall': out=apply(t,scene(now=.20),baseline=(1.,0.,{**BASELINE[2],'preferred_wall':'right'}))
    else: t.reset(); out=apply(t,scene(source=.1,now=.2))
    assert out[0]==0 and t.confirmations<=1


def test_duplicate_scan_does_not_count_or_extend_window():
    t=FreshMemoryParallelTrial(CFG); apply(t,scene())
    for now in (.18,.20,.22,.24):
        assert apply(t,scene(now=now,angle=.0015))[0]==0 and t.confirmations==1
    assert t.confirmed['t_ref']==.10
    out=apply(t,scene(now=.26,angle=.0015))
    assert out[0]==0 and t.confirmations==0


def test_unaffected_trace_matches_runtime():
    a,b=MeasuredParallelTrial(CFG),FreshMemoryParallelTrial(CFG)
    for k in range(40):
        sc=scene(source=(k//5)*.1,now=(k//5)*.1+.10+(k%5)*.02,y=-.35)
        assert apply(a,sc)==apply(b,sc)


def test_explicit_policy_selects_new_state_and_retains_default():
    from arena_autonomy.local_follow import LocalFollow
    from arena_autonomy.measured_lane_preference_trial import StraightLanePreferenceSelector
    assert FollowConfig().rear_lane_keep_policy=='timed' and not FollowConfig().rear_lane_keep_trial
    follow=LocalFollow(replace(CFG,rear_lane_keep_policy='observed_fresh_v2'))
    assert isinstance(follow.parallel_trial,FreshMemoryParallelTrial)
    assert isinstance(follow.pass_selector,StraightLanePreferenceSelector)
    assert type(LocalFollow(CFG).parallel_trial) is MeasuredParallelTrial
    with pytest.raises(ValueError): FollowConfig(rear_lane_keep_policy='observed_fresh_v2')
