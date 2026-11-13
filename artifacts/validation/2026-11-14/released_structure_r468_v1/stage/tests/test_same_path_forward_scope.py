"""현재 v4 전방 증거를 같은 이동 경로에만 연결하며 별개 측방을 합산하지 않는다."""
from dataclasses import replace
import math

import numpy as np
import pytest

from arena_autonomy.scan_input import project_scan, scan_angles
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.traffic_recovery_trial import TimedPolicy
from arena_autonomy.same_path_gap_trial import bounded_same_path_gap
from test_forward_gap_scope import mixed
from test_local_follow import history
from test_partial_observation import sample


def fixture(representation='uniform',sign=1.):
    h=history()
    raw=mixed(representation,side=sign*32.,front=0.)
    policy=TimedPolicy(True,True,True)
    earlier=sample(.4)
    earlier.ranges=[2.]*len(earlier.ranges)
    points,_,_=project_scan(earlier,h,.5,timing_verified=True)
    policy.evaluate(earlier,points,h,.5,.2)
    points,_,_=project_scan(raw,h,.62,timing_verified=True)
    proof=policy.evaluate(raw,points,h,.62,.2)
    assert proof['allowed'] and proof['reason']=='bounded_forward_missing',proof
    scene=motion_scene(raw,h,.62,timing_verified=True)
    command=dict(now=.62,speed=0.,target=.2,steering=0.,previous=0.)
    return [scene,raw,np.array([[.14,0.]]),np.full(len(scene.rays),.1),.2,.15,proof,command]


@pytest.mark.parametrize('representation',['uniform','cw','ccw'])
@pytest.mark.parametrize('sign',[-1.,1.])
def test_actual_scoped_proof_reaches_same_path(representation,sign):
    args=fixture(representation,sign)
    before=tuple(args[1].ranges)
    result=bounded_same_path_gap(*args)
    assert result['accepted'],result
    assert not result['unknown_filled']
    assert tuple(args[1].ranges)==before


@pytest.mark.parametrize('change',[dict(forward_gap_deg=.1),dict(forward_gap_deg=1.51),
    dict(total_gap_deg=6.1),dict(forward_gap_lidar_yaw_rad=math.nan),
    dict(forward_gap_deg=True),dict(minimum_gap_neighbor_m=.99),dict(speed_cap_mps=.21)])
def test_invalid_or_inconsistent_scoped_proof_is_not_permission(change):
    args=fixture();args[-2].update(change)
    assert not bounded_same_path_gap(*args)['accepted']


@pytest.mark.parametrize('kind',['extra_front','rear','rotated_side','near_hit','wrong_source'])
def test_scope_rechecks_actual_unknown_geometry_and_current_hits(kind):
    args=fixture()
    scene,raw=args[:2]
    if kind in ('extra_front','rear','near_hit'):
        angle={'extra_front':-20.,'rear':110.,'near_hit':2.}[kind]
        index=int(np.argmin(abs(np.degrees(scan_angles(raw))-angle)))
        raw.ranges[index]=.1 if kind=='near_hit' else math.inf
        args[0]=motion_scene(raw,history(),.62,timing_verified=True)
    elif kind=='rotated_side':
        rays=tuple(replace(r,direction_xy=(1.,0.)) if r.status=='unknown' else r for r in scene.rays)
        args[0]=replace(scene,rays=rays)
    else:raw.header.frame_id='other'
    assert not bounded_same_path_gap(*args)['accepted']


def test_old_total_only_proof_does_not_silently_gain_larger_scope():
    args=fixture()
    for name in ('forward_gap_scope','forward_gap_deg','forward_gap_lidar_yaw_rad'):
        del args[-2][name]
    assert bounded_same_path_gap(*args)['reason']=='forward_proof_outside_scope'
