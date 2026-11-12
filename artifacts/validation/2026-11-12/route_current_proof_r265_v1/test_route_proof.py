import copy
from dataclasses import replace
from functools import lru_cache
import json
import math
from pathlib import Path
import pytest
from arena_autonomy.follow_trace import restore_call, freeze, equal_output
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scan_input import scan_angles
from arena_autonomy import guard_observation_policy as original
import route_proof
from test_current_path_recheck import fixture_for,SCOPES

A=Path(__file__).resolve().parent

@lru_cache()
def evidence():
    record=json.loads((A/'result.json').read_text())[0]
    positive=next(r for r in record['rows'] if r['speed']>0)
    with (A.parents[3]/record['trace']).open() as f:
        header=json.loads(next(f))['data']
        calls=[e['data'] for e in map(json.loads,f) if e['kind']=='call']
    old=next(c for c in calls if c['input']['reference_s']==positive['original_ref'])
    current=next(c['input'] for c in calls if c['input']['reference_s']==positive['current_ref'])
    return header,old,current

def example():
    header,old,current=copy.deepcopy(evidence())
    _,scan,history=restore_call(current,MotionConfig(**header['motion_config']))
    scene=motion_scene(scan,history,current['reference_s'],timing_verified=current['timing_verified'],
                       epoch=current['epoch'],rear_half_angle_deg=current['rear_half_angle_deg'])
    return old['output'],old['input'],current,scene,history,FollowConfig(**header['follow_config'])

def test_fresh_current_region_proves_old_route_without_restamping_or_mutating():
    args=example();before=copy.deepcopy(args[:3])
    assert original.rebase(*args)['speed_mps']==0
    result=route_proof.rebase(*args)
    assert result['speed_mps']>0
    assert result['path_source_s']==args[0][2]['normal']['path_source_s']
    assert result['current_route_proof']['accepted']
    assert result['current_route_proof']['region']['unobserved']==0
    assert result['current_route_proof']['current_evidence_first_s']==args[3].source_first_s
    assert result['current_route_proof']['source_timestamp_renewed'] is False
    assert result['current_route_proof']['old_occupancy_permission_reused'] is False
    assert args[:3]==before

@pytest.mark.parametrize('fault',('expired_geometry','motion_epoch','sensor_epoch','side','unknown_front','occupied_front'))
def test_missing_new_evidence_does_not_renew_route(fault):
    out,old,current,scene,history,cfg=example()
    if fault=='expired_geometry':out[2]['normal']['path_source_s']-=.1
    elif fault=='motion_epoch':current['epoch']+=1
    elif fault=='sensor_epoch':current['scan']['measurements']['source_epoch']+=1
    elif fault=='side':current['side']='right' if current['side']=='left' else 'left'
    else:
        _,scan,_=restore_call(current,history.config)
        angles=scan_angles(scan)+history.config.lidar_yaw_rad
        distances=list(scan.ranges_m)
        for i,angle in enumerate(angles):
            if abs(math.remainder(angle,math.tau))<.35:
                distances[i]=math.inf if fault=='unknown_front' else scan.range_min_m+.01
        scan=replace(scan,ranges_m=tuple(distances))
        current['scan']['measurements']['ranges_m']=distances
        scene=motion_scene(scan,history,current['reference_s'],timing_verified=True,
                           epoch=current['epoch'],rear_half_angle_deg=current['rear_half_angle_deg'])
    result=route_proof.rebase(out,old,current,scene,history,cfg)
    assert result['speed_mps']==0
    assert not result.get('current_route_proof',{}).get('accepted')

@pytest.mark.parametrize('scope',SCOPES)
def test_nonexpired_existing_output_is_identical(scope):
    args=fixture_for(scope)
    assert equal_output(freeze(route_proof.rebase(*args)),freeze(original.rebase(*args)))

def test_new_zero_plan_is_not_revived():
    out=example()[0];out[0]=0
    assert route_proof.rebase(out,None,None,None,None,None) is None
