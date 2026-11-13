"""r464 국소 경계 갱신 회귀. 전체 Evaluator/주행 재현으로 취급하지 않는다.

run_focused_tests.py가 r457 원본 package와 변경 local_follow만 연결한다.
원 plan639의 기록 경계는 명시적 국소 fixture이며 전체 worker 상태가 아니다.
"""
from dataclasses import replace
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest

import arena_autonomy.local_follow as candidate
from arena_autonomy.follow_trace import restore_call, equal_output, freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_scene import SensorRay
from arena_autonomy.scene_motion import motion_scene

HERE=Path(__file__).resolve().parents[2]
NAME='arena_autonomy.r464_original_local_follow'
spec=importlib.util.spec_from_file_location(NAME,
    HERE/'original/src/arena_autonomy/arena_autonomy/local_follow.py')
original=importlib.util.module_from_spec(spec)
sys.modules[NAME]=original
spec.loader.exec_module(original)


@pytest.fixture
def inputs():
    with (HERE.parent/'solo_full_lap_r457_v1/run1/follow_decisions.jsonl').open(encoding='utf-8') as stream:
        header=json.loads(next(stream))['data']
    rows=[json.loads(line) for line in (HERE/'saved_plan_window.jsonl').read_text().splitlines()]
    raw=next(row['data']['input'] for row in rows
             if row['kind']=='runtime_plan_submitted' and row['data']['sequence']==641)
    previous=next(row['data']['output'][2]['normal'] for row in rows
             if row['kind']=='runtime_plan_completed' and row['data']['sequence']==639)
    data,scan,history=restore_call(raw,MotionConfig(**header['motion_config']))
    scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
        epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
    return header,data,scene,history,previous


def prepared(module,inputs,changes=None):
    header,data,scene,history,previous=inputs
    cfg=module.FollowConfig(**{**header['follow_config'],**(changes or {})})
    ctl=module.LocalFollow(cfg)
    ctl.path=np.asarray(previous['path_points']).copy()
    ctl.path_ref=previous['reference_stamp_s']
    ctl.path_source=previous['path_source_s']
    ctl.path_wall=previous['path_wall']
    ctl.last_ref=previous['reference_stamp_s']
    ctl.last_source=previous['source_first_s']
    ctl.epoch=scene.source_epoch
    ctl.side=data['side']
    ctl.held=False
    return ctl


def call(ctl,inputs,scene=None):
    _,data,saved,history,_=inputs
    return ctl._baseline_command(saved if scene is None else scene,history,
        data['speed_mps'],data['side'],data['previous_steering_rad'],data['dt_s'],allow_partial=True)


def test_recorded_same_wall_is_refreshed_from_current_points(inputs):
    before=call(prepared(original,inputs),inputs)
    ctl=prepared(candidate,inputs)
    after=call(ctl,inputs)
    assert before[0]>0 and before[2]['boundary_from_memory']
    assert not before[2]['fresh_boundary_accepted']
    assert after[0]>0 and after[2]['reason']=='local_follow'
    assert after[2]['boundary_history_agreement']=='current_same_wall_corner_arc'
    assert after[2]['fresh_boundary_accepted'] and not after[2]['boundary_from_memory']
    assert after[2]['unfiltered_occupancy']
    assert after[2]['occupied_points']==len(inputs[2].occupied_endpoints_xy_m)
    assert ctl.path_source==inputs[2].source_first_s
    assert after[2]['swept_clearance_m']>ctl.config.hold_clearance_m


@pytest.mark.parametrize('case', ['profile_off','expired','other_wall','displaced','same_scan','epoch'])
def test_non_target_keeps_original_result(inputs,case):
    changes={'expiry_route_reference_trial':False} if case=='profile_off' else None
    controllers=[prepared(module,inputs,changes) for module in (original,candidate)]
    for ctl in controllers:
        if case=='expired':ctl.path_source=inputs[1]['reference_s']-.31
        if case=='other_wall':ctl.path_wall='right'
        if case=='displaced':ctl.path=ctl.path+np.array([0.,.20])
        if case=='same_scan':ctl.last_source=inputs[2].source_first_s
        if case=='epoch':ctl.epoch=inputs[2].source_epoch+1
    before,after=[call(ctl,inputs) for ctl in controllers]
    assert equal_output(freeze(before),freeze(after),tolerance=1e-12)
    assert after[2].get('boundary_history_agreement')!='current_same_wall_corner_arc'


def test_refresh_keeps_real_occupied_points_in_hold_check(inputs):
    scene=inputs[2]
    rays=list(scene.rays)
    for i,y in enumerate(np.linspace(-.075,.075,20)):
        hit=(.10,float(y))
        direction=np.asarray(hit)-np.array([.0754,0.])
        direction/=np.linalg.norm(direction)
        rays.append(SensorRay((.0754,0.),hit,scene.source_first_s,'return',scene.source_frame_id,
            scene.source_epoch,1000+i,tuple(direction)))
    added_angles=tuple(math.atan2(r.direction_xy[1],r.direction_xy[0]) for r in rays[len(scene.rays):])
    blocked=replace(scene,rays=tuple(rays),source_angles_rad=scene.source_angles_rad+added_angles)
    out=call(prepared(candidate,inputs),inputs,blocked)
    assert out[0]==0 and out[2]['reason']=='follow_gap_hold'
    assert out[2]['occupied_points']==len(blocked.occupied_endpoints_xy_m)


def test_prior_hold_still_requires_existing_release_confirmation(inputs):
    ctl=prepared(candidate,inputs)
    ctl.held,ctl.hold_reason,ctl.release_count=True,'follow_gap_hold',0
    out=call(ctl,inputs)
    assert out[0]==0 and out[2]['reason']=='follow_release_wait'
    assert ctl.release_count==1
