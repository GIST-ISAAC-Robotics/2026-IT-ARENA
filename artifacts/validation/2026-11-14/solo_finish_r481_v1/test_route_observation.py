"""원 r477 센서 prefix의 LocalFollow 경로/점유와 분리된 결정 순서. 전체plan 재생 아님."""
import copy,json,math
from pathlib import Path
import numpy as np
import pytest
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.race_runtime_policy import current_route_priority as policy

A=Path(__file__).resolve().parent
HEADER=json.loads((A/'header.json').read_bytes())
PREFIX=[r['submitted']['data']['input'] for r in json.loads((A/'focus_plans.json').read_bytes())
        if r['submitted']['data']['input']['reference_s']<=51.220001]


def scene_for(data):
    _,raw,history=restore_call(data,MotionConfig(**HEADER['motion_config']))
    scene=motion_scene(raw,history,data['reference_s'],timing_verified=data['timing_verified'],
        epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
    return scene,raw,history


def call(owner,data):
    scene,raw,history=scene_for(data)
    return owner.command(scene,history,data['speed_mps'],data['side'],
        data['previous_steering_rad'],data['dt_s'],raw_scan=raw)


def warm():
    owner=LocalFollow(FollowConfig(**HEADER['follow_config']))
    for data in PREFIX[:-1]:call(owner,data)
    return owner


def test_original_stall_input_has_current_path_and_existing_sweep_positive(monkeypatch):
    before=copy.deepcopy(PREFIX)
    observed=policy.observed_command
    monkeypatch.setattr(policy,'observed_command',lambda *a,**k:None)
    baseline=call(warm(),PREFIX[-1])
    monkeypatch.setattr(policy,'observed_command',observed)
    candidate=call(warm(),PREFIX[-1])
    assert baseline[0]==0 and baseline[2]['partial_observation']['reason']=='lateral_metric_outside_trial'
    assert candidate[0]>0 and candidate[2]['reason']=='local_follow'
    assert candidate[0]==candidate[2]['target_speed_before_hold_mps']
    assert candidate[2]['path_points']==baseline[2]['path_points']
    assert candidate[2]['swept_clearance_m']==baseline[2]['swept_clearance_m']
    assert candidate[2]['partial_observation']['reason']==policy.POLICY
    assert not candidate[2]['partial_observation']['unknown_filled']
    assert PREFIX==before


def test_current_path_decision_precedes_legacy_partial_lateral_gate(monkeypatch):
    owner=warm()
    original=owner.partial_policy.evaluate;reports=[]
    def observe(*a,**k):
        report=original(*a,**k);reports.append(report);return report
    monkeypatch.setattr(owner.partial_policy,'evaluate',observe)
    result=call(owner,PREFIX[-1])
    assert len(reports)==1 and reports[0]['allowed'] is False
    assert reports[0]['reason']=='lateral_metric_outside_trial'
    assert result[0]>.2 and result[2]['reason']=='local_follow'
    # 기존 기억/transport는 갱신하고 그 중복 lateral veto만 적용하지 않는다.
    scene,_,_=scene_for(PREFIX[-1])
    assert owner.partial_policy.structure_last_source==scene.source_first_s


def test_actual_front_occupied_sweep_still_stops():
    owner=warm();data=copy.deepcopy(PREFIX[-1]);scene,_,_=scene_for(data)
    index=min((r for r in scene.rays if r.status=='return'),key=lambda r:abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))).source_index
    data['scan']['measurements']['ranges_m'][index]=.08
    result=call(owner,data)
    assert result[0]==0
    assert result[2].get('swept_clearance_m',0)<=owner.config.hold_clearance_m


def test_total_current_front_returns_lost_still_stops():
    owner=warm();data=copy.deepcopy(PREFIX[-1]);scene,_,_=scene_for(data)
    for ray in scene.rays:
        if abs(math.atan2(ray.direction_xy[1],ray.direction_xy[0]))<=math.radians(100):
            data['scan']['measurements']['ranges_m'][ray.source_index]={'nonfinite':'inf'}
    assert call(owner,data)[0]==0


def test_actual_path_loss_still_stops():
    owner=warm();data=copy.deepcopy(PREFIX[-1])
    data['scan']['measurements']['ranges_m']=[{'nonfinite':'inf'} for _ in data['scan']['measurements']['ranges_m']]
    assert call(owner,data)[0]==0


def test_unselected_policy_does_not_take_new_decision():
    owner=warm();data=PREFIX[-1];scene,raw,history=scene_for(data)
    result=owner._path_command(scene,history,data['speed_mps'],data['side'],data['previous_steering_rad'],data['dt_s'],allow_partial=True)
    assert result[0]>0
    cfg=copy.deepcopy(owner.config);object.__setattr__(cfg,'guard_observation_policy','off')
    assert policy.observed_command(cfg,scene,result,raw,history.config.lidar_yaw_rad) is None


def test_near_invalid_return_not_excused_as_free():
    owner=warm();data=copy.deepcopy(PREFIX[-1]);scene,_,_=scene_for(data)
    index=min(scene.rays,key=lambda r:abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))).source_index
    data['scan']['measurements']['ranges_m'][index]=.001
    assert call(owner,data)[0]==0
