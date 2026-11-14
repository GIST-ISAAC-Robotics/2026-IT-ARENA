"""독립 guard의 명시 현재 관측 정책. 합성 계약이며 실차/ROS 실행 증거가 아니다."""
import ast
import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import time

import numpy as np
import pytest

from arena_autonomy.independent_guard_route import front_evidence
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory, MotionUnavailable
from arena_autonomy.local_path import mask_scan, swept_limit, braking_speed
from arena_autonomy.partial_observation import PartialObservationPolicy
from arena_autonomy.known_structure import structure_policy
from arena_autonomy.scan_input import project_scan, scan_first_stamp
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy


def stamp(value):
    return NS(sec=int(value), nanosec=round((value-int(value))*1e9))


def scan(first=.5, ranges=None):
    return NS(header=NS(frame_id='laser_frame', stamp=stamp(first)),
        ranges=[8.]*500 if ranges is None else list(ranges), angle_min=-math.pi,
        angle_increment=math.tau/500, time_increment=.0002, range_min=.05, range_max=12.)


def history():
    h=MotionHistory(MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675,
                                lidar_x_m=.06, history_s=5.))
    for t in np.arange(0., 3.0001, .01):
        h.add_mean_wheel(float(t), 0.)
        h.add_gyro(float(t), 0.)
    return h


def policy(base=PartialObservationPolicy, selected=True):
    p=base(allow_narrow_forward=True, surface_translation_enabled=True,
           lateral_metric_enabled=True, oblique_metric_enabled=True)
    if selected:
        p.command_observation_priority=True
    return p


def evaluate(p, raw, h=None, now=None, speed=.8, points=None, **kwargs):
    h=history() if h is None else h
    now=scan_first_stamp(raw)+.1 if now is None else now
    if points is None:
        points=project_scan(raw,h,now,timing_verified=True)[0]
    return p.evaluate(raw,points,h,now,speed,**kwargs)


@pytest.mark.parametrize('kind',['self_only','rear_only','far_external_same_angle'])
def test_front_evidence_after_distance_limited_self_mask(kind):
    values=np.full(500,np.nan)
    values[0]=2. # 후방 반환은 전방 증거가 아니다.
    if kind!='rear_only':
        values[250]=.10 if kind=='self_only' else 2.
    raw=scan(ranges=values)
    before=np.asarray(raw.ranges).copy()
    masked=mask_scan(raw,dict(rear_half_angle_deg=0.,sectors_deg=[[0.,3.]],
                             range_limits_m=[[.05,.15]]))
    result=front_evidence(masked,0.)
    assert result['accepted'] == (kind=='far_external_same_angle')
    assert result['unknown_filled'] is False
    np.testing.assert_array_equal(raw.ranges,before)
    if kind=='far_external_same_angle': assert masked.ranges[250]==2.


@pytest.mark.parametrize('kind,reason',[
    ('front_lost','front_unobserved'),('below_min','invalid_near_return'),
    ('metadata_hole','angular_metadata_hole')])
def test_current_input_failure_is_not_replaced_by_permission(kind,reason):
    raw=scan()
    if kind=='front_lost': raw.ranges=[float('nan')]*500
    elif kind=='below_min': raw.ranges[250]=.01
    else: raw.angle_increment=math.pi/500
    assert front_evidence(raw,0.)['reason']==reason
    result=evaluate(policy(),raw,points=[])
    assert not result['allowed']


def test_partial_observation_has_no_global_gap_speed_or_empty_history_veto():
    raw=scan(); raw.ranges[315:370]=[float('nan')]*55
    selected=policy(); original=policy(selected=False)
    before=np.asarray(raw.ranges).copy()
    result=evaluate(selected,raw,speed=1.5)
    assert result['allowed'] and result['speed_cap_mps'] is None
    assert not evaluate(original,raw,speed=1.5)['allowed']
    assert selected.previous is not None and selected.last_source==.5
    np.testing.assert_array_equal(raw.ranges,before)


def test_actual_missing_near_point_latches_and_does_not_expire_into_permission():
    p=policy(); h=history()
    evaluate(p,scan(.3),h,points=[[.06,.15]],now=.4)
    raw=scan(.5); raw.ranges[375]=float('nan')
    result=evaluate(p,raw,h,points=[],now=.6)
    assert not result['allowed'] and p.latch_reason=='previous_point_in_missing_sweep'
    raw=scan(1.); raw.ranges[375]=float('nan')
    later=evaluate(p,raw,h,points=[],now=1.1)
    assert not later['allowed'] and p.latched
    assert p.last_source==1. # 거절 중에도 실제 fresh 관측은 기록한다.


def test_reobserved_point_and_two_full_scans_keep_existing_recovery_sequence():
    p=policy(); h=history()
    evaluate(p,scan(.3),h,points=[[.06,.15]],now=.4)
    raw=scan(.5); raw.ranges[375]=float('nan')
    result=evaluate(p,raw,h,points=[[.06,.15]],now=.6)
    assert result['allowed'] and result['reobserved_previous_points']==1
    p._latch('previous_point_in_missing_sweep',.6)
    assert not evaluate(p,scan(.7),h,points=[],now=.8)['allowed']
    assert evaluate(p,scan(.9),h,points=[],now=1.)['allowed']


def test_timed_recovery_uses_actual_projection_and_updates_fresh_partial_scan():
    p=policy(RecoveryPolicy); h=history()
    raw=scan(.5); raw.ranges[315:370]=[float('nan')]*55
    result=evaluate(p,raw,h)
    assert result['allowed']
    assert p.previous is not None and p.current_times is not None
    assert not evaluate(p,raw,h,points=[[1.,0.]])['allowed']


def test_timed_recovery_keeps_actual_disappeared_point_latched():
    p=policy(RecoveryPolicy); h=history()
    previous=scan(.3,[2.]*500); previous.ranges[375]=.15
    assert evaluate(p,previous,h,now=.4,speed=.4)['allowed']
    missing=scan(.5,[2.]*500); missing.ranges[375]=float('nan')
    result=evaluate(p,missing,h,now=.6,speed=.4)
    assert not result['allowed'] and p.latched
    assert p.latch_reason=='previous_point_in_missing_sweep'
    assert p.previous[2]==.5


def test_virtual_shadow_cap_removed_but_actual_hidden_memory_kept():
    cls=structure_policy(PartialObservationPolicy,'[[0.0,3.0]]')
    p=policy(cls); h=history()
    p.structure_previous=(np.array([[.2,0.]]),.5)
    def masked(first):
        return mask_scan(scan(first),dict(rear_half_angle_deg=0.,sectors_deg=[[0.,3.]]))
    result=evaluate(p,masked(.5),h,now=.6,points=[],speed=1.)
    detail=result['known_structure']
    assert detail['shadow_cap_mps'] is None
    assert detail['virtual_shadow_cap_enabled'] is False
    assert detail['actual_hidden_memory_cap_enabled'] is True
    assert detail['hidden_memory_points']==1 and detail['memory_cap_mps']<.3
    later=evaluate(p,masked(.9),h,now=1.,points=[],speed=1.)
    assert later['known_structure']['hidden_memory_points']==1
    assert later['known_structure']['memory_cap_mps']<.3


def control_method():
    # 실제 control 본문을 실행한다. ROS transport/node 구성만 제외하며 제어 로직을
    # 복제하거나 더미 판정으로 대체하지 않는다. Linux에서도 같은 비ROS 회귀가 된다.
    import arena_autonomy
    source=next(Path(p)/'lidar_safety.py' for p in arena_autonomy.__path__
                if (Path(p)/'lidar_safety.py').is_file())
    tree=ast.parse(source.read_text(encoding='utf-8'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='LidarSafety')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='control')
    def message(): return NS(header=NS(stamp=stamp(0),frame_id=''),drive=NS(speed=0.,steering_angle=0.))
    namespace=dict(np=np,math=math,time=time,json=json,MotionUnavailable=MotionUnavailable,
        AckermannDriveStamped=message,String=lambda **kw:NS(**kw),swept_limit=swept_limit,
        braking_speed=braking_speed,seconds=lambda s:s.sec+s.nanosec*1e-9)
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),namespace)
    return namespace['control'],namespace


def node(raw=None,speed=.8,steering=0.,previous=.4,partial=True):
    h=history(); raw=scan() if raw is None else raw
    outputs=[];statuses=[]
    timer=NS(nanoseconds=600000000,to_msg=lambda:stamp(.6))
    p=policy(RecoveryPolicy) if partial else None
    def project(raw):
        points,ids,_=project_scan(raw,h,.6,timing_verified=True)
        return points,len(ids)
    n=NS(get_clock=lambda:NS(now=lambda:timer),last_clock=.59,
        command=NS(header=NS(stamp=stamp(.5)),drive=NS(speed=speed,steering_angle=steering)),
        scan=raw,scan_wall=time.monotonic(),command_wall=time.monotonic(),wall_timeout=3.,
        limit=.458,wheelbase=.135,length=.20,width=.15,previous_safe_steering=previous,
        partial_policy=p,command_observation_priority=True,recent_observation_enabled=False,
        motion=NS(history=h,project=project,drain_pending=lambda:None),signed_motion_policy='forward_drive',
        vehicle_marker_channel=NS(profile=None,snapshot=lambda _:None,reset=lambda:None),
        publisher=NS(publish=outputs.append),status=NS(publish=statuses.append))
    return n,outputs,statuses


@pytest.mark.parametrize('kind,expected',[
    ('stop','clear'),('stale_command','stale_input'),('reverse','invalid_or_reverse_command'),
    ('nan','invalid_or_reverse_command'),('stale_scan','stale_input'),
    ('motion_gap','motion_missing'),('actual_obstacle','lidar_braking')])
def test_actual_control_preserves_stops_and_braking(kind,expected):
    n,outputs,statuses=node()
    if kind=='stop': n.command.drive.speed=0.
    elif kind=='stale_command': n.command.header.stamp=stamp(.3)
    elif kind=='reverse': n.command.drive.speed=-.1
    elif kind=='nan': n.command.drive.speed=float('nan')
    elif kind=='stale_scan': n.scan.header.stamp=stamp(.1)
    elif kind=='motion_gap':
        def fail(_): raise MotionUnavailable('motion_missing')
        n.motion.project=fail
    else: n.scan.ranges[250]=.05
    control,_=control_method(); control(n)
    report=json.loads(statuses[-1].data)
    assert outputs[-1].drive.speed==0.
    assert report['reason']==expected
    if kind=='stale_command': assert n.partial_policy.last_source==.5


def test_final_occupancy_sweep_keeps_only_current_steering():
    n,outputs,_=node(steering=.1,previous=.4,partial=False)
    control,namespace=control_method(); calls=[]
    def record(points,steering,*args,**kwargs):
        calls.append(steering)
        return swept_limit(points,steering,*args,**kwargs)
    namespace['swept_limit']=record
    control(n)
    assert calls==[.1] and outputs[-1].drive.speed>0.


def test_unregistered_source_frame_still_rejected():
    with pytest.raises(ValueError,match='registered lidar source frame'):
        MotionConfig(lidar_frame_id='unknown_frame')


def test_generated_profile_connects_only_explicit_shared_selection():
    import runtime_installed_limits as subject
    import yaml
    base={name:{'ros__parameters':{'max_steering_angle_rad':.45,
        'g1_traffic_recovery_trial':True}} for name in subject.NODES}
    planner=base['local_pursuit']['ros__parameters']
    planner['g1_guard_observation_policy']='selected_r181'
    original=copy.deepcopy(base)
    result=subject.derived_profile(yaml.safe_dump(base),.2)
    assert result['lidar_safety']['ros__parameters']['g1_guard_observation_policy']=='selected_r181'
    assert base==original
    for remove in ('selection','planner_traffic','guard_traffic'):
        off=copy.deepcopy(base)
        if remove=='selection': off['local_pursuit']['ros__parameters'].pop('g1_guard_observation_policy')
        else: off['local_pursuit' if remove=='planner_traffic' else 'lidar_safety']['ros__parameters']['g1_traffic_recovery_trial']=False
        result=subject.derived_profile(yaml.safe_dump(off),.2)
        assert 'g1_guard_observation_policy' not in result['lidar_safety']['ros__parameters']
