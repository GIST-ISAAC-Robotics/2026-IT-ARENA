"""원 기억 보존/명령별 저속 cap. 실제 ROS·차량 실행은 하지 않는다."""
import copy,json
import numpy as np
import pytest
from arena_autonomy.guard_memory_command import RetainedCommandMemory,limited_memory_command
from arena_autonomy.lidar_motion import MotionConfig,MotionHistory,MotionUnavailable
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from arena_autonomy.scan_input import project_scan
import test_independent_guard_route as fixture

PARTIAL=dict(allowed=False,reason='known_obstacle_latched',speed_cap_mps=None)

def seeded(points=((.06,.15),),reference=.4,source=.3,h=None):
    p=fixture.policy(RecoveryPolicy)
    p.latched=True;p.latch_reason='previous_point_in_missing_sweep';p.latch_time=.5
    p.latch_evidence={'scope':'test_actual_point_memory','points':copy.deepcopy(points)}
    p.guard_memory=RetainedCommandMemory(points,reference,source,('laser_frame',0),range(len(points)))
    return p,fixture.history() if h is None else h

def check(p,h,raw=None,now=.6,requested=.8,measured=0.,steering=0.,previous=0.,partial=None):
    raw=fixture.scan(now-.1) if raw is None else raw
    raw.ranges[375]=float('nan')
    points=project_scan(raw,h,now,timing_verified=True)[0]
    return limited_memory_command(p,PARTIAL if partial is None else partial,raw,points,h,now,
        requested,measured,steering,previous,.135,.20,.15)

def actual_latch(distance=.15):
    p=fixture.policy(RecoveryPolicy);h=fixture.history()
    old=fixture.scan(.3,[2.]*500);old.ranges[375]=distance
    assert fixture.evaluate(p,old,h,now=.4,speed=.4)['allowed']
    raw=fixture.scan(.5,[2.]*500);raw.ranges[375]=float('nan')
    output=fixture.evaluate(p,raw,h,now=.6,speed=.4)
    assert output['reason']=='known_obstacle_latched' and p.guard_memory is not None
    return p,h,raw

def test_real_latch_capture_preserves_unrelated_memory_original_evidence_and_unknown():
    p,h,raw=actual_latch();before=copy.deepcopy(p.latch_evidence)
    original=p.guard_memory.original.copy();ranges=np.array(raw.ranges)
    p.anchor_probe.ended=p.clearance_probe.ended=True
    result=check(p,h,raw)
    assert result['accepted'] and result['speed_cap_mps']==.8
    assert result['retained_points']==1 and not result['memory_relevant_to_command']
    assert p.latched and p.latch_evidence==before
    np.testing.assert_array_equal(p.guard_memory.original,original)
    np.testing.assert_array_equal(raw.ranges,ranges)
    assert result['full_2mps_reachable_set_guaranteed'] is False

def test_raw_and_retained_immediate_obstacles_each_stop():
    p,h=seeded(((.06,.08),));result=check(p,h)
    assert not result['accepted'] and result['current_memory_clearance_m']==0
    p,h=seeded();raw=fixture.scan();raw.ranges[250]=.05
    result=check(p,h,raw)
    assert not result['accepted'] and result['current_raw_clearance_m']==0

def test_previous_steering_only_conflict_stops():
    p,h=seeded(((.13,.13),));raw=fixture.scan()
    raw.ranges[320:350]=[float('nan')]*30
    result=check(p,h,raw,previous=.45)
    assert result['current_memory_clearance_m']>result['previous_memory_clearance_m']
    assert not result['accepted'] and result['previous_memory_clearance_m']<=.06

def test_higher_measured_speed_uses_real_braking_budget_then_low_target():
    p,h=seeded();result=check(p,h,requested=.1,measured=.5)
    assert result['accepted'] and result['speed_cap_mps']==.1
    assert result['checked_speed_mps']==.5 and result['braking_speed_mps']>=.5
    p,h=seeded(((.35,0.),));raw=fixture.scan()
    raw.ranges[245:255]=[float('nan')]*10
    result=check(p,h,raw,measured=.5)
    assert not result['accepted'] and result['braking_speed_mps']<.5


def test_unobserved_forward_memory_keeps_crawl_cap():
    p,h=seeded(((.35,0.),));raw=fixture.scan()
    raw.ranges[245:255]=[float('nan')]*10
    result=check(p,h,raw,measured=.1)
    assert result['accepted'] and result['speed_cap_mps']==.1
    assert result['memory_relevant_to_command'] and result['retained_points']==1

def test_continuous_transport_survives_probe_window_and_motion_buffer_rollover():
    h=MotionHistory(MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06,history_s=.5))
    p,_=seeded(((.3,.2),),h=h);original=p.guard_memory.original.copy();last=-1
    for tick in range(6,23):
        now=tick/10
        for sample in range(last+1,round(now*100)+1):
            h.add_mean_wheel(sample/100,.1/.0325);h.add_gyro(sample/100,0.)
        last=round(now*100)
        raw=fixture.scan(now-.1);raw.ranges[250:375]=[float('nan')]*125
        result=check(p,h,raw,now=now,measured=.1)
        assert result['accepted'] and len(p.guard_memory.points)==1
        np.testing.assert_allclose(p.guard_memory.points,original-[.1*(now-.4),0.],atol=1e-12)
    assert h.wheels.times[0]>.4 and p.guard_memory.original_reference==.4 and p.latched
    np.testing.assert_array_equal(p.guard_memory.original,original)

def test_additional_actual_lost_point_retained_once_can_stop_new_command():
    p,h=seeded();assert check(p,h)['accepted'];p.current_source_key=('laser_frame',0)
    old=np.array([[.06,.08],[1.,1.]])
    p.point_ids[(.5,.6)]=np.array([375,376]);threat=np.array([[True,False]])
    p._retain_latched_threat(threat,(old,.6,.5));p._retain_latched_threat(threat,(old,.6,.5))
    assert len(p.guard_memory.additional_original)==1
    result=check(p,h,now=.7)
    assert not result['accepted'] and len(p.guard_memory.points)==2
    np.testing.assert_array_equal(p.guard_memory.original,[[.06,.15]])

def test_observed_memory_handoff_restores_normal_cap_without_deleting_original():
    p,h=seeded(((.5,.2),));raw=fixture.scan(ranges=[2.]*500);points=project_scan(raw,h,.6,timing_verified=True)[0]
    # 실제 투영된 한 반환을 보존점으로 설정하여 대응을 정확히 검사한다.
    p.guard_memory=RetainedCommandMemory(points[[300]],.6,.5,('laser_frame',0),[300])
    original=p.guard_memory.original.copy()
    result=limited_memory_command(p,PARTIAL,raw,points,h,.6,.8,0.,0.,0.,.135,.20,.15)
    assert result['accepted'] and result['active_points_reobserved']==1
    assert result['speed_cap_mps']==.8 and result['normal_cap_restored']
    assert len(p.guard_memory.points)==0 and p.latched
    np.testing.assert_array_equal(p.guard_memory.original,original)

def test_real_side_reobservation_and_new_loss_preserve_evidence_and_relevance():
    p,h,raw=actual_latch();original=p.guard_memory.original.copy()
    first=check(p,h,raw)
    assert first['speed_cap_mps']==.8 and first['retained_points']>0
    assert not first['memory_relevant_to_command'] and not first['all_retained_points_currently_resolved']
    seen=fixture.scan(.6,[2.]*500);seen.ranges[375]=.15
    points=project_scan(seen,h,.7,timing_verified=True)[0]
    partial=p.evaluate(seen,points,h,.7,.8)
    observed=limited_memory_command(p,partial,seen,points,h,.7,.8,0.,0.,0.,.135,.20,.15)
    assert observed['accepted'] and observed['speed_cap_mps']==.8 and p.latched
    assert observed['retained_points']==0 and observed['all_retained_points_currently_resolved']
    missing=fixture.scan(.7,[2.]*500);missing.ranges[375]=float('nan')
    points=project_scan(missing,h,.8,timing_verified=True)[0]
    partial=p.evaluate(missing,points,h,.8,.8)
    lost=limited_memory_command(p,partial,missing,points,h,.8,.8,0.,0.,0.,.135,.20,.15)
    assert lost['accepted'] and lost['speed_cap_mps']==.8 and lost['retained_points']>0
    assert not lost['memory_relevant_to_command'] and not lost['all_retained_points_currently_resolved']
    assert p.latched and len(p.guard_memory.additional_original)>0
    np.testing.assert_array_equal(p.guard_memory.original,original)

@pytest.mark.parametrize('failure',['epoch','frame','reverse_time','motion_reset','motion_missing'])
def test_epoch_time_motion_failures_cannot_create_permission(failure):
    p,h=seeded();assert check(p,h)['accepted'];raw=fixture.scan(.6);now=.7
    if failure=='epoch':raw.source_epoch=1
    elif failure=='frame':raw.header.frame_id='changed_frame'
    elif failure=='reverse_time':raw=fixture.scan(.4);now=.5
    elif failure=='motion_reset':
        h.reset()
        for sample in range(80):h.add_mean_wheel(sample/100,0.);h.add_gyro(sample/100,0.)
    else:h.gyro.times=[];h.gyro.values=[]
    try:result=check(p,h,raw,now=now)
    except MotionUnavailable:return  # 동일 상위 입력 검증이 먼저 실패한 경우도 허가 없음.
    assert not result['accepted'] and p.latched

def test_only_selected_known_loss_and_positive_command_can_get_cap():
    p,h=seeded();assert not check(p,h,requested=0)['accepted']
    assert not check(p,h,partial={**PARTIAL,'reason':'traffic_recovery_invalid'})['accepted']
    p.command_observation_priority=False;assert not check(p,h)['accepted']

def test_existing_structure_memory_cap_is_preserved_even_when_base_latched():
    p,h=seeded()
    result=check(p,h,partial={**PARTIAL,'known_structure':{'speed_cap_mps':0.}})
    assert not result['accepted'] and result['reason']=='existing_guard_cap_zero'
    result=check(p,h,partial={**PARTIAL,'known_structure':{'speed_cap_mps':.04}})
    assert result['accepted'] and result['speed_cap_mps']==.04

def guard_node(raw,p,h,speed=.8):
    n,outputs,statuses=fixture.node(raw=raw,speed=speed,previous=0.)
    n.partial_policy=p;n.motion.history=h;n.motion.drain_pending=lambda:None
    def project(raw):
        points,ids,_=project_scan(raw,h,.6,timing_verified=True)
        return points,len(ids)
    n.motion.project=project
    return n,outputs,statuses

def test_actual_control_ignores_unrelated_memory_and_stops_for_raw_obstacle():
    for obstacle in (False,True):
        p,h,raw=actual_latch()
        if obstacle:raw.ranges[250]=.05
        n,out,status=guard_node(raw,p,h)
        control,_=fixture.control_method();control(n)
        assert out[-1].drive.speed==(0. if obstacle else .8) and p.latched
        report=json.loads(status[-1].data)
        assert report['partial_observation']['retained_memory_command']['accepted'] is (not obstacle)

@pytest.mark.parametrize('failure',['stop','stale_command','missing_command','stale_scan'])
def test_actual_control_preserves_command_and_input_stops(failure):
    p,h,raw=actual_latch();n,out,status=guard_node(raw,p,h)
    if failure=='stop':n.command.drive.speed=0.
    elif failure=='stale_command':n.command.header.stamp=fixture.stamp(.3)
    elif failure=='missing_command':n.command=None
    else:n.scan.header.stamp=fixture.stamp(.1)
    control,_=fixture.control_method();control(n)
    assert out[-1].drive.speed==0 and p.latched
    if failure=='stale_command':assert p.guard_memory.reference==.4

def test_normal_positive_control_without_latch_is_unchanged():
    n,out,_=fixture.node(previous=0.);n.motion.drain_pending=lambda:None
    control,_=fixture.control_method();control(n)
    assert out[-1].drive.speed==.8 and not n.partial_policy.latched
