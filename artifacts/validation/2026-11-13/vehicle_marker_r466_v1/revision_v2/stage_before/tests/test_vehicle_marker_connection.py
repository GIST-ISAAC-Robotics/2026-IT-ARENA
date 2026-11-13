"""실제 plan/current 함수 및 guard.control 본문 연결; ROS I/O만 double, 장치 사용 없음."""
import ast
import copy
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_vehicle_marker_regions import profile,packet,history,STAGE
from arena_autonomy.vehicle_marker_channel import checked_packet,MarkerChannel
from arena_autonomy.vehicle_marker_occupancy import marker_scope,additional_clearance,_call_cloud
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.lidar_motion import MotionUnavailable
from test_local_follow import scene,settle,scan,history as full_history


def wire_packet(profile,stamp=1.,depth=.25):
    p=packet(profile,stamp=stamp,depth=depth)
    p.update(clock_domain='ros_sim',timing_verified=False,fresh=True)
    return p


def warm(controller,h):
    settle(controller,h)
    for now in (.8,.9,1.):controller.command(scene(now),h,0.,'left')


@pytest.mark.parametrize('change',[
    dict(clock_domain='ros_system'),dict(frame_id='world'),dict(vision_epoch=-1),
    dict(source_stamp_s=2.),dict(source_stamp_s=.5),dict(timing_verified=True),
    dict(purpose='calibration_only'),dict(profile_id='other')])
def test_wire_mismatch_is_optional_unavailable(profile,change):
    assert checked_packet(dict(wire_packet(profile),**change),profile,1.1,'ros_sim') is None


def test_shape_code_id_and_ttl_are_not_trusted_from_json(profile):
    p=wire_packet(profile)
    assert checked_packet(p,profile,1.1,'ros_sim')
    for field,value in [('id',20),('black_code_size_m',.05),('width_m',.01),
                        ('polygon_xy_m',[[0.,0.]]*4)]:
        changed=copy.deepcopy(p);changed['regions'][0][field]=value
        assert checked_packet(changed,profile,1.1,'ros_sim') is None
    p['max_age_s']=999
    assert checked_packet(p,profile,1.3,'ros_sim') is None


def test_context_nested_exception_reset_and_no_scene_mutation(profile):
    h=history();cfg=FollowConfig();p=wire_packet(profile)
    data=dict(vehicle_marker_regions=p,reference_s=1.1)
    assert additional_clearance(2.,0.,0.,cfg)==2.
    with marker_scope(data,h,cfg,profile,'ros_sim') as outer:
        assert additional_clearance(2.,0.,0.,cfg)<.5
        with pytest.raises(RuntimeError):
            with marker_scope(dict(reference_s=1.1),h,cfg,profile,'ros_sim'):
                assert additional_clearance(2.,0.,0.,cfg)==2.
                raise RuntimeError('inner failure')
        assert _call_cloud.get() is outer
    assert _call_cloud.get() is None and additional_clearance(2.,0.,0.,cfg)==2.


def test_actual_local_follow_point_path_and_wall_preserved(profile):
    h=full_history();baseline=LocalFollow();warm(baseline,h)
    other=copy.deepcopy(baseline);s=scene(1.1);before=copy.deepcopy(s)
    out=baseline.command(s,h,0.,'left')
    p=wire_packet(profile,depth=.55)
    with marker_scope(dict(reference_s=1.1,vehicle_marker_regions=p),h,other.config,profile,'ros_sim') as scope:
        marked=other.command(s,h,0.,'left')
    assert out[0]>marked[0] and scope['limited_checks']>0
    assert marked[1]==out[1] and marked[2]['path_points']==out[2]['path_points']
    assert marked[2]['path_wall']==out[2]['path_wall']
    assert marked[2]['occupied_points']==out[2]['occupied_points']==len(s.occupied_endpoints_xy_m)
    assert s==before
    # Empty optional packet uses exactly the original input and output.
    base=LocalFollow();warm(base,h);other=copy.deepcopy(base)
    p['regions']=[]
    with marker_scope(dict(reference_s=1.1,vehicle_marker_regions=p),h,other.config,profile,'ros_sim'):
        assert other.command(s,h,0.,'left')==base.command(s,h,0.,'left')


def test_actual_m3_candidate_sweep_receives_only_separate_cloud(profile,monkeypatch):
    import arena_autonomy.m3_execution_trial as mx
    from test_m3_execution_trial import ctl
    controller=ctl();h=full_history();s=scene(1.1)
    path=np.column_stack((np.linspace(-.2,2.,80),np.zeros(80)))
    baseline=(.7,0.,dict(path_points=path.tolist(),boundary_age_s=.1))
    monkeypatch.setattr(mx,'arc_checks',lambda *a,**k:dict(observation_ok=True,geometry_ok=True))
    args=('pass',path,s,None,h,0.,0.,.1,.055,baseline)
    out=controller._evaluate_command(*args)
    with marker_scope(dict(reference_s=1.1,vehicle_marker_regions=wire_packet(profile)),h,
                      controller.config,profile,'ros_sim') as scope:
        marked=controller._evaluate_command(*args)
    assert marked['clearance']<out['clearance'] and marked['target']<out['target']
    assert scope['checks']==1 and marked['detail']['occupied_points']==out['detail']['occupied_points']


def test_snapshot_and_actual_evaluator_plan_current_connections(profile,monkeypatch):
    from arena_autonomy.follow_trace import snapshot_call,restore_call
    from arena_autonomy.race_runtime_policy.evaluator import Evaluator
    import arena_autonomy.scene_motion as sm
    h=full_history();raw=scan(first=.95);p=wire_packet(profile)
    data=snapshot_call(raw,h,1.1,0,0.,'left',0.,.1,True,0.,vehicle_marker_regions=p)
    off=snapshot_call(raw,h,1.1,0,0.,'left',0.,.1,True,0.)
    assert set(data)-set(off)=={'vehicle_marker_regions'}
    assert all(data[key]==value for key,value in off.items())
    monkeypatch.setattr(sm,'motion_scene',lambda *a,**k:scene(1.1))
    def evaluator(stage):
        obj=Evaluator.__new__(Evaluator);obj.stage=stage;obj.time_policy='verified_only';obj.corner_budget=False
        obj.vehicle_marker_profile=profile;obj.vehicle_marker_clock_domain='ros_sim'
        obj.motion=h.config;obj.follow=LocalFollow();warm(obj.follow,h)
        return obj
    plan=evaluator('plan');baseline=copy.deepcopy(plan.follow)
    expected=baseline.command(scene(1.1),h,0.,'left',0.,.1)
    result=plan(data)
    assert result[0]<expected[0] and result[2]['vehicle_marker_sweep']['limited_checks']>0
    current=evaluator('current')
    original=dict(speed_mps=.75,steering_rad=0.,reference_s=1.1,reason='rebase_ok',runtime_authorized=False)
    current.current_policy=NS(rebase=lambda *a,**k:original)
    got=current(dict(current=data,output=expected,submitted=off))
    assert got['speed_mps']<original['speed_mps'] and got['vehicle_marker_occupancy']['applied']
    current.vehicle_marker_profile=None
    assert current(dict(current=off,output=expected,submitted=off))==original


class RosString:
    def __init__(self,*,data):self.data=data


class FakeNode:
    def __init__(self,path):self.path=path;self.now_value=1.1;self.wall_timeout=3.;self.sent=[];self.subscriptions=[]
    def declare_parameter(self,*a):return NS(value=self.path)
    def get_parameter(self,*a):return NS(value=True)
    def get_clock(self):return NS(now=lambda:NS(nanoseconds=round(self.now_value*1e9),to_msg=lambda:None))
    def create_publisher(self,*a):return NS(publish=self.sent.append)
    def create_subscription(self,*a):self.subscriptions.append(a)


def test_ros_packet_producer_receiver_empty_expiry_and_clock_reset(profile,tmp_path,monkeypatch):
    monkeypatch.setitem(sys.modules,'std_msgs.msg',NS(String=RosString))
    path=tmp_path/'profile.json';path.write_text(json.dumps(profile),encoding='utf-8')
    producer=FakeNode(str(path));consumer=FakeNode(str(path))
    pub=MarkerChannel(producer,producer=True);sub=MarkerChannel(consumer,producer=False)
    from arena_autonomy.vehicle_markers import VehicleRegionBuffer
    buffer=VehicleRegionBuffer(profile);buffer.update(packet(profile),stamp=1.,epoch=0)
    pipeline=NS(vehicle_regions=buffer,epoch=0)
    pub.publish(pipeline);sub.receive(producer.sent[-1])
    assert sub.snapshot(1.1)['regions']
    empty=packet(profile,stamp=1.1);empty['regions']=[];buffer.update(empty,stamp=1.1,epoch=0)
    pub.publish(pipeline);sub.receive(producer.sent[-1])
    assert not sub.snapshot(1.11)['regions']
    consumer.now_value=1.16
    sub.reset(1.15);sub.receive(producer.sent[-1]);assert sub.latest is None
    consumer.now_value=1.36;assert sub.snapshot(1.36) is None
    consumer.now_value=.5;sub.receive(producer.sent[-1]);assert sub.latest is None
    off=FakeNode('');channel=MarkerChannel(off,producer=False)
    assert channel.snapshot(1.) is None and not off.subscriptions and not off.sent


def test_actual_independent_guard_control_body_keeps_drive_safe_boundary(profile,monkeypatch):
    # ROS packages unavailable on Windows: execute the unchanged staged method AST with I/O doubles.
    source=(STAGE/'src/arena_autonomy/arena_autonomy/lidar_safety.py').read_text(encoding='utf-8')
    cls=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name=='LidarSafety')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='control')
    from arena_autonomy.local_path import swept_limit,braking_speed
    class Drive:
        def __init__(self):self.header=NS(stamp=None,frame_id='');self.drive=NS(speed=0.,steering_angle=0.)
    namespace=dict(math=math,np=np,json=json,time=time,AckermannDriveStamped=Drive,String=RosString,
        MotionUnavailable=MotionUnavailable,seconds=lambda x:x.sec+x.nanosec*1e-9,
        swept_limit=swept_limit,braking_speed=braking_speed)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),'lidar_safety.control','exec'),namespace)
    import arena_autonomy.scan_input as si
    monkeypatch.setattr(si,'audit_scan_front',lambda *a:NS(reason='ok'))
    h=history();points=np.column_stack((np.linspace(.1,2.,50),np.full(50,1.)))
    def node(packet):
        n=FakeNode('');n.last_clock=None;n.previous_safe_steering=0.;n.limit=.37;n.length=.20;n.width=.15;n.wheelbase=.145
        n.partial_policy=None;n.recent_observation_enabled=False;n.scan=scan(first=1.)
        n.scan_wall=n.command_wall=time.monotonic();n.command=Drive();n.command.header.stamp=NS(sec=1,nanosec=0)
        n.command.drive.speed=.75;n.motion=NS(history=h,project=lambda scan:(points,len(points)))
        n.vehicle_marker_channel=NS(profile=profile,snapshot=lambda now:packet,reset=lambda:None)
        n.vehicle_marker_config=FollowConfig();n.safe=[];n.status_rows=[]
        n.publisher=NS(publish=n.safe.append);n.status=NS(publish=n.status_rows.append)
        return n
    base=node(None);namespace['control'](base)
    marked=node(wire_packet(profile));namespace['control'](marked)
    assert 0<=marked.safe[0].drive.speed<base.safe[0].drive.speed
    assert json.loads(marked.status_rows[0].data)['reason']=='vehicle_marker_braking'
    assert len(marked.safe)==1 and marked.safe[0].header.frame_id=='base_link'
