"""전환 후보의 실제 상태·센서 시각·기록 경계와 이전 실행 경로를 확인한다."""
from dataclasses import asdict, replace
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from arena_autonomy.bypass_handoff import (BlockingPersistence, BypassHandoffFollow,
    HandoffExecutionConfig, handoff_baseline, front_patch)
from arena_autonomy.follow_trace import FollowTrace, replay_events, freeze, controller_mode, comparable_decision
from arena_autonomy.g1_g2_node import G1G2Node, status_detail
from arena_autonomy.local_pursuit import LocalPursuit
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.scene_motion import motion_scene
from probe_bypass_entry_sequence import render_scan


def context(now=.5):
    physical=[dict(t=t,rear_pose=[-.0675,0.,0.],speed=0.,steering=0.) for t in (0.,2.)]
    scan=render_scan(physical,now,-.15)
    history=MotionHistory(MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06))
    for t in np.arange(0.,2.01,.01):
        history.add_mean_wheel(float(t),0.);history.add_gyro(float(t),0.)
    return scan,history,motion_scene(scan,history,now,timing_verified=True,rear_half_angle_deg=30.)


def subject():return BypassHandoffFollow(allow_simulation_trial=True)


def sparse_scene(points):
    return SimpleNamespace(rays=[SimpleNamespace(hit_xy_m=np.asarray(p),source_index=200+3*i)
                                  for i,p in enumerate(points)])


def test_sparse_preparation_uses_actual_returns_without_connecting_gaps():
    points=[(1.11,-.20),(1.27,-.20),(1.11,-.10),(1.27,-.10)]
    sc=sparse_scene(points)
    assert front_patch(sc)[0] is None
    selected,reason=front_patch(sc,'sparse_returns_v1')
    assert reason=='sparse_front_returns' and selected.shape==(4,2)
    np.testing.assert_equal(selected,np.asarray(points))
    # 두 가까운 물체인지 같은 물체인지 구분하지 않는다. 실제4점이 그대로 남는다.
    assert all(any(np.array_equal(p,q) for q in points) for p in selected)


@pytest.mark.parametrize('points',[
    [(1.,-.1),(1.,-.12)],  # 현재 실제 반환 부족
    [(1.,-.1),(1.4,-.1),(1.5,-.1)],  # 멀리 있는 점으로 가까운 조각을 보충하지 않음
    [(1.,-.20),(1.,.20),(1.,0.)],  # 넓은 측방 분포
    [(1.,-.24),(1.,-.22),(1.,-.20)],  # 중심 진행 영역에 들어오지 않은 조각
    [(1.8,-.10),(1.8,-.11),(1.8,-.12)],
])
def test_sparse_preparation_rejects_insufficient_or_wide_support(points):
    assert front_patch(sparse_scene(points),'sparse_returns_v1')[0] is None


def test_sparse_option_is_explicit_and_legacy_trace_keeps_original():
    old=asdict(HandoffExecutionConfig());old.pop('front_patch_policy')
    assert HandoffExecutionConfig.from_mapping(old).front_patch_policy=='contiguous_three_v0'
    cfg=HandoffExecutionConfig(front_patch_policy='sparse_returns_v1')
    assert HandoffExecutionConfig.from_mapping(asdict(cfg))==cfg
    with pytest.raises(ValueError):HandoffExecutionConfig(front_patch_policy='fill_gaps')
    with pytest.raises(ValueError):front_patch(sparse_scene([]),'fill_gaps')


def test_sparse_reserve_uses_nearest_return_not_rear_posts():
    h=BypassHandoffFollow(execution=HandoffExecutionConfig(preparation_policy='observed_entry_space_v1',
        persistence_window_policy='trim_before_compare_v1',follow_entry_space_enabled=True,
        front_patch_policy='sparse_returns_v1'),allow_simulation_trial=True)
    h.monitor.rows=[{},{}]
    sc=sparse_scene([(1.11,-.20),(1.27,-.20),(1.11,-.10),(1.27,-.10)])
    sc.t_ref_s=.5;sc.source_first_s=.4
    with patch('arena_autonomy.bypass_handoff.straight_reference',return_value=({},{})):
        target,detail=h._follow_entry_space(sc,(1.,0.,{}),0.,1.)
    assert target==0. and detail['rear_axle_to_patch_m']==pytest.approx(1.1775)


@pytest.mark.parametrize('motion,ready',[(0.,True),(.25,False)])
def test_sparse_observation_still_requires_fresh_low_motion_window(motion,ready):
    monitor=BlockingPersistence(HandoffExecutionConfig(front_patch_policy='sparse_returns_v1',
        persistence_window_policy='trim_before_compare_v1'))
    history=MotionHistory(MotionConfig(wheel_radius_m=.0325,history_s=3.))
    for t in np.arange(0.,2.,.01):
        history.add_mean_wheel(float(t),0.);history.add_gyro(float(t),0.)
    for k in range(11):
        t=round(.5+k*.1,9);dx=motion*(t-.5)
        sc=sparse_scene([(1.11+dx,-.20),(1.27+dx,-.20),(1.11+dx,-.10),(1.27+dx,-.10)])
        sc.t_ref_s=t;sc.source_first_s=t-.1;sc.source_epoch=0;sc.source_frame_id='laser_frame'
        with patch('arena_autonomy.bypass_handoff.source_valid',return_value=True):
            r=monitor.update(sc,history)
            for _ in range(4):
                duplicate=monitor.update(sc,history)
                assert duplicate['fresh'] is False and duplicate['ready']==r['ready']
        if k<8:assert r['ready'] is False
    assert r['ready'] is ready
    assert r['identity_or_stationarity_proven'] is False


def relevance():
    # 새 관측 근거 조건은 명시 선택한다. 기본은 기존 즉시 준비 감속이다.
    return BypassHandoffFollow(execution=HandoffExecutionConfig(preparation_policy='observed_entry_relevance_v1'),
                               allow_simulation_trial=True)


def test_explicit_trial_and_options():
    with pytest.raises(ValueError):BypassHandoffFollow()
    with pytest.raises(ValueError):BypassHandoffFollow(replace(handoff_baseline(),pass_release_trial=False),allow_simulation_trial=True)
    with pytest.raises(ValueError):HandoffExecutionConfig(minimum_distinct_scans=True)
    with pytest.raises(ValueError):HandoffExecutionConfig(maximum_patch_displacement_m=.07)
    assert HandoffExecutionConfig.from_mapping(asdict(HandoffExecutionConfig()))==HandoffExecutionConfig()
    assert controller_mode(subject())=='g1_g2_handoff_trial'


@pytest.mark.parametrize('enabled',[False,True])
def test_explicit_g1_prefix_configuration_is_preserved(enabled):
    config=replace(handoff_baseline(),short_boundary_prefix_enabled=enabled)
    ctl=BypassHandoffFollow(config,allow_simulation_trial=True)
    assert ctl.normal.config.short_boundary_prefix_enabled is enabled
    assert ctl.config is config
    assert ctl.bypass.guide.stationary_outer_boundary is True
    assert ctl.bypass.guide.start_confirmations == 0
    assert handoff_baseline().short_boundary_prefix_enabled is False


@pytest.mark.parametrize('window',['compare_then_trim_v0','trim_before_compare_v1'])
@pytest.mark.parametrize('ego,target,ready',[(.35,0.,True),(.35,.25,False),(0.,.03,True)])
def test_patch_motion_and_slow_target_limit(ego,target,ready,window):
    monitor=BlockingPersistence(HandoffExecutionConfig(persistence_window_policy=window))
    history=MotionHistory(MotionConfig(wheel_radius_m=.0325,history_s=3.))
    for t in np.arange(0.,2.,.01):
        history.add_mean_wheel(float(t),ego/.0325);history.add_gyro(float(t),0.)
    for k in range(11):
        t=round(.5+.1*k,9)
        cloud=np.c_[np.ones(7)*(1.+(target-ego)*(t-.5)),np.linspace(-.18,-.1,7)]
        sc=SimpleNamespace(t_ref_s=t,source_first_s=t-.1,source_epoch=0,source_frame_id='laser_frame')
        with patch('arena_autonomy.bypass_handoff.source_valid',return_value=True),patch(
                'arena_autonomy.bypass_handoff.front_patch',return_value=(cloud,'face')):
            result=monitor.update(sc,history)
    assert result['ready']==ready
    assert result['identity_or_stationarity_proven'] is False
    with patch('arena_autonomy.bypass_handoff.source_valid',return_value=True):
        assert monitor.update(sc,history)['fresh'] is False


def stubbed(offset=.16,seen=False,phase='entry'):
    h=subject();captured=[]
    h.normal.command=lambda *a,**kw:(.3,0.,dict(selector_choice='baseline',selector_sides={
        'left':dict(seen=seen,present=False),'right':dict(seen=False,present=False)}))
    h.monitor.update=lambda *a:dict(ready=True,observed=True,fresh=True)
    def candidate(sc,history,speed,*a,**kw):
        captured.append(speed)
        return .35,.03,dict(selected=dict(offset_m=offset),entry_phase=phase)
    h.bypass.command=candidate
    return h,captured


@pytest.mark.parametrize('offset,seen,mode',[(.16,False,'BYPASS'),(0.,False,'FOLLOW'),(.16,True,'FOLLOW')])
def test_entry_gate_and_quantized_speed(offset,seen,mode):
    h,seen_speed=stubbed(offset,seen);scan,history,scene=context()
    output=h.command(scene,history,.364649,'left',0.,.02,raw_scan=scan)
    assert seen_speed==[.364649]
    assert h.mode==mode and output[0]<=.35


def test_reacquire_requires_distinct_sources_and_keeps_side_history():
    h,_=stubbed(phase='centered');h.mode='BYPASS'
    history_token=h.normal.pass_selector.sides
    for stamp in (.5,.5):
        scan,history,scene=context(stamp)
        h.command(scene,history,.3,'left',0.,.02,raw_scan=scan)
        assert h.mode=='REACQUIRE'
    scan,history,scene=context(.6)
    result=h.command(scene,history,.3,'left',0.,.02,raw_scan=scan)
    assert result[2]['reason']=='handoff_return' and h.mode=='FOLLOW'
    assert h.normal.pass_selector.sides is history_token


@pytest.mark.parametrize('fault',['epoch','clock','frame','stale','negative','nan','no_scan','bad_dt'])
def test_faults_drop_active_bypass(fault):
    h,_=stubbed();scan,history,scene=context(.6)
    h.command(scene,history,.3,'left',0.,.02,raw_scan=scan)
    assert h.mode=='BYPASS'
    speed,dt=.3,.02
    if fault=='epoch':
        scene=motion_scene(scan,history,.6,timing_verified=True,epoch=1,rear_half_angle_deg=30.)
    elif fault=='clock':
        scan,history,scene=context(.5)
    elif fault=='frame':
        scene=replace(scene,source_frame_id='bad',rays=tuple(replace(r,source_frame_id='bad') for r in scene.rays))
    elif fault=='stale':scene=replace(scene,t_ref_s=1.)
    elif fault=='negative':speed=-.01
    elif fault=='nan':speed=float('nan')
    elif fault=='no_scan':scan=None
    elif fault=='bad_dt':dt=0.
    result=h.command(scene,history,speed,'left',0.,dt,raw_scan=scan)
    assert result[0]==0 and h.mode=='FOLLOW'


def test_follow_above_command_cap_preserves_measurement_but_cannot_enter_g2():
    h,captured=stubbed();scan,history,scene=context()
    observed=[]
    def normal(sc,hist,speed,*a,**kw):
        observed.append(speed)
        return .3,0.,dict(selector_choice='baseline',selector_sides={})
    h.normal.command=normal
    result=h.command(scene,history,1.021,'left',0.,.02,raw_scan=scan)
    assert observed==[1.021] and captured==[]
    assert result[0]==.3 and h.mode=='FOLLOW'


def test_reverse_encoder_hold_preserves_g1_history_and_drops_bypass():
    h,captured=stubbed();scan,history,scene=context()
    h.command(scene,history,.3,'left',.01,.02,raw_scan=scan)
    assert h.mode=='BYPASS'
    old_sides=h.normal.pass_selector.sides
    speeds=[]
    original=h.normal.command
    def normal(sc,hist,speed,*a,**kw):
        speeds.append(speed)
        return original(sc,hist,speed,*a,**kw)
    h.normal.command=normal
    with patch.object(h.normal,'reset',wraps=h.normal.reset) as reset:
        result=h.command(scene,history,-.048619886,'left',.01,.02,raw_scan=scan)
    assert result[:2]==(0.,.01)
    assert result[2]['reason']=='handoff_reverse_motion_hold'
    assert speeds==[-.048619886] and not reset.called
    assert h.normal.pass_selector.sides is old_sides
    assert h.mode=='FOLLOW' and h.bypass.guide.path is None and not h.monitor.rows


def test_ros_command_recording_matches_schema5_and_resets(tmp_path):
    node=G1G2Node.__new__(G1G2Node)
    node.follow=subject();scan,history,scene=context()
    node.motion=SimpleNamespace(history=history,last_meta={'reference_stamp_s':.5},verified=True,counts={'resets':0})
    node.side='left';node.last_steering=0.;node.settings={'control_rate_hz':50.};node.rear_blind_half_angle=30.
    path=tmp_path/'handoff.jsonl'
    node.follow_trace=FollowTrace(path,history.config,node.follow.config,
        mode='g1_g2_handoff_trial',execution_config=node.follow.execution)
    for now in np.round(np.arange(.5,1.61,.1),9):
        node.scan=context(float(now))[0];node.motion.last_meta['reference_stamp_s']=float(now)
        output=node.command(None)
        json.dumps(output[2],allow_nan=False)
        node.last_steering=output[1]
    node.reset_follow('SENSOR_STOP')
    node.scan.header.frame_id='bad'
    assert node.command(None)[0]==0
    node.follow_trace.close()
    events=[json.loads(line) for line in path.read_text().splitlines()]
    assert events[0]['data']['schema']==5
    outputs=[e['data']['output'][2] for e in events if e['kind']=='call']
    assert any(d.get('handoff_state')=='BYPASS' for d in outputs)
    assert any(d.get('bypass',{}).get('candidates') for d in outputs)
    result=replay_events(events)
    assert result['passed'] and result['matched']==13


def test_compact_status_and_nondeterministic_cost_exclusion_are_bounded():
    full=dict(mode='g1_g2_handoff_trial',normal=dict(partial_observation=dict(
        surface_correspondence=dict(processing_ms=2.,residual_m=.1))),bypass=dict(
        selected=dict(speed_mps=np.float64(.35)),candidates=[dict(eligible=np.bool_(True))]),reason='test')
    compact=status_detail(full);json.dumps(compact,allow_nan=False)
    assert compact['bypass']['eligible_candidates']==1
    comparable=comparable_decision([.35,0.,freeze(full)])
    fit=comparable[2]['normal']['partial_observation']['surface_correspondence']
    assert fit==dict(processing_ms=0.,residual_m=.1)
    assert full['normal']['partial_observation']['surface_correspondence']['processing_ms']==2.


def test_default_launch_and_drive_boundary_unchanged():
    root=Path(__file__).resolve().parents[1]
    assert all('g1_g2_handoff_trial' not in p.read_text() for p in (root/'src/arena_bringup/launch').glob('*.py'))
    assert G1G2Node.control is LocalPursuit.control
    assert G1G2Node.reset_follow is LocalPursuit.reset_follow
    assert G1G2Node.on_scan is LocalPursuit.on_scan


def test_actual_front_patch_is_only_observed_returns():
    _,_,scene=context();cloud,reason=front_patch(scene)
    assert reason=='current_front_patch' and len(cloud)>=3
    assert all(any(np.array_equal(p,r.hit_xy_m) for r in scene.rays if r.hit_xy_m is not None) for p in cloud)


def test_preparation_policy_options_and_old_schema5_mapping():
    assert HandoffExecutionConfig().preparation_policy=='front_patch_v0'
    assert subject().execution.preparation_policy=='front_patch_v0'
    with pytest.raises(ValueError):HandoffExecutionConfig(preparation_policy='front_patch')
    old=asdict(HandoffExecutionConfig());old.pop('preparation_policy')
    # 선택지가 생기기 전의 schema5 기록은 당시의 즉시 준비 감속으로 재생한다.
    assert HandoffExecutionConfig.from_mapping(old).preparation_policy=='front_patch_v0'


def fast_normal(h,speed=.8):
    h.normal.command=lambda *a,**kw:(speed,0.,dict(selector_choice='baseline',selector_sides={
        'left':dict(seen=False,present=False),'right':dict(seen=False,present=False)}))


@pytest.mark.parametrize('explicit',[False,True])
def test_legacy_policy_caps_first_patch_without_new_detail(explicit):
    h=(BypassHandoffFollow(execution=HandoffExecutionConfig(preparation_policy='front_patch_v0'),
                           allow_simulation_trial=True) if explicit else subject())
    fast_normal(h);scan,history,scene=context()
    output=h.command(scene,history,.3,'left',0.,.02,raw_scan=scan)
    assert output[0]==.35 and 'preparation' not in output[2]


def test_stationary_edge_patch_prepares_after_one_displacement_check():
    h=relevance();fast_normal(h)
    scan,history,scene=context(.5)
    first=h.command(scene,history,.3,'left',0.,.02,raw_scan=scan)
    assert first[0]==.8 and first[2]['preparation']['reason']=='single_scan_patch'
    scan,history,scene=context(.6)
    second=h.command(scene,history,.3,'left',0.,.02,raw_scan=scan)
    prepared=second[2]['preparation']
    assert second[0]==.35 and prepared['applied'] and prepared['reason']=='observed_entry_preparation'
    assert prepared['patch_arc_clearance_m']<2.5
    assert prepared['identity_or_stationarity_proven'] is False and prepared['free_space_assumed'] is False


def moving_monitor(speeds,ego=.35):
    """ego 운동으로 이송한 지면 기준 조각 속도 순서. 스캔당 0.1초."""
    monitor=BlockingPersistence();history=MotionHistory(MotionConfig(wheel_radius_m=.0325,history_s=3.))
    for t in np.arange(0.,3.,.01):
        history.add_mean_wheel(float(t),ego/.0325);history.add_gyro(float(t),0.)
    position,states=1.,[]
    for k,speed in enumerate(speeds):
        t=round(.5+.1*k,9)
        if k:position+=(speed-ego)*.1
        cloud=np.c_[np.ones(7)*position,np.linspace(-.18,-.1,7)]
        sc=SimpleNamespace(t_ref_s=t,source_first_s=t-.1,source_epoch=0,source_frame_id='laser_frame')
        with patch('arena_autonomy.bypass_handoff.source_valid',return_value=True),patch(
                'arena_autonomy.bypass_handoff.front_patch',return_value=(cloud,'face')):
            result=monitor.update(sc,history)
        states.append((monitor.motion_evidence(),len(monitor.rows),result['ready']))
    return monitor,states


def test_moving_patch_motion_evidence_and_stop_expiry():
    _,moving=moving_monitor([.25]*12)
    # 첫 초기화 전 두세 스캔만 근거 없음, 이후 0.25 m/s 이동은 계속 이동 증거다.
    assert not any(state[0] for state in moving[:3]) and all(state[0] for state in moving[4:])
    assert not any(state[2] for state in moving)
    monitor,stopping=moving_monitor([.25]*6+[0.]*10)
    # 60 mm/0.25 m/s=0.24초를 넘게 같은 범위에 머물면 감속/정지 가능성이 있으므로
    # 이동 증거로 준비를 막지 않는다. 이후 0.8초 누적의 진입 준비 판정은 그대로다.
    assert stopping[5][0] and not stopping[6][0] and stopping[-1][2]
    assert abs(monitor.motion_rate_mps-.25)<.02


def test_large_patch_jump_is_new_patch_not_motion_evidence():
    _,states=moving_monitor([0.,0.,0.,3.5,0.,0.])
    # 0.35 m 도약은 60 mm+0.5 m/s×0.1 s 연관 범위를 넘는다. 다른 반환의 시작이다.
    assert states[3][1]==1 and not any(state[0] for state in states)


def test_preparation_requires_g1_arc_and_two_wall_reference():
    from probe_bypass_entry_sequence import evaluation_boxes
    from g0_geometry import TruthBox, Footprint
    h=relevance();fast_normal(h)
    scan,history,scene=context(.6)
    h.monitor.rows=[dict(source=.4,time=.5,patch=np.c_[np.ones(5),np.linspace(.3,.4,5)]),
                    dict(source=.5,time=.6,patch=np.c_[np.ones(5),np.linspace(.3,.4,5)])]
    off=h._preparation(scene,history,(.8,0.,{}),dict(observed=True),0.)
    assert off['reason']=='patch_outside_g1_swept_arc' and not off['applied']
    def corner(at,target_y=0.,restart=False):
        # 왼쪽 벽이 끊기고 전방을 막는 바깥 벽. 곡선 진입부의 벽 반환 대표.
        return [TruthBox(-.3,-.45,.4,Footprint(3.3,.05),label='wall'),
                TruthBox(-.75,.45,.4,Footprint(2.5,.05),label='wall'),
                TruthBox(1.35,0.,.4,Footprint(.05,2.),label='wall')]
    with patch('probe_bypass_entry_sequence.evaluation_boxes',corner):
        scan,history,scene=context(.6)
    face,_=front_patch(scene)
    h.monitor.rows=[dict(source=.4,time=.6,patch=face),dict(source=.5,time=.6,patch=face)]
    curve=h._preparation(scene,history,(.8,0.,{}),dict(observed=True),0.)
    assert curve['reason']=='two_wall_straight_reference_unavailable' and not curve['applied']
    assert curve['patch_arc_clearance_m']<2.5
    within=h._preparation(scene,history,(.3,0.,{}),dict(observed=True),0.)
    assert within['reason']=='g1_command_within_cap'


def test_old_schema5_trace_without_policy_replays_legacy(tmp_path):
    ctl=BypassHandoffFollow(execution=HandoffExecutionConfig(preparation_policy='front_patch_v0'),
                            allow_simulation_trial=True)
    path=tmp_path/'old.jsonl'
    trace=FollowTrace(path,MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06),ctl.config,
        mode='g1_g2_handoff_trial',execution_config=ctl.execution)
    for now in (.5,.6,.7):
        scan,history,scene=context(now)
        output=ctl.command(scene,history,0.,'left',0.,.02,raw_scan=scan)
        trace.record_call(scan,history,now,0,0.,'left',0.,.02,True,30.,output)
    trace.close()
    events=[json.loads(line) for line in path.read_text().splitlines()]
    events[0]['data']['execution_config'].pop('preparation_policy')
    assert replay_events(events)['passed']


def trace_node(tmp_path,**parameters):
    """create_follow_trace만 확인하는 최소 노드. ROS 실행/Gazebo 검증 아님."""
    node=G1G2Node.__new__(G1G2Node)
    values=dict(g1_g2_simulation_trial=True,use_sim_time=True,**parameters)
    node.declare_parameter=lambda name,default=None:SimpleNamespace(value=values.get(name,default))
    node.get_parameter=lambda name:SimpleNamespace(value=values[name])
    node.get_logger=lambda:SimpleNamespace(info=lambda message:None)
    node.g1_enabled=True;node.r0_plus_enabled=False;node.m3_execution_enabled=False
    node.follow=subject().normal
    node.motion=SimpleNamespace(history=context()[1])
    return node


@pytest.mark.parametrize('parameters,policy',[({},'front_patch_v0'),
    (dict(g1_g2_preparation_policy='front_patch_v0'),'front_patch_v0'),
    (dict(g1_g2_preparation_policy='observed_entry_relevance_v1'),'observed_entry_relevance_v1'),
    (dict(g1_g2_preparation_policy='observed_entry_space_v1'),'observed_entry_space_v1')])
def test_node_preparation_parameter_default_and_explicit(tmp_path,parameters,policy):
    node=trace_node(tmp_path,**parameters);path=tmp_path/'node.jsonl'
    trace=node.create_follow_trace(str(path));trace.close()
    assert node.follow.execution.preparation_policy==policy
    header=json.loads(path.read_text().splitlines()[0])['data']
    assert header['schema']==5 and header['execution_config']['preparation_policy']==policy


@pytest.mark.parametrize('value',['observed_entry_relevance','FRONT_PATCH_V0','',True,1,None])
def test_node_rejects_unregistered_preparation_parameter(tmp_path,value):
    node=trace_node(tmp_path,g1_g2_preparation_policy=value);path=tmp_path/'bad.jsonl'
    with pytest.raises(ValueError):node.create_follow_trace(str(path))
    assert not path.exists()


def space():
    return BypassHandoffFollow(execution=HandoffExecutionConfig(preparation_policy='observed_entry_space_v1'),
                               allow_simulation_trial=True)


def test_padding_growth_zero_is_identical_and_growth_is_bounded():
    from arena_autonomy.bypass_entry_trial import _compare_entries
    from arena_autonomy.local_follow import FollowConfig
    scan,history,scene=context(.6);config=handoff_baseline()
    base=_compare_entries(scene,.6,.3,0.,.02,config=config,stationary_boundaries=True)
    zero=_compare_entries(scene,.6,.3,0.,.02,config=config,stationary_boundaries=True,padding_growth_s=0.)
    assert freeze(base)==freeze(zero)
    grown=_compare_entries(scene,.6,.3,0.,.02,config=config,stationary_boundaries=True,padding_growth_s=.1)
    pairs=[(a,b) for a in base['candidates'] for b in grown['candidates']
           if a['offset_m']==b['offset_m'] and a['transition_m']==b['transition_m'] and 'maximum_occupancy_padding_m' in a]
    assert pairs and all(abs(b['maximum_occupancy_padding_m']-a['maximum_occupancy_padding_m']-.05)<1e-9 for a,b in pairs)
    for bad in (-.01,.31,True,float('nan')):
        assert _compare_entries(scene,.6,.3,0.,.02,config=config,stationary_boundaries=True,
                                padding_growth_s=bad)['reason']=='invalid_input'


def test_entry_space_holds_while_longest_transition_remains_consistent():
    h=space();fast_normal(h,.3)
    for now in (.5,.6):
        scan,history,scene=context(now)
        output=h.command(scene,history,.3,'left',0.,.02,raw_scan=scan)
    # G1이 0.35 m/s 이하여도 위치 유지 판단은 한다. 정지/ID·자유 공간 증명은 아니다.
    assert output[2]['preparation']['applied'] and output[2]['entry_space']['hold'] is True
    assert output[0]==0. and any(t==1.2 for _,t in output[2]['entry_space']['consistent'])
    assert output[2]['entry_space']['free_space_assumed'] is False


@pytest.mark.parametrize('usable,mode',[([],'FOLLOW'),([(.16,.6)],'BYPASS')])
def test_entry_space_admits_only_execution_consistent_selection(usable,mode):
    h=space()
    h.normal.command=lambda *a,**kw:(.3,0.,dict(selector_choice='baseline',selector_sides={
        'left':dict(seen=False,present=False),'right':dict(seen=False,present=False)}))
    h.monitor.update=lambda *a:dict(ready=True,observed=True,fresh=True)
    h.bypass.command=lambda *a,**kw:(.35,.03,dict(selected=dict(offset_m=.16,transition_m=.6),entry_phase='entry'))
    h._execution_entries=lambda *a:(usable,dict(reason='stub'))
    scan,history,scene=context()
    output=h.command(scene,history,.3,'left',0.,.02,raw_scan=scan)
    assert h.mode==mode and output[2]['entry_admission']['admitted'] is (mode=='BYPASS')
    if mode=='FOLLOW':
        assert output[2]['reason']=='handoff_follow' and h.bypass.guide.path is None


@pytest.mark.parametrize('window,ready', [('compare_then_trim_v0',False),('trim_before_compare_v1',True)])
def test_expired_initial_partial_face_does_not_reset_bounded_window(window,ready):
    monitor=BlockingPersistence(HandoffExecutionConfig(persistence_window_policy=window))
    for index in range(10):
        now=round(.5+.1*index,9)
        scan,history,scene=context(now)
        # 처음 일부만 보인 조각의 중앙값은 55 mm 다르다. 마지막 0.8 s 안의
        # 조각은 10 mm만 달라졌으며, 0.9 s 전 첫 조각만 65 mm 차이다.
        shift=.055 if index==0 else (-.01 if index==9 else 0.)
        cloud=np.c_[np.ones(7)*(1.+shift),np.linspace(-.18,-.1,7)]
        with patch('arena_autonomy.bypass_handoff.front_patch',return_value=(cloud,'test')):
            result=monitor.update(scene,history)
        if index<8:
            assert not result['ready']
    assert result['ready'] is ready
    assert monitor.motion_evidence() is not ready
    if ready:
        assert result['source_span_s']>=.8-1e-9
        assert result['sample_count']>=6


def test_persistence_window_default_schema_and_explicit_node(tmp_path):
    assert HandoffExecutionConfig().persistence_window_policy=='compare_then_trim_v0'
    old=asdict(HandoffExecutionConfig());old.pop('persistence_window_policy')
    assert HandoffExecutionConfig.from_mapping(old).persistence_window_policy=='compare_then_trim_v0'
    with pytest.raises(ValueError):HandoffExecutionConfig(persistence_window_policy='unchecked')
    node=trace_node(tmp_path,g1_g2_persistence_window='trim_before_compare_v1')
    path=tmp_path/'window.jsonl';trace=node.create_follow_trace(str(path));trace.close()
    assert json.loads(path.read_text().splitlines()[0])['data']['execution_config']['persistence_window_policy']=='trim_before_compare_v1'
    node=trace_node(tmp_path,g1_g2_persistence_window=True)
    with pytest.raises(ValueError):node.create_follow_trace(str(tmp_path/'invalid-window.jsonl'))


def follow_space():
    return BypassHandoffFollow(execution=HandoffExecutionConfig(preparation_policy='observed_entry_space_v1',
        persistence_window_policy='trim_before_compare_v1',follow_entry_space_enabled=True),allow_simulation_trial=True)


def test_follow_reserve_is_explicit_and_keeps_known_zero_command():
    assert not HandoffExecutionConfig().follow_entry_space_enabled
    with pytest.raises(ValueError):HandoffExecutionConfig(follow_entry_space_enabled=True)
    with pytest.raises(ValueError):HandoffExecutionConfig(follow_entry_space_enabled=1)
    h=follow_space();scan,history,scene=context(.6)
    h.monitor.rows=[dict(source=.4),dict(source=.5)]
    target,evidence=h._follow_entry_space(scene,(0.,0.,{}),0.,0.)
    assert target==0. and not evidence['applied']
    assert evidence['free_space_assumed'] is False


def test_follow_reserve_uses_observed_distance_not_actor_speed_or_static_identity():
    h=follow_space();scan,history,scene=context(.6)
    h.monitor.rows=[dict(source=.4),dict(source=.5)]
    cloud=np.c_[np.ones(5)*1.35,np.linspace(-.18,-.1,5)]
    with patch('arena_autonomy.bypass_handoff.front_patch',return_value=(cloud,'test')):
        target,evidence=h._follow_entry_space(scene,(.8,0.,{}),0.,.8)
    assert target==pytest.approx((1.35+.135/2-1.2)/.8)
    assert evidence['applied'] and evidence['identity_or_stationarity_proven'] is False
    with patch('arena_autonomy.bypass_handoff.straight_reference',return_value=(None,{})):
        target,evidence=h._follow_entry_space(scene,(.8,0.,{}),0.,.8)
    assert target==.8 and evidence['reason']=='two_wall_straight_reference_unavailable'


def test_follow_reserve_does_not_create_missing_patch_or_cap_off_path():
    h=follow_space();scan,history,scene=context(.6)
    h.monitor.rows=[dict(source=.4),dict(source=.5)]
    for cloud,expected in [(None,'no_patch'),(np.c_[np.ones(5),np.ones(5)*.6],'patch_outside_g1_swept_arc')]:
        with patch('arena_autonomy.bypass_handoff.front_patch',return_value=(cloud,'no_patch')):
            target,evidence=h._follow_entry_space(scene,(.8,0.,{}),0.,.8)
        assert target==.8 and evidence['reason']==expected


@pytest.mark.parametrize('reason',['follow_gap_hold','follow_release_wait'])
def test_centered_handoff_accepts_valid_g1_hold_without_advancing(reason):
    h,_=stubbed(phase='centered');h.mode='BYPASS'
    h.execution=replace(h.execution,reacquire_policy='known_hold_v1')
    token=h.normal.pass_selector.sides
    h.normal.command=lambda *a,**kw:(0.,.01,dict(reason=reason,path_points=[[0.,0.],[.5,0.]],
        observed_path_ahead_m=.5,swept_clearance_m=.2))
    for now in (.5,.5):
        scan,history,scene=context(now)
        out=h.command(scene,history,.3,'left',0.,.02,raw_scan=scan)
        assert out[:2]==(0.,.01) and out[2]['reason']=='handoff_reacquire_hold'
        assert h.mode=='REACQUIRE'
    scan,history,scene=context(.6)
    out=h.command(scene,history,.3,'left',0.,.02,raw_scan=scan)
    assert out[:2]==(0.,.01) and out[2]['reason']=='handoff_return'
    assert h.mode=='FOLLOW' and h.normal.pass_selector.sides is token


@pytest.mark.parametrize('reason,path,policy',[
    ('follow_boundary_missing',[[0.,0.],[.5,0.]],'known_hold_v1'),
    ('follow_gap_hold',[],'known_hold_v1'),
    ('follow_gap_hold',[[0.,0.],[.5,0.]],'positive_command_v0')])
def test_hold_reacquire_does_not_promote_missing_path_or_legacy(reason,path,policy):
    h,_=stubbed(phase='centered');h.mode='BYPASS';h.execution=replace(h.execution,reacquire_policy=policy)
    h.normal.command=lambda *a,**kw:(0.,0.,dict(reason=reason,path_points=path,
        observed_path_ahead_m=.5,swept_clearance_m=.2))
    for now in (.5,.6):
        scan,history,scene=context(now)
        out=h.command(scene,history,.3,'left',0.,.02,raw_scan=scan)
        assert h.mode=='REACQUIRE' and out[2]['reason']!='handoff_return'


def test_node_explicit_follow_space_and_known_hold_options_are_recorded(tmp_path):
    node=trace_node(tmp_path,g1_g2_preparation_policy='observed_entry_space_v1',
        g1_g2_persistence_window='trim_before_compare_v1',g1_g2_follow_entry_space=True,
        g1_g2_reacquire_policy='known_hold_v1')
    path=tmp_path/'explicit.jsonl';trace=node.create_follow_trace(str(path));trace.close()
    cfg=json.loads(path.read_text().splitlines()[0])['data']['execution_config']
    assert cfg['follow_entry_space_enabled'] is True and cfg['reacquire_policy']=='known_hold_v1'


@pytest.mark.parametrize('options',[dict(g1_g2_follow_entry_space=True),dict(g1_g2_follow_entry_space='true'),
    dict(g1_g2_reacquire_policy='all_stops'),dict(g1_g2_reacquire_policy=True)])
def test_node_rejects_unregistered_or_incomplete_follow_space_options(tmp_path,options):
    node=trace_node(tmp_path,**options)
    with pytest.raises(ValueError):node.create_follow_trace(str(tmp_path/'invalid-combination.jsonl'))
    assert not (tmp_path/'invalid-combination.jsonl').exists()
