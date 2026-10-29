"""명시 G2 실행 계약·기존 ROS 명령 경계·판단 기록/재생을 확인한다."""
import json
import math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from arena_autonomy.bypass_entry_follow import BypassEntryFollow,BypassExecutionConfig
from arena_autonomy.follow_trace import FollowTrace,replay_events,controller_mode,equal_output,freeze
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig,MotionHistory
from arena_autonomy.local_pursuit import LocalPursuit
from arena_autonomy.scene_motion import motion_scene
from probe_bypass_entry_sequence import render_scan


def inputs(now=.5):
    physical=[dict(t=t,rear_pose=[-.0675,0.,0.],speed=0.,steering=0.) for t in (0.,.8)]
    scan=render_scan(physical,now,-.15)
    history=MotionHistory(MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06))
    for t in np.arange(.2,.81,.01):history.add_mean_wheel(float(t),0.);history.add_gyro(float(t),0.)
    return scan,history


def test_bypass_cannot_be_created_without_explicit_trial():
    with pytest.raises(ValueError):BypassEntryFollow()
    with pytest.raises(ValueError):BypassExecutionConfig(tracked_wall_fit=True,wall_reanchor=False)
    with pytest.raises(ValueError):BypassExecutionConfig(wide_entry=1)
    follow=BypassEntryFollow(allow_simulation_trial=True)
    assert controller_mode(follow)=='g2_entry_trial'


def test_ros_command_stage_and_trace_replay_match_without_new_input_sources(tmp_path):
    scan,history=inputs();config=FollowConfig()
    path=tmp_path/'g2.jsonl';execution=BypassExecutionConfig()
    trace=FollowTrace(path,history.config,config,mode='g2_entry_trial',execution_config=execution)
    node=LocalPursuit.__new__(LocalPursuit)
    node.motion=SimpleNamespace(history=history,last_meta={'reference_stamp_s':.5},verified=True,counts={'resets':0})
    node.follow=BypassEntryFollow(config,execution,allow_simulation_trial=True)
    node.follow_trace=trace;node.side='left';node.last_steering=0.;node.settings={'control_rate_hz':50.}
    node.rear_blind_half_angle=30.
    oracle=BypassEntryFollow(config,execution,allow_simulation_trial=True)
    outputs=[]
    for i,now in enumerate((.5,.52,.6,.62,.7,.72)):
        node.scan=inputs(.5 if i<2 else .6 if i<4 else .7)[0]
        node.motion.last_meta['reference_stamp_s']=now
        scene=motion_scene(node.scan,history,now,timing_verified=True,rear_half_angle_deg=30.)
        expected=oracle.command(scene,history,0.,'left',node.last_steering,.02)
        out=node.command(None);outputs.append(out)
        assert equal_output(freeze(out),freeze(expected))
        assert out[0]<=.35 and out[2]['output_requires_independent_guard']
        node.last_steering=out[1]
        if i==3:node.reset_follow('SENSOR_STOP');oracle.reset()
    node.scan.header.frame_id='bad_frame'
    bad=node.command(None)
    assert bad[:2]==(0.,0.) and bad[2]['mode']=='g2_entry_trial'
    trace.close()
    events=[json.loads(line) for line in path.read_text().splitlines()]
    assert events[0]['data']['schema']==4
    report=replay_events(events)
    assert report['passed'] and report['matched']==7
    assert any(o[0]>0 for o in outputs)
    assert all(not o[2]['automatic_race_mode_selection'] for o in outputs)


def test_g2_trace_rejects_missing_typed_options(tmp_path):
    with pytest.raises(ValueError):FollowTrace(tmp_path/'bad.jsonl',MotionConfig(),FollowConfig(),mode='g2_entry_trial')


def test_default_launches_do_not_select_g2_and_node_reuses_existing_drive_boundary():
    root=Path(__file__).resolve().parents[1]
    for name in ('simulation.launch.py','demo.launch.py','local_pursuit.launch.py','g1_follow.launch.py'):
        assert 'g2_entry_trial' not in (root/'src/arena_bringup/launch'/name).read_text()
    from arena_autonomy.g2_entry_node import G2EntryNode
    # command는 상태 발행용 detail 요약만 덧붙인다. 명령 값·기록은 아래 검사로 LocalPursuit와 대조한다.
    assert G2EntryNode.control is LocalPursuit.control
    assert G2EntryNode.reset_follow is LocalPursuit.reset_follow


def test_g2_status_detail_is_json_safe_while_trace_keeps_full_decision(tmp_path):
    from arena_autonomy.g2_entry_node import G2EntryNode,status_detail
    scan,history=inputs();config=FollowConfig();execution=BypassExecutionConfig()
    nodes={}
    for name,kind in (('base',LocalPursuit),('g2',G2EntryNode)):
        node=kind.__new__(kind)
        node.motion=SimpleNamespace(history=history,last_meta={'reference_stamp_s':.5},verified=True,counts={'resets':0})
        node.follow=BypassEntryFollow(config,execution,allow_simulation_trial=True)
        node.follow_trace=FollowTrace(tmp_path/(name+'.jsonl'),history.config,config,mode='g2_entry_trial',execution_config=execution)
        node.side='left';node.last_steering=0.;node.settings={'control_rate_hz':50.};node.rear_blind_half_angle=30.
        nodes[name]=node
    selected=0
    for i,now in enumerate((.5,.52,.6,.62,.7,.72)):
        scan=inputs(.5 if i<2 else .6 if i<4 else .7)[0]
        outputs={}
        for name,node in nodes.items():
            node.scan=scan;node.motion.last_meta['reference_stamp_s']=now
            outputs[name]=node.command(None)
        base,g2=outputs['base'],outputs['g2']
        assert g2[:2]==base[:2]
        # 기존 WallFollow.control과 같은 직렬화 조건. reason은 상태 이름으로 쓰인다.
        status={'state':'X',**g2[2]}
        json.dumps(status,ensure_ascii=False,allow_nan=False)
        assert g2[2]['reason']==base[2]['reason'] and g2[2]['mode']=='g2_entry_trial'
        assert 'candidates' not in g2[2] and g2[2]['candidate_count']==len(base[2].get('candidates',[]))
        assert g2[2]==status_detail(base[2])
        selected+=g2[2].get('selected') is not None
        for node in nodes.values():node.last_steering=base[1]
    assert selected>0
    for node in nodes.values():node.follow_trace.close()
    traces={name:[json.loads(line) for line in (tmp_path/(name+'.jsonl')).read_text().splitlines()] for name in nodes}
    full=[e['data']['output'][2] for e in traces['g2'] if e['kind']=='call']
    assert any(len(d.get('candidates',[]))>1 and 'path_points' in d['candidates'][0] for d in full)
    assert [e['data'] for e in traces['g2'] if e['kind']=='call']==[e['data'] for e in traces['base'] if e['kind']=='call']
    assert replay_events(traces['g2'])['passed']
    nonfinite=status_detail({'reason':'x','selected':{'speed_mps':float('inf')},'candidates':[{'eligible':np.bool_(True)}]})
    json.dumps(nonfinite,allow_nan=False)
    assert nonfinite['eligible_candidates']==1 and nonfinite['selected']['speed_mps']=={'nonfinite':'inf'}
