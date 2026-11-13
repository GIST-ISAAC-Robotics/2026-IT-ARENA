"""명시 옵션·전달·단일 호출 계약. 실제 ROS 노드/주행을 만들지 않는다."""
import ast
import copy
from dataclasses import asdict
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from arena_autonomy.runtime_node_backend import (CORNER_POLICY, POLICY, evaluator_header,
    validate_corner_selection, validate_selection)
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.race_runtime_policy import corner_candidate
from arena_autonomy.follow_trace import FollowTrace
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.bypass_handoff import HandoffExecutionConfig, handoff_baseline


def options():
    return dict(runtime_selected=True, use_sim_time=True, motion_verified=True, motion_time_policy='verified_only')


def header():
    return dict(schema=5, controller_mode='g1_g2_handoff_trial',
        motion_config=asdict(MotionConfig()), follow_config=asdict(FollowConfig()),
        execution_config=asdict(HandoffExecutionConfig()), runtime_policy=POLICY,
        numeric_execution='native_v1', runtime_time_policy='verified_only',
        current_corner_budget=CORNER_POLICY, current_corner_environment=dict(
            use_sim_time=True, motion_timing_verified=True, backend='native2slot_v1'))


def test_off_is_unconditional_no_new_permission_and_native_registration_stays_required():
    assert not validate_corner_selection('off', **{key:None for key in options()})
    assert validate_corner_selection(CORNER_POLICY, **options())
    assert validate_selection(POLICY, execution='process_candidate',submission='fresh_native_v1',
        recheck_in_worker=True,retain_reference=True,numeric_execution='native_v1',hardware_shadow=False)
    with pytest.raises(ValueError):
        validate_selection(POLICY,execution='process_candidate',submission='fresh_native_v1',
            recheck_in_worker=True,retain_reference=True,numeric_execution='python',hardware_shadow=False)


@pytest.mark.parametrize('policy', [None,True,1,'r396','R396_v1','on'])
def test_only_registered_explicit_option(policy):
    with pytest.raises(ValueError): validate_corner_selection(policy, **options())


@pytest.mark.parametrize('key,value', [('runtime_selected',False),('use_sim_time',False),
    ('use_sim_time',1),('motion_verified',False),('motion_verified',1),('motion_time_policy','host_estimated_v1')])
def test_wrong_environment_never_enables_corner(key,value):
    values=options();values[key]=value
    with pytest.raises(ValueError):validate_corner_selection(CORNER_POLICY,**values)


@pytest.mark.parametrize('fault', ['numeric','backend','sim_time','verified','estimated','missing_environment','bad_environment'])
def test_evaluator_independently_rejects_bad_on_header_before_controller_creation(fault):
    selected=header()
    if fault=='numeric':selected['numeric_execution']='python'
    elif fault=='backend':selected['current_corner_environment']['backend']='unregistered'
    elif fault=='sim_time':selected['current_corner_environment']['use_sim_time']=False
    elif fault=='verified':selected['current_corner_environment']['motion_timing_verified']=False
    elif fault=='estimated':selected['runtime_time_policy']='host_estimated_v1'
    elif fault=='missing_environment':del selected['current_corner_environment']
    elif fault=='bad_environment':selected['current_corner_environment']='true'
    with pytest.raises(ValueError):Evaluator('current',selected)


def test_evaluator_header_off_is_exact_old_header_and_on_adds_only_selection():
    initial=dict(schema=5, marker='unchanged', numeric_execution='native_v1')
    node=NS(motion=NS(timing_policy='verified_only',verified=True),runtime_dual_selected=True,
        current_corner_budget='off',get_parameter=lambda key:NS(value=True))
    before=copy.deepcopy(initial)
    off=evaluator_header(node,initial)
    assert off==dict(initial,runtime_policy=POLICY,runtime_time_policy='verified_only') and initial==before
    node.current_corner_budget=CORNER_POLICY
    on=evaluator_header(node,initial)
    assert {k:v for k,v in on.items() if k not in ('current_corner_budget','current_corner_environment')}==off
    assert on['current_corner_budget']==CORNER_POLICY
    assert on['current_corner_environment']==dict(use_sim_time=True,motion_timing_verified=True,backend='native2slot_v1')
    node.current_corner_budget='off';node.motion.timing_policy='host_estimated_v1';node.motion.verified=False
    node.get_parameter=lambda key:NS(value=False)
    assert evaluator_header(node,initial)==dict(initial,runtime_policy=POLICY,runtime_time_policy='host_estimated_v1')


def test_trace_off_bytes_are_unchanged_and_on_header_is_explicit(tmp_path):
    def record(name,**extra):
        path=tmp_path/name
        trace=FollowTrace(path,MotionConfig(),FollowConfig(),mode='g1_g2_handoff_trial',
            execution_config=HandoffExecutionConfig(),**extra)
        trace.close()
        lines=path.read_bytes().splitlines()
        assert json.loads(lines[0])['kind']=='header' and json.loads(lines[-1])['kind']=='footer'
        return lines[0]
    default=record('default.jsonl');off=record('off.jsonl',current_corner_budget='off')
    assert default==off
    old=json.loads(default)['data']
    new=json.loads(record('on.jsonl',current_corner_budget=CORNER_POLICY))['data']
    assert 'current_corner_budget' not in old
    assert new.pop('current_corner_budget')==CORNER_POLICY and new==old


def test_trace_invalid_scope_rejected_before_file_creation(tmp_path):
    path=tmp_path/'bad.jsonl'
    with pytest.raises(ValueError):FollowTrace(path,MotionConfig(),FollowConfig(),current_corner_budget=CORNER_POLICY)
    assert not path.exists()


@pytest.mark.parametrize('choice', ['off',CORNER_POLICY])
def test_g1_node_declaration_reaches_trace_without_ros_node_or_output(tmp_path,choice):
    from arena_autonomy.g1_g2_node import G1G2Node
    parameters={'g1_g2_simulation_trial':True,'g1_current_corner_budget':choice,'use_sim_time':True}
    def declare(name,default):return NS(value=parameters.setdefault(name,default))
    node=NS(declare_parameter=declare,get_parameter=lambda name:NS(value=parameters[name]),
        hardware_inputs=False,g1_enabled=True,r0_plus_enabled=False,m3_execution_enabled=False,
        follow=NS(config=handoff_baseline()),motion=NS(history=NS(config=MotionConfig())),
        get_logger=lambda:NS(info=lambda message:None))
    trace=G1G2Node.create_follow_trace(node,str(tmp_path/'node.jsonl'))
    trace.close()
    recorded=json.loads((tmp_path/'node.jsonl').read_text().splitlines()[0])['data']
    assert node.current_corner_budget==choice
    assert recorded.get('current_corner_budget','off')==choice
    assert ('current_corner_budget' in recorded)==(choice!='off')


@pytest.mark.parametrize('timing', [False,None,1])
def test_on_evaluator_rejects_unverified_current_before_calculation(timing):
    evaluator=Evaluator.__new__(Evaluator)
    evaluator.stage='current';evaluator.time_policy='verified_only';evaluator.corner_budget=True
    with pytest.raises(ValueError,match='verified input timing'):
        evaluator(dict(current=dict(timing_verified=timing)))


@pytest.mark.parametrize('enabled', [False,True])
def test_current_baseline_called_once_and_noneligible_does_not_call_proposal(monkeypatch,enabled):
    import arena_autonomy.follow_trace as trace
    import arena_autonomy.scene_motion as scenes
    calls=[];before=dict(speed_mps=.2,steering_rad=.1,reason='original_positive')
    evaluator=Evaluator.__new__(Evaluator)
    evaluator.stage='current';evaluator.time_policy='verified_only';evaluator.corner_budget=enabled
    evaluator.follow=NS(config=FollowConfig());evaluator.motion=MotionConfig()
    def base(*args,**kwargs):calls.append('baseline');return before
    evaluator.current_policy=NS(rebase=base)
    info=dict(reference_s=1.,timing_verified=True,epoch=0,rear_half_angle_deg=0.)
    monkeypatch.setattr(trace,'restore_call',lambda *args:(info,object(),object()))
    monkeypatch.setattr(scenes,'motion_scene',lambda *args,**kwargs:object())
    def forbidden(*args,**kwargs):raise AssertionError('noneligible attempted distance proposal')
    monkeypatch.setattr(corner_candidate,'propose',forbidden)
    result=evaluator(dict(output=[.2,.1,{}],submitted={},current={'timing_verified':True}))
    assert result==before and calls==['baseline']


def test_on_plan_evaluator_does_not_apply_current_budget(monkeypatch):
    import arena_autonomy.follow_trace as trace
    evaluator=Evaluator.__new__(Evaluator)
    evaluator.stage='plan';evaluator.time_policy='verified_only';evaluator.corner_budget=True
    evaluator.follow=object();evaluator.motion=MotionConfig()
    plan=[.45,.1,{'reason':'same_plan'}];calls=[]
    def replay(*args):calls.append(args);return plan
    monkeypatch.setattr(trace,'replay_call',replay)
    assert evaluator(dict(timing_verified=True))==plan and len(calls)==1


def test_eligible_candidate_uses_injected_proof_once_after_single_core_and_region(monkeypatch):
    calls=[]
    before=dict(speed_mps=0.,steering_rad=.1,reference_s=1.,path_source_s=.9,
        original_reference_s=.9,path_points=[[0.,0.],[1.,0.]],reason='held',current_route_proof={})
    current=dict(speed_mps=0.,previous_steering_rad=-.1,dt_s=.1,
        scan={'measurements':dict(ranges_m=[2.,2.],angles_rad=[0.,.1],acquired_at_s=[.9,.91])})
    monkeypatch.setattr(corner_candidate,'eligible',lambda *a:True)
    def proposal(*args,**kwargs):calls.append('proposal');return dict(accepted=True,proposed_target_mps=.09,allowed_distance_m=.03)
    monkeypatch.setattr(corner_candidate,'propose',proposal)
    def core(*args):calls.append('core');return {**before,'speed_mps':.09,'reason':'rebase_current_follow'}
    def region(*args,context,**kwargs):
        import numpy as np
        calls.append('region')
        assert args[3:5]==(.1,-.1)
        context.update(queries=np.array([[0.,.08]]),free=np.array([False]),masked=np.array([False]),padding=np.array([.01,.01]))
        return dict(distance_m=.03,queries=1,unobserved=1)
    monkeypatch.setattr(corner_candidate,'ps',NS(motion_region=region,lateral_config=lambda cfg:cfg,
        SelectorConfig=lambda:None,region_blocked=lambda *a,**kw:None))
    def proof(*args):calls.append('proof');return dict(accepted=True,unknown_filled=False)
    result=corner_candidate.try_from_current_result(before,[.45,.1,{}],{},current,object(),object(),
        FollowConfig(),core_rebase=core,proof_callback=proof,require_current_region=True)
    assert calls==['proposal','core','region','proof']
    assert result['speed_mps']==.09 and result[corner_candidate.DIAGNOSTIC]['applied']
    assert result['path_source_s']==before['path_source_s'] and result['path_points']==before['path_points']


def test_runtime_packaging_has_no_artifact_dynamic_import_or_global_proof_replacement():
    import arena_autonomy.race_runtime_policy.evaluator as evaluator
    base=Path(evaluator.__file__).parent
    for name in ('corner_budget.py','corner_candidate.py','range_mask_component.py','profile_evidence.py'):
        text=(base/name).read_text(encoding='utf8');tree=ast.parse(text)
        assert 'artifacts/' not in text and 'importlib' not in text
        assert not any(isinstance(node,(ast.Import,ast.ImportFrom)) and (
            any(alias.name=='importlib' for alias in node.names) or getattr(node,'module','')=='importlib') for node in ast.walk(tree))
    assert 'proof_callback' in (base/'corner_candidate.py').read_text(encoding='utf8')
