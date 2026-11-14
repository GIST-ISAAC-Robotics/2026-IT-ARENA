import ast
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from arena_autonomy import follow_trace as subject
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from test_decision_process import fixture

NODE_SOURCE = Path(subject.__file__).with_name('g1_g2_node.py')


def recorder(tmp_path, detail='compact_v1', name='trace.jsonl'):
    path=tmp_path/name
    return subject.FollowTrace(path, MotionConfig(), FollowConfig(), detail=detail),path


def rows(trace,path):
    trace.close()
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]


def test_full_default_equals_explicit_full_and_replays(tmp_path):
    header,sample=fixture();data=sample(.6)
    output=subject.replay_call(LocalFollow(), data, MotionConfig(**header['motion_config']))
    paths=[]
    for name, options in (('default.jsonl', {}), ('explicit.jsonl', {'detail': 'full'})):
        path=tmp_path/name;paths.append(path)
        trace=subject.FollowTrace(path,MotionConfig(**header['motion_config']),FollowConfig(), **options)
        trace.emit('call',dict(input=data,output=output));trace.emit('reset',dict(reason='STOP'))
        trace.close()
    assert paths[0].read_bytes()==paths[1].read_bytes()
    result=subject.replay_events([json.loads(line) for line in paths[1].read_text().splitlines()])
    assert result['passed'] and result['calls']==1


def test_compact_skips_snapshot_and_never_walks_large_payload(tmp_path,monkeypatch):
    def forbidden(*args,**kwargs):raise AssertionError('trace-only snapshot was evaluated')
    monkeypatch.setattr(subject,'snapshot_call',forbidden)
    trace,path=recorder(tmp_path)
    large=object()  # freeze would reject this; compact must never touch it.
    output=(.4,.1,dict(reason='handoff_follow',handoff_state='FOLLOW',path_points=large,
                     normal=dict(reason='follow_current',scan=large)))
    trace.record_call(large,large,1.,2,.3,'left',.1,.1,False,0.,output,timing_policy='host_estimated_v1')
    output[2]['normal']['reason']='mutated_after_emit'
    saved=rows(trace,path)
    event=next(row for row in saved if row['kind']=='call_summary')
    assert event['data']['output']['normal']['reason']=='follow_current'
    assert event['data']['input']['timing_verified'] is False
    assert event['data']['input']['timing_policy']=='host_estimated_v1'
    assert saved[-1]['data']['capture_errors']==0
    assert saved[0]['data']['replayable'] is False
    with pytest.raises(ValueError,match='not replay input'):subject.replay_events(saved)


def test_async_discard_reason_and_current_stop_survive_without_sensor_payload(tmp_path):
    trace,path=recorder(tmp_path)
    poison=object()
    trace.emit('call',dict(input=dict(scan=poison,history=poison,reference_s=1.),
        output=(.5,.1,dict(reason='planned',path_points=poison)),
        delivery=dict(kind='discarded',reason='decision_source_expired',source_age_s=.31),
        compute=dict(stage='plan',sequence=9,input=poison),
        current_rebase=dict(input=poison,output=dict(speed_mps=0.,steering_rad=.1,reference_s=1.2,
            reason='current_occupied',path_points=poison,independent_guard_required=True,runtime_authorized=False))))
    trace.emit('reset',dict(reason='decision_source_expired'))
    trace.emit('parallel_discard',dict(reason='old_generation',stage='current',sequence=9,generation=2))
    trace.emit('route_publication',dict(retained=False,reason='not_published_as_checked'))
    saved=rows(trace,path)
    event=next(row['data'] for row in saved if row['kind']=='call_summary')
    assert event['delivery']['reason']=='decision_source_expired'
    assert event['current_rebase']['speed_mps']==0. and event['current_rebase']['reason']=='current_occupied'
    assert event['current_rebase']['runtime_authorized'] is False
    assert [r['kind'] for r in saved]==['header','call_summary','reset','parallel_discard','route_publication','footer']
    assert saved[-1]['data']['capture_errors']==0


def test_complete_plan_audit_is_explicit_summary_not_implicit_full_capture(tmp_path):
    trace,path=recorder(tmp_path)
    meta=dict(stage='plan',generation=2,sequence=3,submitted_wall_s=4.)
    trace.emit('runtime_plan_submitted',dict(meta,input=object()))
    trace.emit('runtime_plan_completed',dict(meta,output=(.4,.1,dict(reason='ok',large=object()))))
    saved=rows(trace,path)
    assert saved[1]['kind']=='runtime_plan_submitted_summary' and saved[1]['data']==meta
    assert saved[2]['kind']=='runtime_plan_completed_summary'
    assert saved[2]['data']['output']['reason']=='ok'
    assert saved[-1]['data']['capture_errors']==0


def test_control_state_changes_braking_and_stop_are_small_actual_publication_events(tmp_path):
    trace,path=recorder(tmp_path)
    trace.record_control(1.,'RUNNING',.5,.1,True)
    trace.record_control(1.1,'RUNNING',.6,.1,True)  # positive continuation has no new event
    trace.record_control(1.2,'RUNNING',.4,.1,False)  # unissued candidate is not braking
    trace.record_control(1.3,'RUNNING',.3,.1,True)
    trace.record_control(1.4,'MOTION_STOP',0.,.1,True)
    trace.record_control(1.5,'MOTION_STOP',0.,.1,True)
    trace.record_control(1.6,'EXPLICIT_STOP',0.,0.,True,explicit_stop=True)
    saved=rows(trace,path)
    events=[r['data'] for r in saved if r['kind']=='control_event']
    assert [e['at_s'] for e in events]==[1.,1.3,1.4,1.6]
    assert events[1]['command_decreased'] and events[2]['speed_mps']==0.
    assert events[-1]['explicit_stop'] and events[-1]['published']
    assert all(e['scope']=='drive_publication_not_guard_or_mcu' for e in events)


@pytest.mark.parametrize('detail',['full','compact_v1'])
def test_path_mode_and_execution_validation_remain_required(tmp_path,detail):
    with pytest.raises(ValueError,match='must be absolute'):
        subject.FollowTrace('relative.jsonl',MotionConfig(),FollowConfig(),detail=detail)
    with pytest.raises(ValueError,match='unsupported decision trace mode'):
        subject.FollowTrace(tmp_path/'bad.jsonl',MotionConfig(),FollowConfig(),mode='unregistered',detail=detail)
    with pytest.raises(ValueError,match='typed execution'):
        subject.FollowTrace(tmp_path/'bad.jsonl',MotionConfig(),FollowConfig(),mode='g1_g2_handoff_trial',detail=detail)
    assert not (tmp_path/'bad.jsonl').exists()


def test_detail_selection_and_capture_failure_do_not_change_control(tmp_path,monkeypatch):
    with pytest.raises(ValueError,match='trace detail'):
        recorder(tmp_path,detail='off')
    trace,path=recorder(tmp_path)
    trace.writer.queue.join()
    def failed(data):raise OSError('synthetic capture failure')
    monkeypatch.setattr(trace.writer,'submit',failed)
    trace.record_control(1.,'MOTION_STOP',0.,0.,True)
    assert rows(trace,path)[-1]['data']['capture_errors']==1


def node_methods():
    tree=ast.parse(NODE_SOURCE.read_text(encoding='utf-8'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='G1G2Node')
    methods=[copy.deepcopy(n) for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('control','stop','trace_control')]
    small=ast.ClassDef(name='Node',bases=[ast.Name(id='Parent',ctx=ast.Load())],keywords=[],body=methods,decorator_list=[])
    class Parent:
        def control(self):
            self.order.append('published');self.last_speed=.3;self.last_status='RUNNING';self.command_was_published=True
            return 'same control result'
        def stop(self):self.order.append('stopped');self.last_speed=0.
    env=dict(Parent=Parent)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[small],type_ignores=[])),'staged actual node methods','exec'),env)
    return env['Node']


@pytest.mark.parametrize('detail',['full','compact_v1'])
def test_actual_node_option_reaches_trace_and_pre_emit_copy_is_only_full(tmp_path,detail):
    from arena_autonomy.bypass_handoff import HandoffExecutionConfig
    tree=ast.parse(NODE_SOURCE.read_text(encoding='utf-8'))
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='G1G2Node')
    create=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='create_follow_trace')
    construction=next(n.value for n in create.body if isinstance(n,ast.Assign)
                      and any(isinstance(t,ast.Name) and t.id=='trace' for t in n.targets))
    option=ast.FunctionDef(name='option',args=copy.deepcopy(create.args),
        body=[copy.deepcopy(create.body[0]),ast.Return(value=copy.deepcopy(construction))],decorator_list=[])
    env=dict(FollowTrace=subject.FollowTrace)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[option],type_ignores=[])),'actual trace option','exec'),env)
    def declare(name,default):
        assert (name,default)==('g1_trace_detail','full')
        return NS(value=detail)
    owner=NS(declare_parameter=declare,current_corner_budget='off',
        motion=NS(history=NS(config=MotionConfig())),
        follow=NS(config=FollowConfig(),execution=HandoffExecutionConfig()))
    trace=env['option'](owner,tmp_path/'option.jsonl')
    assert trace.detail==detail
    trace.close()
    command=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='command')
    assignments=[n for n in ast.walk(command) if isinstance(n,ast.Assign)
                 and any(isinstance(v,ast.IfExp) for v in ast.walk(n.value))
                 and isinstance(n.targets[0],ast.Subscript)]
    assert len(assignments)==1
    calls=[]
    def counted(value):calls.append(value);return 'full frozen result'
    result=object();snapshot=object();evidence={}
    env=dict(self=NS(follow_trace=trace),evidence=evidence,current=snapshot,current_result=result,
             freeze=counted,time=NS(perf_counter=lambda:1.),began=1.)
    exec(compile(ast.fix_missing_locations(ast.Module(body=copy.deepcopy(assignments),type_ignores=[])),
                 'actual pre-emit trace capture','exec'),env)
    assert calls==([result] if detail=='full' else [])
    assert evidence['current_rebase']['input'] is snapshot


@pytest.mark.parametrize('runtime',[False,True])
def test_actual_node_records_after_parent_publication_and_preserves_result(tmp_path,monkeypatch,runtime):
    from arena_autonomy import runtime_node_backend
    monkeypatch.setattr(runtime_node_backend,'control',lambda node,parent:parent())
    Node=node_methods();node=Node();trace,path=recorder(tmp_path)
    node.follow_trace=trace;node.order=[];node.motion=NS(drain_pending=lambda **kwargs:node.order.append('drain'))
    node.runtime_dual_selected=runtime;node.decision_delivery=None;node.last_steering=.1;node.now_s=lambda:1.
    assert node.control()=='same control result' and node.order==['drain','published']
    node.stop();assert node.order[-1]=='stopped'
    events=[r['data'] for r in rows(trace,path) if r['kind']=='control_event']
    assert events[0]['speed_mps']==.3 and events[0]['published']
    assert events[-1]['speed_mps']==0. and events[-1]['explicit_stop']
