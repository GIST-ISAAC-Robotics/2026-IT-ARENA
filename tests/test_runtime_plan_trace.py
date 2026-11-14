"""제출 소유권/폐기/reset/기록 실패와 실제 Lane spawn의 좁은 재생 계약. ROS/장치 없음."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import time
from types import SimpleNamespace as NS

import pytest
from arena_autonomy.follow_trace import FollowTrace
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.runtime_dual_delivery import RuntimeDualDelivery
from arena_autonomy.runtime_dual_state import RuntimeDualProcess
from arena_autonomy import runtime_node_backend as backend
from test_runtime_dual_slots import FakeLane, plan, snapshot, unused_factory


class TraceLane(FakeLane):
    def offer(self,*args,**kwargs):
        super().offer(*args,**kwargs)
        self.inflight['submitted_wall_s']=100.
    def complete(self,output):
        super().complete(output)
        self.completed.update(submitted_wall_s=100.,began_wall_s=100.01,
            completed_wall_s=100.02,received_wall_s=100.03)


def worker():
    return RuntimeDualProcess({},evaluator_factory=unused_factory,supported=lambda output:output[0]>0,
        lane_factory=TraceLane)


def recorder(tmp_path):
    path=tmp_path/'trace.jsonl'
    return FollowTrace(path,MotionConfig(),FollowConfig()),path


def events(trace,path):
    trace.close()
    return [json.loads(line) for line in path.read_text(encoding='utf8').splitlines()]


def by_kind(rows,kind):return [r['data'] for r in rows if r['kind']==kind]


def align(delivery):assert not delivery.offer(snapshot(1.,.9))


def test_default_off_never_reads_capture_fields_or_emits():
    class Worker:
        def offer_plan(self,data):return True
        def collect(self):return [dict(stage='plan',output=['existing result'])]
        @property
        def plan(self):raise AssertionError('OFF inspected lane metadata')
        @property
        def plan_input(self):raise AssertionError('OFF inspected owned capture')
    def forbidden(*args):raise AssertionError('OFF emitted new event')
    d=RuntimeDualDelivery(Worker(),lambda:1.12,forbidden)
    align(d);assert d.offer(snapshot(1.1,1.));d.collect()
    assert d.counts==dict(submitted=1,applied=0,discarded=0,invalidated=0)


def test_on_and_off_parent_discard_and_output_are_identical(tmp_path):
    results=[]
    for enabled in (False,True):
        folder=tmp_path/str(enabled);folder.mkdir();trace,path=recorder(folder);w=worker()
        d=RuntimeDualDelivery(w,lambda:1.12,trace.emit,complete_plan_trace=enabled)
        align(d);assert d.offer(snapshot(1.1,1.));w.plan.complete(plan(0.))
        result=d.poll(1.5,previous_steering_rad=.1,side='left',motion_epoch=0,sensor_epoch=0)
        assert result['kind']=='discarded' and result['reason']=='decision_source_expired'
        results.append((result,copy.deepcopy(d.counts)));rows=events(trace,path);w.close()
        assert len(by_kind(rows,'runtime_plan_submitted'))==int(enabled)
        assert len(by_kind(rows,'runtime_plan_completed'))==int(enabled)
        assert not by_kind(rows,'route_publication')
    assert results[0]==results[1]


def test_success_only_and_parent_owned_input_remains_after_mutation_and_discard(tmp_path):
    trace,path=recorder(tmp_path);w=worker();d=RuntimeDualDelivery(w,lambda:1.12,trace.emit,complete_plan_trace=True)
    align(d);data=snapshot(1.1,1.)
    data.update(timing_verified=False,timing_policy='host_estimated_v1',history=dict(wheels=dict(times=[1.],values=[-.02])))
    data['scan']['measurements'].update(time_basis='unverified_estimate',ranges_m=[1.,{'nonfinite':'inf'}])
    original=copy.deepcopy(data);assert d.offer(data)
    data['scan']['measurements']['ranges_m'][0]=999.;data['history']['wheels']['values'][0]=99.
    assert w.plan_input==original
    d.clock=lambda:1.23;assert not d.offer(snapshot(1.2,1.1))  # 실제 occupied offer 실패
    output=plan(0.);w.plan.complete(output);d.collect()
    w.saved_plan['input']['scan']['measurements']['ranges_m'][0]=88.
    w.saved_plan['output'][2]['reason']='mutated after capture'
    w.discard_plan((0,1));rows=events(trace,path);w.close()
    submitted=by_kind(rows,'runtime_plan_submitted');completed=by_kind(rows,'runtime_plan_completed')
    assert len(submitted)==len(completed)==1 and submitted[0]['input']==original
    assert completed[0]['output']==output and 'input' not in completed[0]
    assert {k:submitted[0][k] for k in ('generation','sequence','reset_reason','submitted_wall_s')}==dict(
        generation=0,sequence=1,reset_reason=None,submitted_wall_s=100.)


def test_generation_reset_keeps_submission_but_does_not_invent_late_completion(tmp_path):
    trace,path=recorder(tmp_path);w=worker();d=RuntimeDualDelivery(w,lambda:1.12,trace.emit,complete_plan_trace=True)
    align(d);assert d.offer(snapshot(1.1,1.))
    w.invalidate('decision_source_expired');w.plan.complete(plan(0.));d.collect()
    assert w.old_generation==1 and w.saved_plan is None
    d.clock=lambda:1.32;assert d.offer(snapshot(1.3,1.2));w.plan.complete(plan(0.));d.collect()
    rows=events(trace,path);w.close()
    submitted=by_kind(rows,'runtime_plan_submitted');completed=by_kind(rows,'runtime_plan_completed')
    assert [(s['generation'],s['sequence'],s['reset_reason']) for s in submitted]==[(0,1,None),(1,2,'decision_source_expired')]
    assert [(s['generation'],s['sequence']) for s in completed]==[(1,2)]


def test_repeated_current_for_same_plan_does_not_duplicate_plan_capture(tmp_path):
    trace,path=recorder(tmp_path);w=worker();d=RuntimeDualDelivery(w,lambda:1.12,trace.emit,complete_plan_trace=True)
    align(d);assert d.offer(snapshot(1.1,1.));output=plan();output[2]['normal']['path_source_s']=1.1
    w.plan.complete(output);d.collect()
    assert w.offer_next_current(snapshot(1.2,1.1),now_s=1.21)
    w.current.complete(dict(speed_mps=.4,steering_rad=.2,reference_s=1.2));d.collect()
    token=(w.ready['generation'],w.ready['sequence'])
    w.commit_published(token,now_s=1.21,previous_steering_rad=.1,side='left',motion_epoch=0,sensor_epoch=0)
    assert w.offer_next_current(snapshot(1.3,1.2),now_s=1.31)
    w.current.complete(dict(speed_mps=.4,steering_rad=.2,reference_s=1.3));d.collect();d.collect()
    rows=events(trace,path);w.close()
    assert len(by_kind(rows,'runtime_plan_submitted'))==len(by_kind(rows,'runtime_plan_completed'))==1


@pytest.mark.parametrize('failure',['freeze','writer'])
def test_trace_failure_preserves_control_and_appears_in_footer(tmp_path,monkeypatch,failure):
    import arena_autonomy.follow_trace as trace_module
    trace,path=recorder(tmp_path);trace.writer.queue.join()
    def failed(*args):raise OSError('capture failure')
    w=worker();d=RuntimeDualDelivery(w,lambda:1.12,trace.emit,complete_plan_trace=True)
    with monkeypatch.context() as patch:
        if failure=='freeze':patch.setattr(trace_module,'freeze',failed)
        else:patch.setattr(trace.writer,'write',failed)
        align(d);assert d.offer(snapshot(1.1,1.));w.plan.complete(plan(0.));d.collect()
        assert w.saved_plan['output']==plan(0.) and d.counts['submitted']==1
        trace.writer.queue.join()
    rows=events(trace,path);w.close();footer=rows[-1]
    assert footer['kind']=='footer'
    assert footer['data']['capture_errors' if failure=='freeze' else 'failed']==2


def node(trace,enabled):
    return NS(complete_plan_trace=enabled,follow_trace=trace,now_s=lambda:1.12,runtime_dual_selected=True,
        current_corner_budget='r396_v1',observation_input_scope='host_estimated_crop_v1',
        execution_environment='hardware_runtime_candidate_v1',motion=NS(timing_policy='host_estimated_v1',verified=False,mode='both'),
        has_parameter=lambda k:k in ('use_sim_time','lidar_input_representation'),
        get_parameter=lambda k:NS(value={'use_sim_time':False,'lidar_input_representation':'native_estimated_v1'}[k]))


@pytest.mark.parametrize('enabled',[False,True])
def test_factory_header_is_exact_actual_selection_and_only_on(tmp_path,monkeypatch,enabled):
    import arena_autonomy.runtime_dual_state as state
    trace,path=recorder(tmp_path);captured=[]
    def factory(selected,**kwargs):captured.append(copy.deepcopy(selected));return NS()
    monkeypatch.setattr(state,'RuntimeDualProcess',factory)
    n=node(trace,enabled);header=dict(schema=5,numeric_execution='native_v1',nested=dict(values=[1,2]))
    d=backend.create_delivery(n,header);header['nested']['values'][0]=999
    rows=events(trace,path);actual=by_kind(rows,'runtime_evaluator_header')
    assert d.complete_plan_trace is enabled and len(actual)==int(enabled)
    if enabled:
        assert actual[0]['header']==captured[0]
        assert actual[0]['publication_proven'] is False
        assert actual[0]['header']['current_corner_environment']['motion_timing_verified'] is False
        assert actual[0]['header']['runtime_time_policy']=='host_estimated_v1'


def test_actual_node_option_defaults_off_and_requires_registered_runtime():
    source=Path(backend.__file__).with_name('g1_g2_node.py')
    cls=next(n for n in ast.parse(source.read_text(encoding='utf8')).body if isinstance(n,ast.ClassDef) and n.name=='G1G2Node')
    init=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='__init__')
    index=next(i for i,n in enumerate(init.body) if isinstance(n,ast.Assign) and
        any(isinstance(t,ast.Attribute) and t.attr=='complete_plan_trace' for t in n.targets))
    guard=next(n for n in init.body[index+1:] if isinstance(n,ast.If)
        and any(isinstance(v,ast.Attribute) and v.attr=='complete_plan_trace' for v in ast.walk(n.test)))
    f=ast.parse('def option(self):\n pass').body[0];f.body=copy.deepcopy([init.body[index],guard])
    env={};exec(compile(ast.fix_missing_locations(ast.Module(body=[f],type_ignores=[])),'actual option boundary','exec'),env)
    def owner(selected,requested=None):
        def declare(name,default):
            assert name=='g1_complete_plan_trace' and default is False
            return NS(value=default if requested is None else requested)
        return NS(runtime_dual_selected=selected,declare_parameter=declare)
    n=owner(False);env['option'](n);assert n.complete_plan_trace is False
    with pytest.raises(ValueError,match='registered runtime'):env['option'](owner(False,True))
    env['option'](owner(True,True))


class StatefulEvaluator:
    """실제 Lane의 전달·reset 순서를 검증하는 작은 결정론 evaluator. 실제 정책 재현은 아니다."""
    def __init__(self,stage,header):self.stage,self.header,self.total=stage,header,0
    def reset(self,reason):self.total=0;self.reset_reason=reason
    def __call__(self,data):
        assert self.stage=='plan'
        self.total+=data['marker']
        return [0.,.2,dict(reason='stored_test_plan',total=self.total,reset_reason=self.reset_reason,
            input_sha256=hashlib.sha256(json.dumps(data,sort_keys=True).encode()).hexdigest(),
            time_policy=self.header['runtime_time_policy'],timing_verified=data['timing_verified'])]


def stateful_factory(stage,header):return StatefulEvaluator(stage,header)


def test_actual_lane_spawn_records_discarded_plans_and_exact_stateful_replay(tmp_path,monkeypatch):
    import arena_autonomy.race_runtime_policy.evaluator as policy
    monkeypatch.setattr(policy,'evaluator_factory',stateful_factory)
    monkeypatch.setattr(policy,'supported',lambda output:False)
    trace,path=recorder(tmp_path);n=node(trace,True);d=backend.create_delivery(n,dict(schema=5,numeric_execution='native_v1'))
    w=d.worker;children=[w.plan.process,w.current.process]
    try:
        align(d)
        for index,ref in enumerate((1.1,1.3,1.5)):
            if index==2:w.invalidate('decision_source_expired')
            d.clock=lambda ref=ref:ref+.02
            data=snapshot(ref,ref-.1);data.update(marker=index+1,timing_verified=False,timing_policy='host_estimated_v1')
            data['scan']['measurements']['time_basis']='unverified_estimate'
            assert d.offer(data);data['marker']=999
            deadline=time.monotonic()+5.
            while w.saved_plan is None:
                w.connection.poll(.02);d.collect()
                assert w.fault is None and time.monotonic()<deadline
            # 부모가 나중에 만료 폐기해도 계산 입력/결과는 이미 각각 한 번 남아 있다.
            result=d.poll(ref+.4,previous_steering_rad=.1,side='left',motion_epoch=0,sensor_epoch=0)
            assert result['kind']=='discarded'
    finally:
        cleanup=w.close();rows=events(trace,path)
    assert cleanup['exitcode']==0 and not cleanup['forced'] and not cleanup['alive']
    assert all(p.exitcode==0 and not p.is_alive() for p in children)
    selected=by_kind(rows,'runtime_evaluator_header')[0]['header']
    replay=StatefulEvaluator('plan',selected);generation=None
    submitted=by_kind(rows,'runtime_plan_submitted');completed=by_kind(rows,'runtime_plan_completed')
    assert len(submitted)==len(completed)==3
    indexed={(r['generation'],r['sequence']):r for r in completed}
    for row in submitted:
        if row['generation']!=generation:replay.reset(row['reset_reason']);generation=row['generation']
        result=indexed[(row['generation'],row['sequence'])]
        assert replay(row['input'])==result['output']
        assert row['submitted_wall_s']==result['submitted_wall_s']
        assert result['submitted_wall_s']<=result['began_wall_s']<=result['completed_wall_s']<=result['received_wall_s']
        assert 'input' not in result
    assert [r['output'][2]['total'] for r in completed]==[1,3,3]
    assert rows[-1]['data']['capture_errors']==rows[-1]['data']['failed']==rows[-1]['data']['dropped']==0
    (tmp_path/'spawn_receipt.json').write_text(json.dumps(dict(
        children=[dict(pid=p.pid,exitcode=p.exitcode,alive=p.is_alive()) for p in children],
        cleanup=cleanup,submitted=len(submitted),completed=len(completed),exact_replayed=len(submitted),
        generation_sequence=[(r['generation'],r['sequence']) for r in submitted],
        reset_reasons=[r['reset_reason'] for r in submitted],physical_actuation=False),indent=2)+'\n',encoding='utf8')
