"""원 r203과 실제 시각 기반 정지 물체 근거를 동일 입력에서 대조한다."""
from pathlib import Path
from collections import Counter
import hashlib,importlib.util,json,statistics
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
trace=A.parent/'released_mount_runtime_r203_v1/two_actor_run1/run1/follow_decisions.jsonl'
digest=hashlib.sha256(trace.read_bytes()).hexdigest()
assert digest=='f4c939829cb7b0a90d07f13f319eac69054c1e87211edc29d06720fa11ad6a93'
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
route=load('r208_route',A.parent/'expiry_route_reference_r186_v1/candidate_v2.py');route.install()
events=[json.loads(line) for line in trace.open()]
before,bm=make_controller(events[0]['data'])
load('r208_candidate',A/'candidate.py').install()
after,am=make_controller(events[0]['data'])
counts=Counter();rows=[];errors=[];intervals=[];previous=None
for e in events[1:]:
    d=e['data']
    if e['kind']=='reset':
        for controller in (before,after):
            if d.get('reason')=='decision_source_expired':route.reset_reference(controller)
            else:controller.reset()
    if e['kind']!='call':continue
    now=d['input']['reference_s']
    if previous is not None:intervals.append(now-previous)
    previous=now
    b=freeze(replay_call(before,d['input'],bm));a=freeze(replay_call(after,d['input'],am));counts['calls']+=1
    if not equal_output(comparable_decision(b),comparable_decision(d['output'])):errors.append(e['seq'])
    bn,an=b[2].get('normal',{}),a[2].get('normal',{})
    before_static=[k for k,v in bn.get('selector_evidence',{}).items() if isinstance(v,dict) and v.get('world_fixed_return')]
    after_static=[k for k,v in an.get('selector_evidence',{}).items() if isinstance(v,dict) and v.get('world_fixed_return')]
    target=bn.get('selector_reason')=='guard_curve_rejected_toward_left'
    counts['before_static']+=bool(before_static);counts['after_static']+=bool(after_static)
    counts['stop_to_positive']+=b[0]<=0<a[0];counts['new_stop']+=a[0]<=0<b[0]
    if target:
        counts['target']+=1;counts['target_positive']+=a[0]>0
    if target or before_static!=after_static or b[:2]!=a[:2]:
        rows.append(dict(seq=e['seq'],reference_s=now,before=b[:2],after=a[:2],target=target,
            before_mode=bn.get('selector_mode'),after_mode=an.get('selector_mode'),
            before_reason=bn.get('selector_reason'),after_reason=an.get('selector_reason'),
            before_static=before_static,after_static=after_static))
summary=dict(counts=dict(counts),baseline_mismatches=errors,trace_sha256=digest,
    reference_interval_s=dict(min=min(intervals),median=statistics.median(intervals),max=max(intervals)),
    nominal_control_dt_s=sorted({e['data']['input']['dt_s'] for e in events if e['kind']=='call'}),
    scope='fixed recorded inputs and nominal previous commands, not a new driving trajectory')
with (A/'comparison.json').open('x') as f:json.dump(dict(summary=summary,rows=rows),f,indent=2);f.write('\n')
print(json.dumps(summary));assert not errors
