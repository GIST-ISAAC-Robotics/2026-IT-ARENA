"""실제 고정 입력의 깊이 질의에서 행 묶음 크기만 대조한다. 제어 연결 없음."""
import argparse
import hashlib
import inspect
import json
from pathlib import Path
import sys
import time
import numpy as np
import arena_autonomy.follow_trace as trace
import arena_autonomy.observation_support as support

def candidate(size):
    source=inspect.getsource(original)
    assert source.count('range(0, len(q), 128)')==1 and source.count('q[start:start+128]')==1
    source=source.replace('range(0, len(q), 128)',f'range(0, len(q), {size})')
    source=source.replace('q[start:start+128]',f'q[start:start+{size}]')
    namespace=dict(support.__dict__)
    exec(compile(source,f'<depth_batch_{size}>','exec'),namespace)
    return namespace['depth_support']

p=argparse.ArgumentParser();p.add_argument('input',type=Path);p.add_argument('--output',type=Path,required=True)
args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
events=[json.loads(line) for line in args.input.open()]
before=hashlib.sha256(args.input.read_bytes()).hexdigest()
original=support.depth_support;source=inspect.getsource(original)
functions={size:candidate(size) for size in (128,256,512,1024)}
saved=[];collect=False;call_index=0;old_replay=trace.replay_call
def observed(scene,queries,padding,*,cycle=None):
    result=original(scene,queries,padding,cycle=cycle)
    if collect:saved.append((scene,np.asarray(queries).copy(),np.asarray(padding).copy(),cycle,result.copy()))
    return result
def replay(*a,**kw):
    global collect,call_index
    collect=call_index%20 in (0,1);call_index+=1
    try:return old_replay(*a,**kw)
    finally:collect=False
# 미리 정적 import가 된 함수 참조를 같은 래퍼로 연결하며 재생 뒤 복구한다.
bindings=[]
for module in list(sys.modules.values()):
    if not getattr(module,'__name__','').startswith('arena_autonomy'):continue
    for name,value in list(vars(module).items()):
        if value is original:bindings.append((module,name));setattr(module,name,observed)
trace.replay_call=replay
try:result=trace.replay_events(events)
finally:
    trace.replay_call=old_replay
    for module,name in bindings:setattr(module,name,original)
assert result['passed']
timings={size:[] for size in functions};mismatches=[];orders=[]
for repeat in range(3):
    order=list(functions)
    order=order[repeat:]+order[:repeat]
    orders.append(order)
    for size in order:
        function=functions[size]
        for index,(scene,queries,padding,cycle,expected) in enumerate(saved):
            start=time.perf_counter_ns();actual=function(scene,queries,padding,cycle=cycle)
            elapsed=(time.perf_counter_ns()-start)/1e6
            if not np.array_equal(actual,expected):mismatches.append(dict(size=size,case=index))
            if repeat:timings[size].append(elapsed)
def stats(values):
    return dict(count=len(values),sum_ms=sum(values),p50_ms=float(np.percentile(values,50)),
                p95_ms=float(np.percentile(values,95)),max_ms=max(values))
summary=dict(scope='captured_function_cost_only_not_full_decision_latency',
    calls=result['calls'],matched=result['matched'],captured_depth_calls=len(saved),
    queries=sum(len(row[1]) for row in saved),source_sha256=hashlib.sha256(source.encode()).hexdigest(),
    input_sha256=before,input_unchanged=before==hashlib.sha256(args.input.read_bytes()).hexdigest(),
    mismatches=mismatches,orders=orders,warmups=1,repeats=2,
    timing={size:stats(values) for size,values in timings.items()})
summary['passed']=not mismatches and summary['input_unchanged']
(args.output/'report.json').write_text(json.dumps(summary,indent=2)+'\n')
(args.output/'before_depth_support.py').write_text(source)
print(json.dumps(summary))
raise SystemExit(0 if summary['passed'] else 1)
