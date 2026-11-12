"""현재 소스의 저장 입력 CPU 측정. ROS·직렬 포트·구동기 접근 없음."""
from pathlib import Path
import hashlib,importlib.util,json,platform,statistics,time,sys
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
A=Path(__file__).resolve().parent;P=A.parent
trace=P/'released_mount_runtime_r203_v1/two_actor_run1/run1/follow_decisions.jsonl'
expected=json.loads((P/'static_side_elapsed_r208_v1/comparison.json').read_bytes())
digest=hashlib.sha256(trace.read_bytes()).hexdigest();assert digest==expected['summary']['trace_sha256']
spec=importlib.util.spec_from_file_location('r214_route',P/'expiry_route_reference_r186_v1/candidate_v2.py')
route=importlib.util.module_from_spec(spec);spec.loader.exec_module(route);route.install()
events=[json.loads(s) for s in trace.open()];rows={r['seq']:r for r in expected['rows']}
source_paths=sorted((A.parents[3]/'src').rglob('*.py'))
sources={str(p.relative_to(A.parents[3])):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths}
baseline={};errors=[];times=[];metadata_errors=[]
for repeat in range(2):
    controller,mode=make_controller(events[0]['data'])
    for event in events[1:]:
        d=event['data'];seq=event['seq']
        if event['kind']=='reset':
            if d.get('reason')=='decision_source_expired':route.reset_reference(controller)
            else:controller.reset()
        if event['kind']!='call':continue
        began=time.perf_counter();out=replay_call(controller,d['input'],mode);elapsed=time.perf_counter()-began
        out=freeze(out);normal=out[2].get('normal',{})
        if repeat==0:
            baseline[seq]=out
            command=rows[seq]['after'] if seq in rows else d['output'][:2]
            if not equal_output(out[:2],command):errors.append(seq)
        else:
            if not equal_output(comparable_decision(out),comparable_decision(baseline[seq])):metadata_errors.append(seq)
            times.append(dict(seq=seq,elapsed_ms=elapsed*1000,speed_mps=out[0],mode=normal.get('selector_mode'),reason=normal.get('selector_reason')))
def stats(values):
    v=sorted(values)
    return dict(count=len(v),p50=statistics.median(v),p95=v[min(len(v)-1,int(.95*len(v)))],max=max(v)) if v else dict(count=0)
after={str(p.relative_to(A.parents[3])):hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths}
result=dict(input_unchanged=hashlib.sha256(trace.read_bytes()).hexdigest()==digest,source_unchanged=sources==after,
    input_sha256=digest,source_sha256=sources,platform=dict(machine=platform.machine(),python=platform.python_version()),
    calls=len(times),pc_command_mismatches=errors,same_platform_output_mismatches=metadata_errors,
    all_ms=stats([r['elapsed_ms'] for r in times]),positive_ms=stats([r['elapsed_ms'] for r in times if r['speed_mps']>0]),
    rows=times,scope='preloaded recorded input with one warmup and one measured pass; no ROS/sensor/device/actuation, not real time proof')
result['passed']=result['input_unchanged'] and result['source_unchanged'] and not errors and not metadata_errors
with (A/'report.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
print(json.dumps({k:v for k,v in result.items() if k not in ('rows','source_sha256')}));raise SystemExit(0 if result['passed'] else 1)
