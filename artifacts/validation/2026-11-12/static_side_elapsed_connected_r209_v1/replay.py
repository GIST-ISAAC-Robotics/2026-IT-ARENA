"""공통 구현 명령이 r208의 저장 비교 후보와 동일한지 확인한다."""
from pathlib import Path
import hashlib,importlib.util,json
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
A=Path(__file__).resolve().parent
P=A.parent
spec=importlib.util.spec_from_file_location('r209_route',P/'expiry_route_reference_r186_v1/candidate_v2.py')
route=importlib.util.module_from_spec(spec);spec.loader.exec_module(route);route.install()
results=[]
for tag,trace,expected_path in (
    ('r203',P/'released_mount_runtime_r203_v1/two_actor_run1/run1/follow_decisions.jsonl',P/'static_side_elapsed_r208_v1/comparison.json'),
    ('r206',P/'delayed_restart_r206_v1/moving_stop_restart/run1/follow_decisions.jsonl',P/'static_side_elapsed_r208_v1/restart_comparison.json')):
    expected=json.loads(expected_path.read_bytes())
    assert hashlib.sha256(trace.read_bytes()).hexdigest()==expected['summary']['trace_sha256']
    rows={r['seq']:r for r in expected['rows']}
    events=[json.loads(line) for line in trace.open()]
    controller,mode=make_controller(events[0]['data'])
    count=0;errors=[]
    for e in events[1:]:
        d=e['data']
        if e['kind']=='reset':
            if d.get('reason')=='decision_source_expired':route.reset_reference(controller)
            else:controller.reset()
        if e['kind']!='call':continue
        result=freeze(replay_call(controller,d['input'],mode));count+=1
        row=rows.get(e['seq']);command=row['after'] if row else d['output'][:2]
        if not equal_output(result[:2],command):errors.append(dict(seq=e['seq'],actual=result[:2],expected=command))
        if row:
            normal=result[2].get('normal',{})
            if (normal.get('selector_mode'),normal.get('selector_reason'))!=(row['after_mode'],row['after_reason']):
                errors.append(dict(seq=e['seq'],error='selector mismatch'))
    results.append(dict(trace=tag,calls=count,errors=errors))
with (A/'replay.json').open('x') as f:json.dump(results,f,indent=2);f.write('\n')
print(json.dumps(results));assert not any(r['errors'] for r in results)
