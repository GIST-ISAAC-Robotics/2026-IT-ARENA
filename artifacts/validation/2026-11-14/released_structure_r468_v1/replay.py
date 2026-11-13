"""r465 고유 plan 제출 순서 재생. 새 주행·센서·구동 출력은 없다."""
from pathlib import Path
import hashlib,json,sys,time

A=Path(__file__).resolve().parent
mode=sys.argv[1]
assert mode in ('base','stage')
sys.path[:0]=[str(A/mode/'src/arena_autonomy'),str(A/mode/'src/arena_vehicle_interface')]
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.runtime_dual_state import current_request

out=A/(mode+'_replay_v1');out.mkdir(exist_ok=False)
trace=A.parent.parent/'2026-11-13/solo_full_lap_r465_v1/run1/follow_decisions.jsonl'
header=None; submitted=[]; completed={}
for line in trace.open(encoding='utf-8'):
    r=json.loads(line);d=r['data']
    if r['kind']=='runtime_evaluator_header':header=d['header']
    elif r['kind']=='runtime_plan_submitted':submitted.append(d)
    elif r['kind']=='runtime_plan_completed':completed[d['generation'],d['sequence']]=d
assert header and len(submitted)==len(completed)==207
source_before={p.relative_to(A/mode).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
    for p in (A/mode).rglob('*') if p.is_file()}
e=Evaluator('plan',header);cur=Evaluator('current',header) if mode=='stage' else None
generation=None; rows=[]; samples=[]
try:
    with (out/'outputs.jsonl').open('x',encoding='utf-8') as stream:
        for item in submitted:
            key=(item['generation'],item['sequence'])
            if generation!=key[0]:e.reset(item['reset_reason']);generation=key[0]
            encoded=json.dumps(item['input'],sort_keys=True)
            started=time.perf_counter();actual=e(item['input']);ms=(time.perf_counter()-started)*1000
            assert encoded==json.dumps(item['input'],sort_keys=True)
            expected=completed[key]['output'];n=actual[2].get('normal',{});old=expected[2].get('normal',{})
            row=dict(key=key,reference_s=item['input']['reference_s'],ms=ms,exact=actual==expected,
                old_command=expected[:2],command=actual[:2],old_reason=old.get('reason'),reason=n.get('reason'),
                selector_reason=n.get('selector_reason'),mode=n.get('selector_mode'),
                path_same=n.get('path_points')==old.get('path_points'),
                sides_same=n.get('selector_sides')==old.get('selector_sides'),
                state_same=actual[2].get('handoff_state')==expected[2].get('handoff_state'),
                structure=n.get('selector_structure_risk'))
            rows.append(row)
            stream.write(json.dumps(dict(key=key,output=actual),allow_nan=False)+'\n')
            if expected[0]==0<actual[0] and len(samples)<3:
                request=current_request(actual,item['input'],item['input'],retained=False,crop_current_selection=True)
                checked=cur(request)
                samples.append(dict(key=key,input=item['input'],plan=actual,current=checked,
                    scope='same_recorded_input_plan_current_not_new_publication'))
finally:
    if cur:cur.close()
    e.close()
after={p.relative_to(A/mode).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
    for p in (A/mode).rglob('*') if p.is_file()}
summary=dict(mode=mode,plans=len(rows),exact=sum(r['exact'] for r in rows),
    zero_to_positive=sum(r['old_command'][0]==0<r['command'][0] for r in rows),
    positive_to_zero=sum(r['old_command'][0]>0 and r['command'][0]<=0 for r in rows),
    positive_decreased=sum(r['old_command'][0]>0 and r['command'][0]<r['old_command'][0] for r in rows),
    changed_sides=sum(not r['sides_same'] for r in rows),changed_state=sum(not r['state_same'] for r in rows),
    sources_unchanged=source_before==after,trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
    current_examples=[dict(key=r['key'],plan_speed=r['plan'][0],speed=r['current']['speed_mps'],
        reason=r['current']['reason'],proof=r['current'].get('current_route_proof')) for r in samples])
(out/'rows.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
(out/'samples.json').write_text(json.dumps(samples,indent=2),encoding='utf-8')
(out/'source_manifest.json').write_text(json.dumps(source_before,indent=2),encoding='utf-8')
(out/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in summary.items() if k!='current_examples'}),flush=True)
assert summary['sources_unchanged']
if mode=='base':assert summary['exact']==207
else:assert not summary['positive_to_zero'] and not summary['positive_decreased']
