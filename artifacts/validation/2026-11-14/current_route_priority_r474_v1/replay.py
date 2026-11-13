"""r469 원 plan 순서와 실제 current를 r474 동결 source로 비교한다."""
from pathlib import Path
import hashlib,json,sys,time
A=Path(__file__).resolve().parent;S=A/'stage'
sys.path[:0]=[str(S/'src/arena_autonomy'),str(S/'src/arena_vehicle_interface')]
from arena_autonomy.race_runtime_policy.evaluator import Evaluator,supported
from arena_autonomy.runtime_dual_state import current_request
TRACE=A.parent/'solo_full_lap_r469_v1/run1/follow_decisions.jsonl'
OUT=A/(sys.argv[1] if len(sys.argv)>1 else 'replay_v1');OUT.mkdir(exist_ok=False)
before=hashlib.sha256(TRACE.read_bytes()).hexdigest()
manifest=json.loads((A/'source_manifest.json').read_text(encoding='utf-8'))['files']
assert all(hashlib.sha256((S/p).read_bytes()).hexdigest()==h for p,h in manifest.items())
records=[json.loads(l) for l in TRACE.open(encoding='utf-8')]
header=next(r['data']['header'] for r in records if r['kind']=='runtime_evaluator_header')
assert not any(header.get(k) for k in ('vehicle_marker_enabled','vehicle_marker_trial'))
submitted=[r['data'] for r in records if r['kind']=='runtime_plan_submitted']
completed={(r['data']['generation'],r['data']['sequence']):r['data'] for r in records if r['kind']=='runtime_plan_completed'}
assert len(submitted)==len(completed)==217
e=Evaluator('plan',header);generation=None;outputs={};plan_rows=[]
try:
    with (OUT/'outputs.jsonl').open('x',encoding='utf-8') as stream:
        for item in submitted:
            key=item['generation'],item['sequence']
            if generation!=key[0]:e.reset(item['reset_reason']);generation=key[0]
            encoded=json.dumps(item['input'],sort_keys=True)
            start=time.perf_counter();actual=e(item['input']);ms=(time.perf_counter()-start)*1000
            assert encoded==json.dumps(item['input'],sort_keys=True)
            expected=completed[key]['output'];outputs[key]=actual
            normal=actual[2].get('normal') or {};old=expected[2].get('normal') or {}
            plan_rows.append(dict(key=key,reference_s=item['input']['reference_s'],ms=ms,exact=actual==expected,
                old_command=expected[:2],command=actual[:2],old_reason=old.get('reason'),reason=normal.get('reason'),
                old_selector_reason=old.get('selector_reason'),selector_reason=normal.get('selector_reason'),
                old_mode=old.get('selector_mode'),mode=normal.get('selector_mode'),
                sides_same=old.get('selector_sides')==normal.get('selector_sides'),
                handoff_same=expected[2].get('handoff_state')==actual[2].get('handoff_state')))
            stream.write(json.dumps(dict(key=key,output=actual),allow_nan=False)+'\n')
finally:e.close()
cur=Evaluator('current',header);current_rows=[];legacy_current=[];same_input=[]
try:
    for record in records:
        if record['kind']!='call' or not record['data'].get('current_rebase'):continue
        c=record['data']['current_rebase'];meta=c['compute'];oldplan=meta['plan'];key=oldplan['generation'],oldplan['sequence']
        for mode,plan in [('original_plan',oldplan['output']),('new_plan_actual_current',outputs[key])]:
            target=legacy_current if mode=='original_plan' else current_rows
            if not supported(plan):
                target.append(dict(trace_seq=record['seq'],key=key,unsupported=True,plan_speed=plan[0]));continue
            request=current_request(plan,oldplan['input'],meta['input'],retained=meta['route_check']['retained'],crop_current_selection=True)
            encoded=json.dumps(request,sort_keys=True);actual=cur(request);assert encoded==json.dumps(request,sort_keys=True)
            expected=c['output']
            target.append(dict(trace_seq=record['seq'],key=key,reference_s=meta['input']['reference_s'],
                measured_speed_mps=meta['input']['speed_mps'],plan_speed=plan[0],exact=actual==expected,
                old_speed=expected['speed_mps'],speed=actual['speed_mps'],old_reason=expected['reason'],reason=actual['reason'],
                policy=actual.get('current_observation_policy'),output=actual))
    stalled=[x for x in submitted if x['input']['reference_s']>=19.22]
    for item in (stalled[0],stalled[len(stalled)//2],stalled[-1]):
        key=item['generation'],item['sequence'];plan=outputs[key]
        if supported(plan):
            request=current_request(plan,item['input'],item['input'],retained=False,crop_current_selection=True)
            same_input.append(dict(key=key,reference_s=item['input']['reference_s'],plan_speed=plan[0],
                current=cur(request),scope='same_recorded_plan_input; no subsequent actual current or publication exists'))
finally:cur.close()
summary=dict(plans=217,exact=sum(r['exact'] for r in plan_rows),
    zero_to_positive=sum(r['old_command'][0]==0<r['command'][0] for r in plan_rows),
    positive_to_zero=sum(r['old_command'][0]>0 and r['command'][0]<=0 for r in plan_rows),
    positive_increased=sum(0<r['old_command'][0]<r['command'][0] for r in plan_rows),
    positive_decreased=sum(r['old_command'][0]>0 and r['command'][0]<r['old_command'][0] for r in plan_rows),
    sides_changed=sum(not r['sides_same'] for r in plan_rows),handoff_changed=sum(not r['handoff_same'] for r in plan_rows),
    current_original_plan_calls=len(legacy_current),current_original_plan_exact=sum(r.get('exact',False) for r in legacy_current),
    new_plan_actual_current_calls=len(current_rows),
    current_zero_to_positive=sum(r.get('old_speed')==0 and r.get('speed',0)>0 for r in current_rows),
    current_positive_to_zero=sum(r.get('old_speed',0)>0 and r.get('speed',0)<=0 for r in current_rows),
    first_stop=next(r for r in current_rows if r['trace_seq']==823),
    same_input_examples=[dict(reference_s=r['reference_s'],plan_speed=r['plan_speed'],speed=r['current']['speed_mps'],reason=r['current']['reason']) for r in same_input],
    source_unchanged=all(hashlib.sha256((S/p).read_bytes()).hexdigest()==h for p,h in manifest.items()),
    trace_unchanged=before==hashlib.sha256(TRACE.read_bytes()).hexdigest(),
    marker_default_off=True,scope='saved simulation sensor numbers; no actual new publication or lap result')
for name,value in [('plan_rows',plan_rows),('legacy_current_rows',legacy_current),('new_plan_actual_current_rows',current_rows),('same_input_current',same_input),('summary',summary)]:
    (OUT/(name+'.json')).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in summary.items() if k!='first_stop'}),flush=True)
assert summary['source_unchanged'] and summary['trace_unchanged']
