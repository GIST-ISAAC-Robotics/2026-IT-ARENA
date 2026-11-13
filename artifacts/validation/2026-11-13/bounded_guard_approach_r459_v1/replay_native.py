"""고정 r452의 실제 고유 제출을 stage로 순차 재생한다. 새 주행/정답 입력 없음."""
import collections
import hashlib
import json
import os
from pathlib import Path
import sys

B=Path(__file__).resolve().parent
S=B/'stage'
sys.path[:0]=[str(S/'src/arena_autonomy'),str(S/'src/arena_vehicle_interface')]
from arena_autonomy.race_runtime_policy.evaluator import Evaluator,supported

O=B/'native_v1';O.mkdir(exist_ok=False)
def write(n,v):(O/n).write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((B/'stage_manifest.json').read_text())
assert all(sha(S/n)==h for n,h in manifest.items())
trace=B.parent/'guard_crawl_drive_r452_v1/run1/follow_decisions.jsonl'
baseline=B.parent/'guard_crawl_entry_r456_v1/pinpoint_v1'
original=json.loads((baseline/'summary.json').read_text())
assert original['replayed']==original['exact']==381 and original['mismatch'] is None
frozen=json.loads((B.parent/'guard_crawl_entry_r456_v1/source_manifest.json').read_text())
assert sha(trace)==next(h for p,h in frozen.items() if p.replace('\\','/').endswith('run1/follow_decisions.jsonl'))
rows=[json.loads(line) for line in trace.open()]
header=next(e['data']['header'] for e in rows if e['kind']=='runtime_evaluator_header')
done={(e['data']['generation'],e['data']['sequence']):e['data'] for e in rows if e['kind']=='runtime_plan_completed'}
submitted=[e['data'] for e in rows if e['kind']=='runtime_plan_submitted']
original_inputs={(e['generation'],e['sequence']):e['input'] for e in submitted}
outputs={};checks=[];evaluator=None;generation=None
try:
    evaluator=Evaluator('plan',header)
    for item in submitted:
        key=(item['generation'],item['sequence'])
        if generation!=key[0]:evaluator.reset(item['reset_reason']);generation=key[0]
        before=json.dumps(item['input'],sort_keys=True)
        output=evaluator(item['input']);expected=done[key]['output']
        assert before==json.dumps(item['input'],sort_keys=True)
        outputs[key]=output
        normal=output[2].get('normal',{});old=expected[2].get('normal',{})
        proof=normal.get('selector_guard_candidate',{}).get('current_approach',{})
        checks.append(dict(generation=key[0],sequence=key[1],reference_s=item['input']['reference_s'],measured=item['input']['speed_mps'],
            old_command=expected[:2],new_command=output[:2],exact=output==expected,command_same=output[:2]==expected[:2],
            old_reason=old.get('reason'),new_reason=normal.get('reason'),
            old_selector_reason=old.get('selector_reason'),new_selector_reason=normal.get('selector_reason'),
            path_same=normal.get('path_points')==old.get('path_points'),
            side_state_same=normal.get('selector_sides')==old.get('selector_sides'),
            current_approach=proof,output=output))
finally:
    if evaluator is not None:evaluator.close()
write('plans.json',checks)

# Only actual recorded current inputs, with the actual originating plan key.
# Stalled original zero plans had no current submission; do not fabricate one.
recorded_current=[e for e in rows if e['kind']=='call' and e['data'].get('current_rebase')]
changed={ (c['generation'],c['sequence']) for c in checks if not c['command_same']}
preferred=[e for e in recorded_current if (e['data']['compute']['generation'],e['data']['compute']['sequence']) in changed]
remaining=[e for e in recorded_current if e not in preferred and e['data']['output'][2].get('normal',{}).get('selector_mode')=='GUARD']
candidates=[]
for e in (preferred[:3]+preferred[-3:]+remaining[:2]):
    if e['seq'] not in {r['seq'] for r in candidates}:candidates.append(e)
current_checks=[];evaluator=None
try:
    evaluator=Evaluator('current',header)
    for event in candidates:
        data=event['data'];key=(data['compute']['generation'],data['compute']['sequence']);output=outputs[key]
        current=data['current_rebase']['input']
        request=dict(output=output,submitted=original_inputs[key],current=current)
        before=json.dumps(request,sort_keys=True)
        value=evaluator(request) if supported(output) else None
        assert before==json.dumps(request,sort_keys=True)
        current_checks.append(dict(trace_seq=event['seq'],plan_key=list(key),plan_command=output[:2],
            plan_command_changed=key in changed,source_reference_s=original_inputs[key]['reference_s'],
            current_reference_s=current['reference_s'],input_is_actual_recorded_current=True,
            original_output=data['current_rebase']['output'],stage_output=value))
finally:
    if evaluator is not None:evaluator.close()
write('actual_current_pairs.json',current_checks)

def group(values):
    return dict(count=len(values),exact=sum(c['exact'] for c in values),command_same=sum(c['command_same'] for c in values),
        old_positive=sum(c['old_command'][0]>0 for c in values),new_positive=sum(c['new_command'][0]>0 for c in values),
        zeros_to_positive=sum(c['old_command'][0]==0 and c['new_command'][0]>0 for c in values),
        positive_to_zero=sum(c['old_command'][0]>0 and c['new_command'][0]==0 for c in values),
        new_speeds=dict(collections.Counter(c['new_command'][0] for c in values)),
        reason_transitions=dict(collections.Counter(str(c['old_selector_reason'])+' => '+str(c['new_selector_reason']) for c in values if not c['exact'])),
        path_changed=sum(not c['path_same'] for c in values),side_state_changed=sum(not c['side_state_same'] for c in values))
stall=[c for c in checks if c['reference_s']>=34.999]
other=[c for c in checks if c['reference_s']<34.999]
modules={n:str(Path(v.__file__).resolve()) for n,v in sys.modules.items()
    if n.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(v,'__file__',None)}
outside={n:p for n,p in modules.items() if not Path(p).is_relative_to(S)}
changed_files=[n for n,h in manifest.items() if sha(S/n)!=h]
summary=dict(pid=os.getpid(),backend='native_v1',actual_header_unmodified=True,original_381_exact_evidence=str(baseline/'summary.json'),
    trace_sha256=sha(trace),plan_count=len(checks),whole=group(checks),original_stall_108=group(stall),other_273=group(other),
    actual_current_pairs=len(current_checks),actual_current_positive=sum(c['stage_output'] is not None and c['stage_output']['speed_mps']>0 for c in current_checks),
    original_stall_current_input_absent=True,stalled_current_values_synthesized=False,
    stage_source_changes=changed_files,external_project_imports=outside,modules=modules,
    new_drive=False,common_source_changes=False,controller_truth_used=False)
write('summary.json',summary)
print(json.dumps({k:v for k,v in summary.items() if k!='modules'},ensure_ascii=False))
assert len(checks)==381 and len(stall)==108 and len(other)==273 and not changed_files and not outside
