"""Stateful saved comparison; output changes do not establish closed-loop progress."""
import collections
import hashlib
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[4]
sys.path[:0]=[str(ROOT/p) for p in ('src/arena_autonomy','src/arena_vehicle_interface','scripts')]
from arena_autonomy import follow_trace as ft
A=Path(__file__).resolve().parent
trace=ROOT/'artifacts/validation/2026-11-11/guard_masked_speed_r84_v1/two_actor_run1/run1/follow_decisions.jsonl'
output=A/'saved_comparison.json';assert not output.exists()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
before={str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
input_hash=sha(trace);original=ft.replay_call;current=None
changed=[];tail=[];added_calls=0;new_positive=0;new_zero=0
def observe(*args,**kwargs):
    global added_calls,new_positive,new_zero
    result=original(*args,**kwargs)
    old=current['data']['output'];seq=current['seq'];now=current['data']['input']['reference_s']
    if not ft.equal_output(ft.freeze(result[:2]),old[:2]):
        changed.append(dict(seq=seq,time=now,before=old[:2],after=ft.freeze(result[:2])))
        new_positive+=int(old[0]<=0<result[0]);new_zero+=int(result[0]<=0<old[0])
    normal=result[2].get('normal',result[2]);partial=normal.get('partial_observation') or {}
    structure=partial.get('known_structure') or {}
    added_calls+=int(structure.get('retained_previous_shadow_points',0)>0)
    if 60.2<=now<=61.:
        evidence=partial.get('latch_evidence') or {}
        tail.append(dict(time=now,speed=result[0],reason=partial.get('reason'),
            latch=partial.get('latch_reason'),points=(partial.get('anchor_diagnostic') or {}).get('threat_set',{}).get('unique_points'),
            added=structure.get('retained_previous_shadow_points',0),
            witnesses=evidence.get('witnesses',[]) if len(tail)==0 or partial.get('latch_time_s')==now else []))
    return result
def events():
    global current
    with trace.open(encoding='utf8') as stream:
        for line in stream:
            current=json.loads(line);yield current
ft.replay_call=observe
try:result=ft.replay_events(events())
finally:ft.replay_call=original
assert sha(trace)==input_hash and all(sha(ROOT/n)==v for n,v in before.items())
summary=dict(scope='saved stateful comparison only; no closed-loop driving',calls=result['calls'],
    full_outputs_matched=result['matched'],command_changes=changed,zero_to_positive=new_positive,
    positive_to_zero=new_zero,retained_transfer_calls=added_calls,late_latch=tail,
    source_unchanged=True,input_unchanged=True,input_sha256=input_hash)
output.write_text(json.dumps(summary,indent=2)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in summary.items() if k not in ('command_changes','late_latch')}))
