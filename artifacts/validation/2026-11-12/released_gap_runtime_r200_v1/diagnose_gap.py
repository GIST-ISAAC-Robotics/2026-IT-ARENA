"""실패한 새 궤적에서 기존 제한이 적용되지 않은 이유만 관찰한다."""
from collections import Counter
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from arena_autonomy import released_body_gap as gap
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision

A=Path(__file__).resolve().parent
trace=A/'two_actor_run1/run1/follow_decisions.jsonl'
spec=importlib.util.spec_from_file_location('r200_diag_route',A.parent/'expiry_route_reference_r186_v1/candidate_v2.py')
route=importlib.util.module_from_spec(spec);spec.loader.exec_module(route);route.install()
events=[json.loads(line) for line in trace.open()]
owner,motion=make_controller(events[0]['data'])
original_propose,original_check=gap.propose,gap.check
rows=[];active=None;seq=None
def check(*args,**kwargs):
    result=original_check(*args,**kwargs)
    if active is not None:active['checks'].append(result)
    return result
def propose(owner,scene,scan,baseline,speed,previous,dt,limit):
    global active
    normal=baseline[2]
    active=dict(seq=seq,reference_s=scene.t_ref_s,speed=speed,baseline=baseline[:2],limit=limit,
        mode=owner.mode,side_present={name:side['present'] for name,side in owner.sides.items()},
        baseline_reason=normal.get('reason'),boundary_from_memory=normal.get('boundary_from_memory'),
        same_source=normal.get('path_source_s')==scene.source_first_s,partial=normal.get('partial_observation'),checks=[])
    result=original_propose(owner,scene,scan,baseline,speed,previous,dt,limit)
    active['accepted']=result is not None
    rows.append(active);active=None
    return result
gap.propose, gap.check=propose,check
mismatch=[];calls=0
for event in events[1:]:
    data=event['data'];seq=event['seq']
    if event['kind']=='reset':
        if data.get('reason')=='decision_source_expired':route.reset_reference(owner)
        else:owner.reset()
    if event['kind']!='call':continue
    result=freeze(replay_call(owner,data['input'],motion));calls+=1
    if not equal_output(comparable_decision(result),comparable_decision(data['output'])):mismatch.append(seq)
summary=dict(calls=calls,mismatches=mismatch,proposals=len(rows),accepted=sum(r['accepted'] for r in rows),
    no_checks=sum(not r['checks'] for r in rows),check_reasons=dict(Counter((c.get('proof') or {}).get('reason',c.get('reason','none')) for r in rows for c in r['checks'])),
    raw_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),rows=rows)
with (A/'gap_diagnosis.json').open('x') as f:json.dump(summary,f,indent=2);f.write('\n')
print(json.dumps({k:v for k,v in summary.items() if k!='rows'}))
assert not mismatch
