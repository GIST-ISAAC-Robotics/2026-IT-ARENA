"""일시 경로 실패 뒤 GUARD 진입 차단을 저장 입력에서 계측한다. 출력을 변경하지 않는다."""
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path

import arena_autonomy.pass_release_trial as ps
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze,comparable_decision,equal_output

A=Path(__file__).resolve().parent
P=A.parent/'adjacent_live_r152_v1/candidate.py'
spec=importlib.util.spec_from_file_location('r155_old',P);old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
TRACE=A.parent/'adjacent_live_r152_v1/two_actor_run2/run1/follow_decisions.jsonl'


def main():
    digest=hashlib.sha256(TRACE.read_bytes()).hexdigest()
    events=[json.loads(line) for line in TRACE.open()]
    old.install();original=ps.PassReleaseSelector._guard_curve_at_cap
    rows=[];current={}
    def observe(owner,scene,view,baseline,parallel,speed,previous,dt,age,detail,raw_scan,*,target_cap=None):
        out=original(owner,scene,view,baseline,parallel,speed,previous,dt,age,detail,raw_scan,target_cap=target_cap)
        if out is None and parallel[2].get('reason')=='parallel_hold_release_wait':
            b=baseline[2];partial=b.get('partial_observation') or {}
            rows.append(dict(seq=current['seq'],reference_s=scene.t_ref_s,mode=owner.mode,
                last_choice=owner.last_choice,actual_speed=speed,baseline_speed=baseline[0],
                baseline_reason=b.get('reason'),fresh=b.get('boundary_from_memory') is False and b.get('path_source_s')==scene.source_first_s,
                partial_allowed=partial.get('allowed'),partial_reason=partial.get('reason'),
                traffic_recovery=owner.follow_config.traffic_recovery_trial,side_present=any(s['present'] for s in owner.sides.values()),
                before=current['before']))
        return out
    ps.PassReleaseSelector._guard_curve_at_cap=observe
    owner,motion=make_controller(events[0]['data']);calls=0;mismatch=[]
    try:
        for event in events[1:-1]:
            if event['kind']=='reset':owner.reset()
            elif event['kind']=='call':
                calls+=1;current.update(seq=event['seq'],before=event['data']['output'][:2])
                output=freeze(replay_call(owner,event['data']['input'],motion))
                if not equal_output(comparable_decision(output),comparable_decision(event['data']['output'])):mismatch.append(event['seq'])
    finally:ps.PassReleaseSelector._guard_curve_at_cap=original
    eligible=[r for r in rows if r['mode']=='GUARD' and r['traffic_recovery'] and r['fresh']
        and r['partial_allowed'] is True and r['partial_reason']=='fully_observed' and not r['side_present']
        and 0<=r['actual_speed']<=.1 and r['baseline_speed']>0]
    result=dict(calls=calls,mismatches=mismatch,not_entered=len(rows),fresh_stopped_without_current_side=len(eligible),
        eligible_sequences=[r['seq'] for r in eligible],previous_choices=dict(Counter(r['last_choice'] for r in rows)),
        first_eligible=eligible[0] if eligible else None,trace_sha256=digest,
        trace_unchanged=hashlib.sha256(TRACE.read_bytes()).hexdigest()==digest,
        scope='Unchanged saved replay; a reentry candidate would still need the full motion/occupancy/guard checks.')
    with (A/'diagnosis.json').open('x') as f:json.dump(result,f,indent=2)
    with (A/'details.jsonl').open('x') as f:
        for row in rows:f.write(json.dumps(row)+'\n')
    assert not mismatch and result['trace_unchanged']
    print(json.dumps({k:v for k,v in result.items() if k!='eligible_sequences'}))


if __name__=='__main__':main()
