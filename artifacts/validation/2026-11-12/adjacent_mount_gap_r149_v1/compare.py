"""기준 재생과 선택형 낮은 목표 검사를 분리한다. 새 궤적을 생성하지 않는다."""
from collections import Counter
import hashlib
import json
from pathlib import Path

import candidate
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, comparable_decision, freeze, equal_output

A=Path(__file__).resolve().parent
TRACE=A.parent/'guard_current_r143_v1/two_actor_run1/run1/follow_decisions.jsonl'


def main():
    digest=hashlib.sha256(TRACE.read_bytes()).hexdigest()
    events=[json.loads(line) for line in TRACE.open()]
    results={}
    for name,install in [('baseline',candidate.prior.prior.install),('candidate',candidate.install)]:
        install()
        owner,motion=make_controller(events[0]['data'])
        calls=0;mismatches=[];changed=[];attempted=accepted=0;reasons=Counter()
        zero_positive=positive_zero=positive_changed=0
        records=[]
        for row in events[1:-1]:
            if row['kind']=='reset':
                owner.reset()
            elif row['kind']=='call':
                calls+=1
                d=row['data'];before=d['output']
                after=freeze(replay_call(owner,d['input'],motion))
                if not equal_output(comparable_decision(before),comparable_decision(after)):
                    mismatches.append(row['seq'])
                n=after[2].get('normal') or {}
                check=n.get('selector_horizon_recheck') or {}
                if check:
                    attempted+=1;accepted+=check['applied']
                    for a in check['attempts']:reasons[a['reason']]+=1
                if not equal_output(before[:2],after[:2]):
                    changed.append(row['seq'])
                    zero_positive+=before[0]<=0<after[0]
                    positive_zero+=before[0]>0>=after[0]
                    positive_changed+=before[0]>0 and after[0]>0
                if check or not equal_output(before[:2],after[:2]):
                    records.append(dict(seq=row['seq'],reference_s=d['input']['reference_s'],
                        delivered=d.get('delivery',{}).get('kind'),before=before[:2],after=after[:2],
                        measured_speed=d['input']['speed_mps'],candidate=check,
                        selector_reason=n.get('selector_reason')))
        results[name]=dict(calls=calls,complete_output_mismatch_sequences=mismatches,
            changed_command_count=len(changed),zero_to_positive=zero_positive,
            positive_to_zero=positive_zero,positive_to_positive=positive_changed,
            attempted=attempted,accepted=accepted,attempt_reasons=dict(reasons))
        with (A/(name+'_details.jsonl')).open('x') as f:
            for r in records:f.write(json.dumps(r)+'\n')
        assert name!='baseline' or not mismatches
    results.update(trace_sha256=digest,trace_unchanged=hashlib.sha256(TRACE.read_bytes()).hexdigest()==digest,
        scope='Saved original sensor trajectory. Candidate commands do not change later sensors; not driving proof.')
    assert results['trace_unchanged']
    with (A/'comparison.json').open('x') as f:json.dump(results,f,indent=2)
    print(json.dumps(results))


if __name__=='__main__':
    main()
