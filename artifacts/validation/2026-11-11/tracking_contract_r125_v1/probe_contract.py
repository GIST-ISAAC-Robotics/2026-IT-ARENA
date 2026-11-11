import json
from pathlib import Path
import sys
from path_contract import proposal,current_candidate

A=Path(__file__).resolve().parent
sys.path.insert(0,str(A.parent/'current_tracking_r117_v1'))
import probe

if __name__=='__main__':
    probe.A=A
    probe.proposal=proposal
    probe.current_candidate=current_candidate
    probe.main()
    rows=[json.loads(line) for line in (A/'details.jsonl').open()]
    old=[json.loads(line) for line in (A.parent/'current_tracking_r117_v1/details.jsonl').open()]
    result=dict(calls=len(rows),old_eligible=sum(r['eligible'] for r in old),
        corrected_eligible=sum(r['eligible'] for r in rows),
        previously_eligible_now_rejected=[dict(index=i,reference_s=r['reference_s'],reason=r['reason'])
            for i,(o,r) in enumerate(zip(old,rows)) if o['eligible'] and not r['eligible']],
        scope='Original path age restored; no runtime or old evidence overwritten')
    with (A/'contract_comparison.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result))
