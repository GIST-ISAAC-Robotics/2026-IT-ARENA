from pathlib import Path
import json,sys
A=Path(__file__).resolve().parent
ROOT=A.parents[3]
import arena_autonomy
if sys.argv[1]=='candidate':arena_autonomy.__path__.insert(0,str(A/'stage/arena_autonomy'))
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, freeze
path=ROOT/'artifacts/validation/2026-11-14/field_wall_drive_r520_v1/failure_inputs/controller_prefix.jsonl'
full=len(sys.argv)>2 and sys.argv[2]=='full'
if full:path=ROOT/'artifacts/validation/2026-11-14/field_wall_drive_r520_v1/run1/follow_decisions.jsonl'
rows=[json.loads(l) for l in path.read_text().splitlines()]
header=rows[0]['data'];owner,motion=make_controller(header);results=[]
for row in rows[1:]:
    if row['kind']=='reset':owner.reset();continue
    if row['kind']!='call':continue
    data=row['data'];out=replay_call(owner,data['input'],motion)
    normal=out[2].get('normal',out[2]);base=normal.get('baseline',normal)
    results.append(dict(seq=row['seq'],reference=data['input']['reference_s'],output=freeze(out),
       release_count=owner.normal.release_count,hold_reason=owner.normal.hold_reason,
       partial_previous=owner.normal.partial_policy.previous is not None))
(A/(sys.argv[1]+('_full' if full else '')+'.json')).write_text(json.dumps(results,ensure_ascii=False,indent=2))
print('calls',len(results),'positive',sum(r['output'][0]>0 for r in results))
for row in results[:8]:
    n=row['output'][2].get('normal',row['output'][2]);print(row['seq'],row['output'][0],n.get('reason'),n.get('baseline_reason'),row['release_count'],row['hold_reason'])
