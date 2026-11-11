"""후보의 각도 범위와 실제 미관측 질의를 별도로 대조한다."""
import json
import numpy as np
from pathlib import Path
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.known_structure import classify
A=Path(__file__).resolve().parent
events=[json.loads(l) for l in (A.parent/'parallel_rebase_r133_v1/two_actor_run1/run1/follow_decisions.jsonl').open()]
owner,motion=make_controller(events[0]['data'])
row=next(r for r in events if r['seq']==790)
data,scan,history=restore_call(row['data']['input'],motion)
policy=owner.normal.partial_policy
valid,excused,rows=classify(scan,policy.structure_sectors)
shadows=policy._shadow(scan,rows,history,data['reference_s'],scan.acquired_at_s[0])
g=np.load(A/'geometry_0.npz');q=g['queries'][~g['free']&~g['masked']]
out=[]
for s,row in zip(shadows,rows):
 d=q-s['origin'];b=np.arctan2(d[:,1],d[:,0]);o=(b-s['low']+np.pi)%(2*np.pi)-np.pi
 inside=(o>=0)&(o<=s['high']-s['low'])
 out.append(dict(center=s['center_deg'],explained=s['explained'],low_deg=np.degrees(s['low']),high_deg=np.degrees(s['high']),radius=s['hidden_width_range_m'],covered=int(inside.sum()),q_bearing_deg=[float(np.degrees(b).min()),float(np.degrees(b).max())],flanks=row['flanks'],ranges=[scan.ranges[i] for i in row['flanks']],rays=row['rays'].tolist()))
(A/'geometry_report.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out))
