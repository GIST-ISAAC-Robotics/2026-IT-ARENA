"""실행 후 독립 검토. 원 평가를 수정하지 않고 실제 진행과 반복 메타데이터를 구분한다."""
from pathlib import Path
import json,math,hashlib
A=Path(__file__).resolve().parent
report=json.loads((A/'run1/report.json').read_bytes())
start=next(r for r in report['steps'] if r['step']=='before_resume')['truth']
poses=[json.loads(line) for line in (A/'run1/ego_poses.jsonl').open()]
poses=[r for r in poses if r[0]>=start[0]]
projections=[(r[1]-start[1])*math.cos(start[3])+(r[2]-start[2])*math.sin(start[3]) for r in poses]
first=None;metadata=0;tail=[]
for line in (A/'run1/follow_decisions.jsonl').open():
 e=json.loads(line)
 if e['kind']!='call':continue
 d=e['data'];b=d['output'][2].get('bypass',{});plan=b.get('corridor_plan') or {}
 c=plan.get('short_transition_comparison',{})
 if c.get('applied'):
  metadata+=1
  if first is None:first=dict(seq=e['seq'],t=d['input']['reference_s'],source=d['input']['scan']['sec']+d['input']['scan']['nanosec']*1e-9,plan=plan)
 if 34.91<d['input']['reference_s']<35.01:
  row=(b.get('candidates') or [{}])[0]
  tail.append(dict(t=d['input']['reference_s'],speed=d['output'][0],steer=d['output'][1],
   reason=row.get('reason'),clearance_m=row.get('command_clearance_m'),max_padding_m=row.get('maximum_occupancy_padding_m')))
out=dict(scope='post-run evaluation only; truth never enters controller',reference=start,samples=len(poses),
 max_projected_progress_m=max(projections),final_projected_progress_m=projections[-1],
 travelled_path_m=sum(math.hypot(b[1]-a[1],b[2]-a[2]) for a,b in zip(poses,poses[1:])),
 repeated_comparison_metadata_rows=metadata,first_comparison=first,tail=tail,
 note='permission_granted=false is the proposal interface contract, not a runtime refusal; repeated rows are not repeated planning',
 raw_report_sha256=hashlib.sha256((A/'run1/report.json').read_bytes()).hexdigest())
(A/'review_result.json').write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in out.items() if k not in ('first_comparison','tail')},ensure_ascii=False))
