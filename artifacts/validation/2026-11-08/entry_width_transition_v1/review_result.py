"""실행 후 독립 확인. 계획 메타데이터와 실제 양수 명령을 구분한다."""
from pathlib import Path
import hashlib
import json
import math

A = Path(__file__).resolve().parent
report = json.loads((A/'run1/report.json').read_bytes())
assessment = json.loads((A/'assessment.json').read_bytes())
start = next(r for r in report['steps'] if r['step']=='before_resume')['truth']
poses = [json.loads(line) for line in (A/'run1/ego_poses.jsonl').open()]
poses = [r for r in poses if r[0] >= start[0]]
projections = [(r[1]-start[1])*math.cos(start[3])+(r[2]-start[2])*math.sin(start[3]) for r in poses]
first = positive = None
metadata_rows = selected_positive_rows = 0
for line in (A/'run1/follow_decisions.jsonl').open():
    event = json.loads(line)
    if event['kind'] != 'call': continue
    d=event['data']; b=d['output'][2].get('bypass', {}); plan=b.get('corridor_plan') or {}
    comparison=plan.get('adaptive_corridor_offset', {})
    if not comparison.get('applied'): continue
    metadata_rows += 1
    record=dict(seq=event['seq'], t=d['input']['reference_s'], output=d['output'][:2],
        selected=b.get('selected'), comparison=comparison)
    if first is None: first=record
    selected=b.get('selected') or {}
    if d['output'][0]>0. and selected.get('offset_m')==comparison['offset_m']:
        selected_positive_rows += 1
        if positive is None: positive=record
assert positive is not None and positive['selected']['offset_m']==-.2875
assert assessment['passed'] and report['completed'] and not report.get('error')
out=dict(scope='post-run truth assessment only; no truth supplied to driving controller',
    reference=start, samples=len(poses), max_projected_progress_m=max(projections),
    final_projected_progress_m=projections[-1],
    path_length_m=sum(math.hypot(b[1]-a[1],b[2]-a[2]) for a,b in zip(poses,poses[1:])),
    first_plan=first, first_positive_selected=positive,
    repeated_plan_metadata_rows=metadata_rows, positive_selected_metadata_rows=selected_positive_rows,
    raw_passed_field=report['passed'], assessment_passed=assessment['passed'],
    raw_flag_note='race_ops_eval initializes passed=False and writes completed separately; dedicated pre-registered OPS assessment is authoritative; raw report unchanged',
    note='repeated plan rows are not repeated planning; adaptive short-preview count is distinct from offset selection',
    raw_report_sha256=hashlib.sha256((A/'run1/report.json').read_bytes()).hexdigest())
(A/'review_result.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in out.items() if k not in ('first_plan','first_positive_selected')},ensure_ascii=False))
