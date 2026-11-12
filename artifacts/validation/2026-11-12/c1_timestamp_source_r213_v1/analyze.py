"""저장 원시 입력에서 명목 간격과 이전 배치 기반 간격의 모순만 비교한다."""
from pathlib import Path
import hashlib,json,statistics,urllib.request
A=Path(__file__).resolve().parent;R=A.parents[3]
files=list((R/'build/r204_recovered').rglob('sdk_raw.jsonl'));assert len(files)==1
path=files[0];digest=hashlib.sha256(path.read_bytes()).hexdigest()
rows=[json.loads(s) for s in path.open()]
periods=[];records=[];previous_last={}
for i,row in enumerate(rows):
    first=row['sdk_estimated_first_us']*1000;n=len(row['nodes']);end=row['end_monotonic_ns']
    steps={'nominal_200us':200000.}
    if periods:
        selected=periods[-8:]
        steps['causal_previous_8_batches']=sum(a for a,b in selected)/sum(b for a,b in selected)
    for name,step in steps.items():
        last=first+(n-1)*step
        records.append(dict(scan=i,policy=name,step_ns=step,
            end_after_grab_ns=max(0,last-end),previous_overlap_ns=max(0,previous_last.get(name,first)-first)))
        previous_last[name]=last
    if i:
        prev=rows[i-1];interval=first-prev['sdk_estimated_first_us']*1000
        if 50_000_000<interval<200_000_000 and len(prev['nodes'])>=450:
            periods.append((interval,len(prev['nodes'])))
        else:periods=[]
def summary(values):
    values=sorted(values)
    return dict(min=min(values),median=statistics.median(values),p95=values[int((len(values)-1)*.95)],max=max(values))
result=dict(raw_sha256=digest,batches=len(rows),policies={},scope='host SDK anchor consistency, not acquisition timestamp accuracy or calibration')
for name in sorted({r['policy'] for r in records}):
    selected=[r for r in records if r['policy']==name]
    result['policies'][name]=dict(batches=len(selected),step_us=summary([r['step_ns']/1000 for r in selected]),
        after_grab_count=sum(r['end_after_grab_ns']>0 for r in selected),
        after_grab_ms=summary([r['end_after_grab_ns']/1e6 for r in selected]),
        overlap_count=sum(r['previous_overlap_ns']>0 for r in selected),
        overlap_ms=summary([r['previous_overlap_ns']/1e6 for r in selected]))
assert hashlib.sha256(path.read_bytes()).hexdigest()==digest
with (A/'analysis.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
print(json.dumps(result))
