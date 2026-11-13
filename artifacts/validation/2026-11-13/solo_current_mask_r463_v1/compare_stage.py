"""실제 current 3개의 원 결과 재현 및 각 motion_region 자기 가림 치수 계측."""
import hashlib,json,os,sys
from pathlib import Path
A=Path(__file__).resolve().parent
sys.path[:0]=[str(A/'stage/src/arena_autonomy'),str(A/'stage/src/arena_vehicle_interface')]
import numpy as np
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.follow_trace import freeze
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((A/'manifest.json').read_bytes())
assert all(sha(A/p)==h for p,h in manifest['files'].items())
data=json.loads((A/'inputs.json').read_bytes())
out=A/'stage_analysis_v1';out.mkdir(exist_ok=False)
original=ps.motion_region
active=[]
def observed(scene,speed,target,steering,previous,dt,cfg,lat,*,context=None):
    context={} if context is None else context
    result=original(scene,speed,target,steering,previous,dt,cfg,lat,context=context)
    q=np.asarray(context.get('queries',[])).reshape(-1,2)
    masked=np.asarray(context.get('masked',np.zeros(len(q),dtype=bool)))
    points=q[masked]
    bounds=None if not len(points) else dict(minimum=points.min(axis=0).tolist(),maximum=points.max(axis=0).tolist(),span=np.ptp(points,axis=0).tolist())
    active.append(dict(speed=speed,target=target,steering=steering,previous=previous,dt=dt,region=result,bounds=bounds,
        body_length_m=cfg.length_m,body_width_m=cfg.width_m,masked_queries=points.tolist(),padding=context.get('padding')))
    return result
ps.motion_region=observed
reports=[]
evaluator=Evaluator('current',data['header'])
try:
    for row in data['calls']:
        active=[];d=row['data'];actual=d['current_rebase'];before=json.dumps(row,sort_keys=True)
        value=evaluator(dict(output=d['output'],submitted=d['compute']['input'],current=actual['input']))
        assert before==json.dumps(row,sort_keys=True)
        reports.append(dict(sequence=row['seq'],exact=value==actual['output'],actual=value,regions=freeze(active)))
finally:evaluator.close();ps.motion_region=original
(out/'results.json').write_text(json.dumps(reports,indent=2),encoding='utf8')
summary=[dict(sequence=r['sequence'],exact=r['exact'],speed=r['actual']['speed_mps'],reason=r['actual']['reason'],
    regions=[{k:v for k,v in x.items() if k!='masked_queries'} for x in r['regions']]) for r in reports]
(out/'summary.json').write_text(json.dumps(dict(pid=os.getpid(),calls=summary,inputs_unchanged=all(sha(A/p)==h for p,h in manifest['files'].items())),indent=2),encoding='utf8')
print(json.dumps([dict(sequence=r['sequence'],speed=r['actual']['speed_mps'],reason=r['actual']['reason'],structure=(r['actual'].get('current_route_proof') or {}).get('current_structure_risk'),regions=len(r['regions'])) for r in reports]))
