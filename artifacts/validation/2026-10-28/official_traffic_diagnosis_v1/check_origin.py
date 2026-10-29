import json
from pathlib import Path
import sys
import numpy as np

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.local_follow import FollowConfig
from origin_candidate import projected_origin_offset, short_prefix_evidence

checks=[]
for phase in (-.015,-.010,-.005,0.,.005,.010,.015):
    p=np.array([[x+phase,.02] for x in np.arange(-.12,.121,.03)])
    index=int(np.argmin(np.linalg.norm(p,axis=1)))
    offset=projected_origin_offset(p,index)
    # 직선 y=.02에서 정확한 길이 원점은 x=0이다.
    assert abs(offset-p[index,0])<1e-12
    checks.append(dict(phase=phase,nearest_x=float(p[index,0]),offset=offset))
for p,index,expected in [(np.array([[.1,0],[.2,0]]),0,0.),(np.array([[-.2,0],[-.1,0]]),1,0.)]:
    assert abs(projected_origin_offset(p,index)-expected)<1e-12
geometry=json.loads((BASE/'geometry.json').read_text(encoding='utf-8'))
rows=[]
for name,data in geometry.items():
    config=FollowConfig(**data['config'])
    for row in data['rows']:
        candidate=short_prefix_evidence(row['points'],row['side'],config)
        rows.append(dict(run=name,t=row['t'],old=row['short']['reason'],new=candidate['reason'],
            supported=candidate['supported'],lead=candidate.get('one_sided_lead_in_m'),
            correction=candidate.get('origin_arc_correction_m'),span=candidate.get('shared_span_m')))
with (BASE/'origin_check.json').open('x',encoding='utf-8') as stream:
    json.dump(dict(projection_checks=checks,geometry=rows),stream,ensure_ascii=False,indent=2)
for row in rows:print(json.dumps(row,ensure_ascii=False))
