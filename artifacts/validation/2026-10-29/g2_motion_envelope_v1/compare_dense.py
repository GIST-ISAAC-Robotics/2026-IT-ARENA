"""팽창 도형과 표본 사이 이동 보정의 효과를 분리하는 저장 입력 진단."""
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from rounded_sweep import rounded_swept_limit

BASE=Path(__file__).resolve().parent
source=BASE.parent/'g2_tracking_preview_v1/prototype2_stop_diagnosis.json'


def dense_contact(points,padding,steering,shape):
    curve=math.tan(steering)/.135
    points=np.asarray(points);padding=np.asarray(padding)
    for start in np.arange(0.,2.5,.1):
        s=start+np.arange(1000)*.0001
        yaw=curve*s
        rx=s-.0675 if abs(curve)<1e-12 else np.sin(yaw)/curve-.0675
        ry=np.zeros_like(s) if abs(curve)<1e-12 else (1-np.cos(yaw))/curve
        dx,dy=points[None,:,0]-rx[:,None],points[None,:,1]-ry[:,None]
        x=np.cos(yaw[:,None])*dx+np.sin(yaw[:,None])*dy-.0675
        y=-np.sin(yaw[:,None])*dx+np.cos(yaw[:,None])*dy
        ox,oy=np.maximum(abs(x)-.155,0.),np.maximum(abs(y)-.13,0.)
        d=np.hypot(ox,oy) if shape=='disk' else np.maximum(ox,oy)
        hit=np.flatnonzero(np.any(d<=padding[None,:]-.055+1e-12,axis=1))
        if len(hit):return float(s[hit[0]])
    return None


if __name__=='__main__':
    out=BASE/'saved_dense_comparison.json'
    if out.exists():raise RuntimeError('preserve prior output')
    raw=json.loads(source.read_text());rows=[]
    for item in raw['samples']:
        g=item['geometry']
        for old in g['first_interference']:
            angle=old['steering'];row=dict(time_s=item['t'],steering_rad=angle,original_axis_clearance_m=old['clearance_m'])
            for shape in ('axis','disk'):
                row[shape+'_interval_lower_m']=rounded_swept_limit(g['points'],angle,wheelbase=.135,horizon=2.5,margin=g['padding_m'],motion_shape=shape)
                row[shape+'_dense_first_contact_m']=dense_contact(g['points'],g['padding_m'],angle,shape)
                if row[shape+'_dense_first_contact_m'] is not None:
                    assert row[shape+'_interval_lower_m']<=row[shape+'_dense_first_contact_m']+1e-12
            rows.append(row)
    result=dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),dense_step_m=.0001,
                code_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in BASE.glob('*.py')},
                scope='dense diagnostic is a sampled comparison; interval lower bound uses analytic maximum motion',cases=rows)
    out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(rows,indent=2))
