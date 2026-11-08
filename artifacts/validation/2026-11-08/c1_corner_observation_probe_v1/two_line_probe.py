"""RDP 꼭짓점 수 대신 같은 원 점의 두 선분 잔차로 분할 후보를 고른다."""
from pathlib import Path
import json, types, math
import numpy as np
import arena_autonomy.piecewise_wall as pw

A=Path(__file__).resolve().parent

def fit(points):
    center=points.mean(axis=0)
    tangent=np.linalg.svd(points-center,full_matrices=False)[2][0]
    if tangent@(points[-1]-points[0])<0:tangent=-tangent
    residual=np.abs((points-center)@[-tangent[1],tangent[0]])
    return tangent,residual

def best_vertices(samples,tolerance=.025):
    options=[]
    for split in range(4,len(samples)-4):
        a,b=samples[:split+1],samples[split:]
        if min(np.linalg.norm(a[-1]-a[0]),np.linalg.norm(b[-1]-b[0]))<.15:continue
        t0,r0=fit(a);t1,r1=fit(b)
        angle=math.atan2(float(np.cross(t0,t1)),float(t0@t1))
        if not math.radians(40)<=abs(angle)<=math.radians(135):continue
        if max(r0.max(),r1.max())>.03:continue
        options.append((float(np.sum(r0*r0)+np.sum(r1*r1)),split))
    return [0,min(options)[1],len(samples)-1] if options else list(range(len(samples)))

if __name__=='__main__':
    output=[]
    for row in json.loads((A/'samples.json').read_bytes()):
        if row['window']!=1:continue
        s=np.asarray(row['samples']); vertices=best_vertices(s)
        fn=types.FunctionType(pw.corner_path.__code__,{**pw.corner_path.__globals__,'_vertices':best_vertices},pw.corner_path.__name__,pw.corner_path.__defaults__)
        path=fn(s,row['side'],.505,.135/math.tan(.3710583448410034))
        summary=dict(t=row['t'],side=row['side'],points=len(s),original_vertices=len(row['vertices']),best_vertices=vertices if len(vertices)==3 else None,
            corner_path=None if path is None else path.tolist())
        output.append(summary)
        print({k:v for k,v in summary.items() if k!='corner_path'},'candidate',path is not None)
    (A/'two_line_result.json').write_text(json.dumps(output,indent=2),encoding='utf8')
