from pathlib import Path
import json, math
import numpy as np
OUT=Path(__file__).parent
data=json.loads((OUT/'sections.json').read_text(encoding='utf-8'))
def hull(points):
    p=sorted(set(map(tuple,points)))
    cross=lambda o,a,b:(a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    low=[];high=[]
    for q in p:
        while len(low)>=2 and cross(low[-2],low[-1],q)<=0:low.pop()
        low.append(q)
    for q in p[::-1]:
        while len(high)>=2 and cross(high[-2],high[-1],q)<=0:high.pop()
        high.append(q)
    h=np.array(low[:-1]+high[:-1]);return np.vstack([h,h[0]])

def angle_bounds(poly,origin=(75.4,0)):
    a=np.rad2deg(np.unwrap(np.arctan2(poly[:,1]-origin[1],poly[:,0]-origin[0])))
    return [float(a.min()),float(a.max())]

posts=[]
for r in data['134.6']:
    for sign in [-1,1]:
        p=np.array([p for line in r['lines'] for p in line if p[1]*sign>0]); posts.append((r['id'],sign,hull(p)))
cases=[]
for t in [0,2,4,6]:
    polys=[];bands=[]
    for idx,sign,p in posts:
        ang=np.linspace(0,2*np.pi,73);delta=t*np.c_[np.cos(ang),np.sin(ang)]
        q=hull((p[:,None,:]+delta[None,:,:]).reshape(-1,2));polys.append(q)
        bands.append({'id':idx,'side':sign,'angles':angle_bounds(q),'bounds':[q.min(axis=0).tolist(),q.max(axis=0).tolist()]})
    rear=min(q[:,0].min() for q in polys)
    cases.append({'radial_extra_mm':t,'blocked_deg':sum(b['angles'][1]-b['angles'][0] for b in bands),'minimum_vehicle_length_mm':103.500001072884-min(-96.200017957608,rear),'post_lateral_max_mm':max(abs(q[:,1]).max() for q in polys),'bands':bands})
(OUT/'wire_cases.json').write_text(json.dumps(cases,indent=2),encoding='utf-8')
print(json.dumps(cases,indent=2))
