from compare_step import *
from itertools import groupby

from PIL import Image
import sys
sys.path.insert(0,str(OUT.parents[1]/'src/arena_autonomy'))
from arena_autonomy.cad_mount import bounds
ORIGIN=np.array([75.4,0.])
DEG=np.arange(-180.,180.,.01)
R=np.deg2rad(DEG)
D=np.array([np.cos(R),np.sin(R)]).T
def cast(lines):
    r=np.full(len(DEG),np.inf)
    for line in lines:
        for a,b in zip(line,line[1:]):
            a=np.asarray(a)-ORIGIN;b=np.asarray(b)-ORIGIN;v=b-a
            den=D[:,0]*v[1]-D[:,1]*v[0]
            ok=abs(den)>1e-9
            t=np.divide(a[0]*v[1]-a[1]*v[0],den,out=np.full(len(D),np.inf),where=ok)
            u=np.divide(a[0]*D[:,1]-a[1]*D[:,0],den,out=np.full(len(D),np.inf),where=ok)
            test=(t>=0)&(u>=-1e-9)&(u<=1+1e-9)
            r[test]=np.minimum(r[test],t[test])
    return r
def spans(r):
    q=[]
    for v, idx in groupby(range(len(r)),lambda i:bool(np.isfinite(r[i]))):
        if not v:continue
        first,*rest=idx
        last=rest[-1] if rest else first
        q.append([round(DEG[first],2),round(DEG[last],2),float(np.min(r[first:last+1])),float(np.max(r[first:last+1]))])
    return q
DCT=get(OUT,'section_comparison.json')
results={}
ARR={}
for z in DCT['old']:
    a,b=[cast([t for e in DCT[k][z] for t in e['lines']]) for k in ('old','new')]
    ARR[z]=(a,b)
    joint=np.isfinite(a)&np.isfinite(b)
    gain=np.isfinite(b)&~np.isfinite(a);lose=np.isfinite(a)&~np.isfinite(b)
    results[z]=dict(old=spans(a),new=spans(b),new_only=spans(np.where(gain,b,np.inf)),
                    old_only=spans(np.where(lose,a,np.inf)),shared_max_depth_delta_mm=float(np.max(abs(a[joint]-b[joint]))),
                    shared_over_0p1mm_deg=round(np.count_nonzero(abs(a[joint]-b[joint])>.1)*.01,2))
    print(z,results[z],flush=True)
save('ray_comparison.json',results)

