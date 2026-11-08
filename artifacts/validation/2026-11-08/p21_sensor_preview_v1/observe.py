"""P21: 실제 취득한 현재 센서점의 왼쪽 직선→사선 연결. 비주행 진단 전용."""
import math
import numpy as np

def line(points):
    p=np.asarray(points,float);m=np.mean(p,axis=0)
    _,_,vh=np.linalg.svd(p-m,full_matrices=False);v=vh[0]
    if v[0]<0:v=-v
    along=(p-m)@v;normal=np.array([-v[1],v[0]])
    return dict(m=m,v=v,heading=math.degrees(math.atan2(v[1],v[0])),
                lo=float(along.min()),hi=float(along.max()),span=float(np.ptp(along)),
                error=float(np.max(np.abs((p-m)@normal))))

def observe(scene):
    # Only current sequential, motion-compensated occupied endpoints and raw adjacency.
    # No map, progress, marker truth, or gap-filled ranges.
    candidates=[];groups=[];ring=[]
    for ray in scene.rays:
        p=ray.hit_xy_m
        eligible=p is not None and -.18<=p[0]<=2.6 and .10<=p[1]<=2.4
        if not eligible or (ring and (ray.source_index!=ring[-1][0]+1 or math.dist(p,ring[-1][1])>.18)):
            if len(ring)>=14:groups.append(ring)
            ring=[]
        if eligible:ring.append((ray.source_index,p))
    if len(ring)>=14:groups.append(ring)
    for group in groups:
        # Split near a corner and compare two independently supported wall tangents.
        # On a left wall the original ray order usually sees the downstream segment first.
        # Restrict maximum physical support length to avoid merging remote walls.
        pts=np.array([p for _,p in group])
        for split in range(6,len(pts)-6):
            lo=split;hi=split
            while lo>0 and np.linalg.norm(pts[lo-1]-pts[split])<.9:lo-=1
            while hi+1<len(pts) and np.linalg.norm(pts[hi+1]-pts[split])<.9:hi+=1
            if split-lo<5 or hi-split<5:continue
            segments=[line(pts[lo:split+1]),line(pts[split:hi+1])]
            a,b=sorted(segments,key=lambda r:r['heading'])
            if abs(a['heading'])>18 or not 30<=b['heading']-a['heading']<=60:continue
            if min(a['span'],b['span'])<.24 or max(a['error'],b['error'])>.023:continue
            mat=np.column_stack([a['v'],-b['v']])
            t,u=np.linalg.solve(mat,b['m']-a['m'])
            corner=a['m']+t*a['v']
            # Arrival along the straight wall then departure along forward-left diagonal.
            if abs(t-a['hi'])>.10 or abs(u-b['lo'])>.10 or not -.12<corner[0]<2.3:continue
            cost=(a['error']+b['error']+abs(t-a['hi'])+abs(u-b['lo']))
            candidates.append((cost,dict(x=float(corner[0]),y=float(corner[1]),
                  straight_deg=a['heading'],turn_deg=b['heading']-a['heading'],
                  straight_span=a['span'],diagonal_span=b['span'],max_residual=max(a['error'],b['error']))))
    return min(candidates,key=lambda c:c[0])[1] if candidates else None
