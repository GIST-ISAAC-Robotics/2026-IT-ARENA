"""미연결 후보: 현재까지 이송한 입구 경로를 유지하고 관측 출구를 연장한다."""
import math
from dataclasses import astuple
import numpy as np
from arena_autonomy.p22_path import Pose,segment,curve,WB
from arena_autonomy.p22_observation import segments


def retained_poses(body):
    body=np.asarray(body,float)
    if body.ndim!=2 or body.shape[1]!=2 or len(body)<3 or not np.all(np.isfinite(body)):
        raise ValueError('invalid retained entry path')
    tangent=np.gradient(body,axis=0)
    heading=np.unwrap(np.arctan2(tangent[:,1],tangent[:,0]))
    rear=body-WB/2*np.column_stack((np.cos(heading),np.sin(heading)))
    distances=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(rear,axis=0),axis=1))]
    curvature=np.gradient(heading)/np.maximum(1e-6,np.gradient(distances))
    return [Pose(float(x),float(y),float(h),float(np.clip(math.atan(WB*k),-.37,.37)),float(s))
            for (x,y),h,k,s in zip(rear,heading,curvature,distances)]


def propose(scene,previous,entry_path):
    segs=segments(scene);candidates=[]
    original=retained_poses(entry_path)
    # 마지막 꼬리는 원 입구 경로의 직선이다. 새 조향 램프를 현재 차체 자세에서 다시 만들지 않는다.
    if abs(original[-1].steer)>1e-5:return []
    for right in segs:
        if not (abs(right['heading_deg'])<15 and .2<-right['m'][1]<.4 and right['span']>=.30):continue
        v=right['v'];n=np.array([v[1],-v[0]]);end=right['m']+right['hi']*v
        if not .15<end[0]<1.2:continue
        left=[s for s in segs if s['m'][1]>.15 and abs(s['heading_deg']-right['heading_deg'])<8
              and .5<float((s['m']-right['m'])@(-n))<.7 and s['span']>=.30]
        if len(left)!=1:continue
        mid=(left[0]['m']+right['m'])/2
        yaw=math.atan2(v[1],v[0]);q=original[-1]
        offset=float((np.array(entry_path[-1])-mid)@n)
        angle=math.atan2(math.sin(q.yaw-yaw),math.cos(q.yaw-yaw))
        if abs(offset)>.04 or abs(angle)>math.radians(5):continue
        for front in segs:
            if abs(float(front['v']@v))>math.sin(math.radians(8)) or front['span']<1.:continue
            c=float(front['m']@v);width=c-float(end@v)
            if not .90<width<1.12:continue
            ends=[front['m']+front[k]*front['v'] for k in ('lo','hi')]
            if max(float((p-end)@n) for p in ends)<.25:continue
            steer=-math.atan(WB/.36);target=yaw-math.pi/2;trial=[q]
            if not curve(trial,q.yaw,steer,target,.65):continue
            e=trial[-1];d=(c-.505-float(np.array([e.x,e.y])@v))/math.cos(q.yaw-yaw)
            if d<0:continue
            path=list(original);segment(path,d,0.)
            if not curve(path,q.yaw,steer,target,.65):continue
            segment(path,.55,0.)
            candidates.append(dict(right_end=end.tolist(),right_heading_deg=right['heading_deg'],
                front_normal_distance_m=c,main_wall_width_m=width,front_span_m=front['span'],
                retained_points=len(original),retained_offset_m=offset,straight_before_turn_m=q.s+d,
                poses=[astuple(p) for p in path]))
    return candidates
