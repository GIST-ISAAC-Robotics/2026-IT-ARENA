"""현재 통로 끝과 맞은편 긴 벽을 이용한 우회전 출구 제안. 미연결/단독 전용."""
import math
from dataclasses import astuple
import numpy as np
from arena_autonomy.p22_path import Pose,segment,curve,WB,RATE
from arena_autonomy.p22_observation import segments

def propose(scene,previous):
    segs=segments(scene);candidates=[]
    # 통로를 따른 오른쪽 짧은 벽은 경로 근거만으로 사용한다. 모든 반환은 점유 검사에 남는다.
    for right in segs:
        if not (abs(right['heading_deg'])<15 and .2<-right['m'][1]<.4 and right['span']>=.30):continue
        v=right['v'];n=np.array([v[1],-v[0]])
        end=right['m']+right['hi']*v
        if not .15<end[0]<1.2:continue
        left=[s for s in segs if s['m'][1]>.15 and abs(s['heading_deg']-right['heading_deg'])<8
              and .5<float((s['m']-right['m'])@(-n))<.7 and s['span']>=.30]
        if not left:continue
        for front in segs:
            angle=abs(float(front['v']@v))
            if angle>math.sin(math.radians(8)) or front['span']<1.:continue
            c=float(front['m']@v)
            width=c-float(end@v)
            if not .90<width<1.12:continue
            ends=[front['m']+front[k]*front['v'] for k in ('lo','hi')]
            if max(float((q-end)@n) for q in ends)<.25:continue
            yaw=math.atan2(v[1],v[0]);target=yaw-math.pi/2
            steer=-math.atan(WB/.36)
            path=[Pose(steer=previous)]
            segment(path,abs(previous)*.65/RATE,0.)
            q=path[-1];trial=[q]
            if not curve(trial,q.yaw,steer,target,.65):continue
            e=trial[-1]
            d=(c-.505-float(np.array([e.x,e.y])@v))/math.cos(q.yaw-yaw)
            if d<0:continue
            segment(path,d,0.)
            if not curve(path,q.yaw,steer,target,.65):continue
            segment(path,.55,0.)
            candidates.append(dict(right_end=end.tolist(),right_heading_deg=right['heading_deg'],
                front_normal_distance_m=c,main_wall_width_m=width,front_span_m=front['span'],
                straight_before_turn_m=q.s+d,poses=[astuple(p) for p in path]))
    return candidates
