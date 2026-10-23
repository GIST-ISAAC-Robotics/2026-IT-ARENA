"""운영 미연결: 외곽 조각-광선 필요조건을 묶어서 계산. 최종 clipping은 동일."""
import math
import numpy as np
from arena_autonomy.observation_support import outside_body
from arena_autonomy.linear_observation import clip_halfplane


def sweep_queries(scene,steer,distance,config):
    curve=math.tan(steer)/config.wheelbase_m
    lx,wy=config.length_m/2,config.width_m/2
    corner=np.array([[-lx,-wy],[lx,-wy],[lx,wy],[-lx,wy]])
    border=np.unique(np.vstack(([(x,y) for x in np.linspace(-lx,lx,11) for y in (-wy,wy)],
                                [(x,y) for x in (-lx,lx) for y in np.linspace(-wy,wy,9)])),axis=0)
    unknown=[r for r in scene.rays if r.status!='return']
    queries=[]; thin=0
    co,si=math.cos(math.pi/500),math.sin(math.pi/500)
    origins=np.array([r.ray_origin_xy_m for r in unknown]).reshape(-1,2)
    directions=np.array([r.direction_xy for r in unknown]).reshape(-1,2)
    lower_all=directions@np.array([[co,-si],[si,co]])
    upper_all=directions@np.array([[co,si],[-si,co]])
    polygons=[]; fragments=[]; owners=[]
    for travel in np.r_[np.arange(.02,distance,.02),distance]:
        yaw=curve*travel;c,s=math.cos(yaw),math.sin(yaw)
        x=travel if curve==0 else math.sin(yaw)/curve
        y=0. if curve==0 else 2*math.sin(yaw*.5)**2/curve
        center=np.array([x-config.wheelbase_m/2+config.wheelbase_m/2*c,y+config.wheelbase_m/2*s])
        rotation=np.array([[c,s],[-s,c]])
        b=border@rotation+center
        queries.extend(b[(abs(b[:,0])>lx+1e-10)|(abs(b[:,1])>wy+1e-10)])
        polygon=(corner@rotation+center).tolist()
        for exterior in outside_body(polygon,config.length_m,config.width_m):
            fragments.append(exterior);owners.append(len(polygons))
        polygons.append(polygon)
    possible=np.zeros((len(polygons),len(unknown)),dtype=bool)
    if fragments and unknown:
        # 첫 꼭짓점을 반복해 길이를 맞춰도 all 판정은 같다. 새 허용치는 넣지 않는다.
        count=max(map(len,fragments))
        padded=np.array([p+[p[0]]*(count-len(p)) for p in fragments])
        # 한 번에 최대 32조각. 긴 이동/전량 결측에서도 메모리 크기를 제한한다.
        for start in range(0,len(fragments),32):
            delta=padded[start:start+32,None]-origins[None,:,None]
            low=lower_all[None,:,None,0]*delta[:,:,:,1]-lower_all[None,:,None,1]*delta[:,:,:,0]
            high=upper_all[None,:,None,0]*delta[:,:,:,1]-upper_all[None,:,None,1]*delta[:,:,:,0]
            forward=np.sum(directions[None,:,None]*delta,axis=3)
            hit=~(np.all(low<0,axis=2)|np.all(high>0,axis=2)|np.all(forward < -1e-12,axis=2))
            np.logical_or.at(possible,owners[start:start+32],hit)
    for j,polygon in enumerate(polygons):
        for i in np.flatnonzero(possible[j]):
            r=unknown[i]
            cut=clip_halfplane(clip_halfplane(polygon,r.ray_origin_xy_m,lower_all[i].tolist(),1),
                               r.ray_origin_xy_m,upper_all[i].tolist(),-1)
            for piece in outside_body(cut,config.length_m,config.width_m):
                queries.extend(piece);queries.append(np.mean(piece,axis=0));thin+=1
    queries=np.asarray(queries).reshape(-1,2)
    exterior=(abs(queries[:,0])>lx+1e-10)|(abs(queries[:,1])>wy+1e-10)
    return np.unique(queries[exterior],axis=0),thin
