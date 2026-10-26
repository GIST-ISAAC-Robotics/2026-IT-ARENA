"""선택형 측방 결측의 현재/최근 광선 지지. 원시 unknown/점유를 바꾸지 않는다.

각도 분해능과 짧은 평면 운동 이력을 전제로 한 표본 지지이며 연속 자유 공간,
횡슬립, 동적 cut-in 또는 실물 제동을 인증하지 않는다. 명시 시험 옵션에서만 연결한다.
"""
from collections import deque
from dataclasses import replace
import math

import numpy as np

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.scene_motion import relative_poses
from arena_autonomy.linear_observation import clip_halfplane
from arena_autonomy.local_path import swept_limit
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.scan_topology import cyclic_count, adjacent_indices


def needs_recent_support(scene):
    return any(r.status!='return' and abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))<=math.radians(100)
               for r in scene.rays)


def side_missing_candidate(scene):
    """계산 단계만의 진입 조건. 이 값만으로 명령을 발행하면 안 된다."""
    return source_valid(scene) and all(
        r.status=='return' for r in scene.rays
        if abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))<=math.radians(60))


def depth_support(scene, queries, padding, *, cycle=None):
    """가능한 양옆 광선을 포함하는 각도 상계. 결측/동률/반환 조건은 불변.

    원점 원 내부는 전체 광선으로 계산한다. 어느 광선 원점과든 기존 반경
    수치 경계 1e-9 m 안에 겹친 질의는 방향이 정의되지 않아 지지하지 않는다.
    """
    cycle_count = cyclic_count(scene, cycle)
    if cycle_count is not None:
        checked_pad = np.asarray(padding, dtype=float)
        if (checked_pad.shape != (len(scene.rays),) or not np.all(np.isfinite(checked_pad))
                or np.any(checked_pad < 0)):
            raise ValueError('finite nonnegative per-ray padding required')
    q = np.asarray(queries, dtype=float).reshape(-1, 2)
    result = np.zeros(len(q), dtype=bool)
    if not len(q):
        return result
    origins = np.array([r.ray_origin_xy_m for r in scene.rays])
    directions = np.array([r.direction_xy for r in scene.rays])
    indices = np.array([r.source_index for r in scene.rays])
    valid = np.array([r.status == 'return' for r in scene.rays])
    depth = np.array([math.dist(r.ray_origin_xy_m, r.hit_xy_m) if r.status == 'return' else -1. for r in scene.rays])
    pad = np.asarray(padding)
    n = len(origins)
    center = origins.mean(axis=0)
    radius = float(np.max(np.linalg.norm(origins-center, axis=1)))+1e-12
    theta = np.mod(np.arctan2(directions[:, 1], directions[:, 0]), math.tau)
    order = np.argsort(theta, kind='stable')
    bearings = np.concatenate((theta[order]-math.tau, theta[order], theta[order]+math.tau))
    ray_order = np.tile(order, 3)

    def evaluate(part, chosen):
        present = chosen < n
        take = np.minimum(chosen, n-1)
        dx = part[:, 0, None]-origins[take, 0]
        dy = part[:, 1, None]-origins[take, 1]
        # 원점 근접 질의는 원점 원+1e-9 경계 안이므로 전체 광선 fallback이다.
        # 선택된 한 광선만 검사하면 signed-zero에 따라 다른 광선을 고를 수 있다.
        origin_overlap = np.any(present & (dx*dx+dy*dy <= 1e-18), axis=1)
        angles = np.arctan2(directions[take, 0]*dy-directions[take, 1]*dx,
                            directions[take, 0]*dx+directions[take, 1]*dy)
        lower = np.argmin(np.where(present & (angles >= -1e-10), np.maximum(angles, 0.), np.inf), axis=1)
        upper = np.argmin(np.where(present & (angles <= 1e-10), np.maximum(-angles, 0.), np.inf), axis=1)
        k = np.arange(len(part))
        lo, hi = take[k, lower], take[k, upper]
        low_radius = np.sqrt(dx[k, lower]**2+dy[k, lower]**2)
        high_radius = np.sqrt(dx[k, upper]**2+dy[k, upper]**2)
        return (~origin_overlap & present[k, lower] & present[k, upper] & (angles[k, lower] >= -1e-10)
            & (angles[k, upper] <= 1e-10) & ((abs(angles[k, lower])+abs(angles[k, upper])) <= .025)
            & adjacent_indices(indices[lo], indices[hi], cycle_count=cycle_count) & valid[lo] & valid[hi]
            & (low_radius > 1e-9) & (high_radius > 1e-9)
            & (depth[lo]-low_radius-pad[lo] > 0) & (depth[hi]-high_radius-pad[hi] > 0))

    for start in range(0, len(q), 128):
        part = q[start:start+128]
        delta = part-center
        distance = np.linalg.norm(delta, axis=1)
        window = np.isfinite(distance) & (distance > radius+1e-9)
        fallback = np.flatnonzero(~window)
        if len(fallback):
            result[start+fallback] = evaluate(part[fallback], np.broadcast_to(np.arange(n), (len(fallback), n)))
        rows = np.flatnonzero(window)
        if not len(rows):
            continue
        phi = np.mod(np.arctan2(delta[rows, 1], delta[rows, 0]), math.tau)
        half = np.arcsin(np.minimum(1., radius/distance[rows]))+.025+2e-10
        low = np.searchsorted(bearings, phi-half, side='left')
        high = np.searchsorted(bearings, phi+half, side='right')
        width = high-low
        nonempty = width > 0
        rows, low, width = rows[nonempty], low[nonempty], width[nonempty]
        if not len(rows):
            continue
        maximum = int(width.max())
        if maximum >= n*.75:
            chosen = np.broadcast_to(np.arange(n), (len(rows), n))
        else:
            offsets = np.arange(maximum)
            positions = np.minimum(low[:, None]+offsets, len(ray_order)-1)
            chosen = np.sort(np.where(offsets < width[:, None], ray_order[positions], n), axis=1)
        result[start+rows] = evaluate(part[rows], chosen)
    return result


def source_valid(scene):
    if (scene.source_frame_id!='laser_frame' or scene.source_first_s is None
            or not 0<=scene.t_ref_s-scene.source_first_s<.3 or len(scene.rays)!=500
            or [r.source_index for r in scene.rays]!=list(range(500))):
        return False
    # 500점 C1 합성/적합 입력 전용. 회전 중 원주 끝 간격은 변할 수 있다.
    # 끝 간격을 0으로 맞추지 않고 내부 방향 순서와 실제 취득 시간 폭을 검사한다.
    angles=np.unwrap([math.atan2(r.direction_xy[1],r.direction_xy[0]) for r in scene.rays])
    gaps=np.diff(angles)
    return bool(np.all((gaps>0)&(gaps<=math.radians(2))) and 5.5<=angles[-1]-angles[0]<=7.
                and .08<=scene.source_last_s-scene.source_first_s<=.12)


def transport_scene(scene,history,now):
    dx,dy,yaw=relative_poses(history,[scene.t_ref_s],now)[0]
    c,s=math.cos(yaw),math.sin(yaw)
    def vector(p): return (c*p[0]-s*p[1],s*p[0]+c*p[1])
    def point(p):
        a,b=vector(p); return (a+dx,b+dy)
    # 같은 생성자와 모든 유효성 검사는 유지한다. 500회 replace의 필드명 조회만
    # 생략하며 좌표 연산 순서·원시 시각·세대·unknown/masked 상태는 그대로다.
    rays=tuple(type(r)(point(r.ray_origin_xy_m),
                       None if r.hit_xy_m is None else point(r.hit_xy_m),
                       r.acquired_at_s,r.status,r.source_frame_id,r.source_epoch,r.source_index,
                       vector(r.direction_xy)) for r in scene.rays)
    return replace(scene,t_ref_s=now,rays=rays)


class RecentScenes:
    """원래 취득 시각 유지. 같은 스캔으로 TTL을 갱신하지 않고 역행/세대 변경 시 비운다."""
    def __init__(self): self.reset()

    def reset(self):
        self.scenes=deque(maxlen=3)
        self.last_ref=self.epoch=None

    def observe(self,scene,history, *, transport=True):
        if (not source_valid(scene) or
                (self.last_ref is not None and scene.t_ref_s<self.last_ref) or
                (self.scenes and scene.source_first_s<self.scenes[-1].source_first_s)):
            self.reset(); return [],'observation_invalid_or_regressed'
        if self.epoch is not None and self.epoch!=scene.source_epoch: self.reset()
        self.last_ref,self.epoch=scene.t_ref_s,scene.source_epoch
        if not self.scenes or scene.source_first_s>self.scenes[-1].source_first_s: self.scenes.append(scene)
        while self.scenes and scene.t_ref_s-self.scenes[0].source_first_s>=.3: self.scenes.popleft()
        available=[scene]
        if not transport: return available,'observations_cached'
        for old in reversed(self.scenes):
            if old.source_first_s==scene.source_first_s: continue
            try: available.append(transport_scene(old,history,scene.t_ref_s))
            except MotionUnavailable: continue
        return available,'observations_ready'


def outside_body(polygon,length,width):
    """볼록 polygon에서 현 명목 몸체 밖인 네 반평면 조각. 겹침은 무해하다."""
    if not polygon:
        return []
    # 축 정렬 몸체의 범위 밖으로 전혀 나가지 않은 반평면은 clipping하지 않는다.
    # 면적 0인 경계만의 교차도 기존 area>1e-14 조건에서 원래 제외됐다.
    xs,ys=zip(*polygon)
    xmin,xmax,ymin,ymax=min(xs),max(xs),min(ys),max(ys)
    result=[]
    planes=(((length/2,0.),(0.,1.),-1,xmax>length/2,xmin>=length/2),
            ((-length/2,0.),(0.,1.),1,xmin<-length/2,xmax<=-length/2),
            ((0.,width/2),(1.,0.),1,ymax>width/2,ymin>=width/2),
            ((0.,-width/2),(1.,0.),-1,ymin<-width/2,ymax<=-width/2))
    for origin,direction,sign,possible,entire in planes:
        if not possible:
            continue
        piece=polygon if entire else clip_halfplane(polygon,origin,direction,sign)
        if len(piece)>=3:
            area=abs(sum(a[0]*b[1]-a[1]*b[0] for a,b in zip(piece,piece[1:]+piece[:1])))*.5
            if area>1e-14: result.append(piece)
    return result


def sweep_queries(scene,steer,distance,config):
    """2cm 외곽과 unknown 교차 polygon. 묶음 필요조건 뒤 원래 clipping 유지."""
    curve=math.tan(steer)/config.wheelbase_m
    lx,wy=config.length_m/2,config.width_m/2
    corner=np.array([[-lx,-wy],[lx,-wy],[lx,wy],[-lx,wy]])
    border=np.unique(np.vstack(([(x,y) for x in np.linspace(-lx,lx,11) for y in (-wy,wy)],
                                [(x,y) for x in (-lx,lx) for y in np.linspace(-wy,wy,9)])),axis=0)
    unknown=[r for r in scene.rays if r.status!='return']
    queries=[]; thin=0
    half=math.pi/500
    co,si=math.cos(half),math.sin(half)
    origins=np.array([r.ray_origin_xy_m for r in unknown]).reshape(-1,2)
    directions=np.array([r.direction_xy for r in unknown]).reshape(-1,2)
    lower_all=directions@np.array([[co,-si],[si,co]])
    upper_all=directions@np.array([[co,si],[-si,co]])
    polygons=[]; fragments=[]; owners=[]
    for travel in np.r_[np.arange(.02,distance,.02),distance]:
        yaw=curve*travel; c,s=math.cos(yaw),math.sin(yaw)
        x=travel if curve==0 else math.sin(yaw)/curve
        # 1-cos(yaw)는 직진에 가까울 때 소거 오차로 0이 된다.
        # 반각식은 같은 원호이며 작은 조향을 실제보다 크게 옆으로 흔들지 않는다.
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
        # 같은 꼭짓점을 반복해 배열 길이를 맞춰도 all 필요조건은 바뀌지 않는다.
        count=max(map(len,fragments))
        padded=np.array([p+[p[0]]*(count-len(p)) for p in fragments])
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
                queries.extend(piece); queries.append(np.mean(piece,axis=0)); thin+=1
    queries=np.asarray(queries).reshape(-1,2)
    # clip의 닫힌 교차 조각에는 현재 몸체 경계 꼭짓점도 들어간다.
    # 일반 외곽 표본과 동일한 기존 1e-10 m 수치 허용치로 제외한다.
    # 새 진입 영역이나 unknown을 자유 공간으로 치환하는 검사가 아니다.
    exterior=(abs(queries[:,0])>lx+1e-10)|(abs(queries[:,1])>wy+1e-10)
    return np.unique(queries[exterior],axis=0),thin


def sweep_support(scenes,speed,target,steering,previous_steering,config, *, protected_sensor_pose=None):
    detail=dict(supported=False,reason='observation_invalid_state',unknown_filled=False,
        free_area_certified=False,continuous_sweep_verified=False,dynamic_cut_in_verified=False,
        actual_body_stop_verified=False,earlier_supported_queries=0)
    if (not scenes or not all(source_valid(s) for s in scenes)
            or not all(math.isfinite(v) for v in (speed,target,steering,previous_steering))
            or not 0<=speed<=1.1 or not 0<target<=1.
            or max(abs(steering),abs(previous_steering))>config.max_steering_rad): return detail
    current=scenes[0]
    # 현재 정면 센서 실패는 과거 이력으로 감추지 않는다. 측방만 이력 후보의 대상이다.
    if any(r.status!='return' and abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))<=math.radians(60)
           for r in current.rays):
        return {**detail,'reason':'current_forward_unobserved'}
    age=current.t_ref_s-current.source_first_s
    v=max(speed,target)
    distance=config.standstill_gap_m+.06+v*(age+.10)+v*v/4
    if distance>2.5: return {**detail,'reason':'observation_horizon_exceeded'}
    padding=per_ray_padding(current,.025+config.measurement_padding_m,config.object_motion_bound_mps)
    points=current.occupied_endpoints_xy_m
    if len(points)<30: return {**detail,'reason':'observation_too_few_returns'}
    for angle in set((steering,previous_steering)):
        clearance=swept_limit(points,angle,config.wheelbase_m,horizon=2.5,margin=padding,
                              length=config.length_m,width=config.width_m)
        if clearance<distance: return {**detail,'reason':'current_occupancy_overrides_history',
                                      'required_distance_m':distance,'clearance_m':clearance}
    queries=[];thin=0
    for angle in set((steering,previous_steering)):
        q,n=sweep_queries(current,angle,distance,config);queries.extend(q);thin+=n
    queries=np.unique(np.asarray(queries).reshape(-1,2),axis=0)
    protected=None
    if protected_sensor_pose is not None:
        from arena_autonomy.observation_lateral import protected_queries, reachable_from_unknown
        protected=protected_queries(queries,protected_sensor_pose)
        detail.update(protected_scope='current_sensor_plus_minus_100deg',
                      protected_queries=int(protected.sum()), rear_queries=int((~protected).sum()),
                      rear_dynamic_risk_unresolved=True, lateral_rejected_queries=0)
    supported=np.zeros(len(queries),dtype=bool); sources=[]
    for i,s in enumerate(scenes):
        if (s.source_epoch!=current.source_epoch or s.t_ref_s!=current.t_ref_s
                or s.source_first_s>current.source_first_s): continue
        pad=np.array([.025+config.measurement_padding_m+config.object_motion_bound_mps*(s.t_ref_s-r.acquired_at_s) for r in s.rays])
        remaining=np.flatnonzero(~supported)
        accepted=depth_support(s,queries[remaining],pad)
        if protected is not None:
            checking=np.flatnonzero(accepted & protected[remaining])
            reached=reachable_from_unknown(s,queries[remaining[checking]],config)
            accepted[checking[reached]]=False
            detail['lateral_rejected_queries']+=int(reached.sum())
        added=int(accepted.sum());supported[remaining[accepted]]=True
        if i>0: detail['earlier_supported_queries']+=added
        sources.append(dict(source_first_s=s.source_first_s,added=added))
    ok=bool(len(queries) and supported.all())
    return {**detail,'supported':ok,'reason':'sampled_recent_observation_support' if ok else 'sweep_unobserved',
        'required_distance_m':distance,'queries':len(queries),'supported_queries':int(supported.sum()),
        'thin_intersections':thin,'sources':sources}
