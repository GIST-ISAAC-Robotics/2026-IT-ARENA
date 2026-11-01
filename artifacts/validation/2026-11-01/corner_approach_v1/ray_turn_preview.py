"""관측 경로 예측의 넓이를 통과하는 각 광선이 그 뒤까지 실제 반환을 주는지 비교(시험 전용)."""
import math
import numpy as np

from arena_autonomy.local_path import braking_speed
from arena_autonomy.observation_support import source_valid
from sensor_turn_probe import rollout


def _slab(origin, direction, half):
    parallel=np.abs(direction)<1.e-12
    div=np.where(parallel,1.,direction)
    t0=(-half-origin)/div; t1=(half-origin)/div
    inside=np.abs(origin)<=half
    return (np.where(parallel,np.where(inside,-np.inf,np.inf),np.minimum(t0,t1)),
            np.where(parallel,np.where(inside,np.inf,-np.inf),np.maximum(t0,t1)))


def check_rays(scene,poses,cfg,held_distance):
    """같은 차체를 근거리 반사/무반환 뒤까지 밀지 않는다. 진리 자세를 사용하지 않는다."""
    rays=scene.rays
    origin=np.asarray([r.ray_origin_xy_m for r in rays])
    direction=np.asarray([r.direction_xy for r in rays])
    hit=np.asarray([r.hit_xy_m if r.status=='return' else (np.nan,np.nan) for r in rays])
    ranges=np.linalg.norm(hit-origin,axis=1)
    margin=.025+cfg['measurement_padding_m']+cfg['object_motion_bound_mps']*np.asarray([
        scene.t_ref_s-r.acquired_at_s for r in rays])
    # 각 3 cm 구간의 끝 자세에서, 그 구간의 차체 전체가 이동할 수 있는 거리까지 팽창한다.
    # 점에 대한 기존 사각 팽창량은 그대로다. 축 변환 중 팽창된 사각형 모서리도 포함한다.
    L=cfg['wheelbase_m']; lx=cfg['length_m']/2; ly=cfg['width_m']/2
    k=math.tan(cfg['max_steering_rad'])/L
    hop=np.diff(poses[:,3],prepend=0.)[:,None]*(1+k*np.hypot(L/2+lx+margin,ly+margin))
    c=np.cos(poses[:,2:3]);s=np.sin(poses[:,2:3])
    dx=hit[:,0]-poses[:,0:1];dy=hit[:,1]-poses[:,1:2]
    if np.any((abs(c*dx+s*dy)<=lx+margin+hop)&(abs(-s*dx+c*dy)<=ly+margin+hop)):
        return 'observed_hit_remains'
    # 초기 조향 유지 중에는 기존 조향 원호도 같은 통로를 검사했다. 경로 조향으로 갈라지는
    # 첫 구간부터는 차체에 걸치는 광선을 모두 본다. 표적 뒤의 가려진 경로도 허용하지 않는다.
    start=np.flatnonzero(poses[:,3]>=held_distance)
    if not len(start):return 'unchanged_steering_only'
    i=start[0];c=c[i:];s=s[i:];hop=hop[i:]
    dx=origin[:,0]-poses[i:,0:1];dy=origin[:,1]-poses[i:,1:2]
    x=c*dx+s*dy;y=-s*dx+c*dy
    vx=c*direction[:,0]+s*direction[:,1];vy=-s*direction[:,0]+c*direction[:,1]
    # 운동 보정 후 빔 사이 간격이 위치마다 다르다. 인접 빔 방향 간격 중 큰 값의 절반으로
    # 각 광선 폭을 잡는다. 유한한 반환/깊이를 확인하며 각도 사이의 모든 실물은 보장하지 않는다.
    angles=np.unwrap(np.arctan2(direction[:,1],direction[:,0]))
    gap=np.diff(np.r_[angles[-1]-math.tau,angles,angles[0]+math.tau])
    half_angle=.5*np.maximum(gap[:-1],gap[1:])
    # source_valid의 내부 광선 간격은 2° 이내지만 한 회전 양 끝은 더 벌어질 수 있다.
    # 그 빈 각도가 실제로 검사하는 차체에 걸릴 때만 거부한다. 뒤쪽 이음매로 앞길을 막지 않는다.
    seam=gap[0]
    if seam>math.radians(2.):
        mx=(origin[0]+origin[-1])*.5
        # 이음매 양 끝의 센서 원점 차이도 영역에 포함한다.
        mdx=mx[0]-poses[i:,0:1];mdy=mx[1]-poses[i:,1:2]
        mid=angles[0]-seam/2;ux=math.cos(mid);uy=math.sin(mid)
        uncertainty=np.linalg.norm(origin[0]-origin[-1])*.5
        # hop의 광선별 상계 중 가장 큰 값 사용
        bound=hop.max(axis=1,keepdims=True)
        width=(np.hypot(mdx,mdy)+np.hypot(lx+bound,ly+bound)+uncertainty)*math.sin(seam/2)+uncertainty
        lo1,hi1=_slab(c*mdx+s*mdy,c*ux+s*uy,lx+bound+width)
        lo2,hi2=_slab(-s*mdx+c*mdy,-s*ux+c*uy,ly+bound+width)
        if np.any(np.minimum(hi1,hi2)>=np.maximum(np.maximum(lo1,lo2),0.)):
            return 'unobserved_scan_seam_crosses_body'
    extent=np.hypot(dx,dy)+np.hypot(lx+hop,ly+hop)
    cone=extent*np.sin(np.clip(half_angle,0.,math.radians(2.)))
    a,b=_slab(x,vx,lx+hop+cone);d,e=_slab(y,vy,ly+hop+cone)
    far=np.minimum(b,e); crossed=far>=np.maximum(np.maximum(a,d),0.)
    if np.any(crossed & ~np.isfinite(ranges)):return 'current_ray_support_missing'
    if np.any(crossed & (far+margin>=ranges)):return 'surface_occludes_path'
    if np.any(crossed.sum(axis=1)<2):return 'insufficient_crossing_rays'
    return 'additional_20cm_supported_in_sample'


def classify(scene,path,span,arc,angle,prev,speed,cfg,old,age):
    if not source_valid(scene):return 'invalid_current_scan'
    desired=min(braking_speed(span-cfg['standstill_gap_m'],age),(span-cfg['standstill_gap_m'])/cfg['headway_s'])
    rollout_speed=max(.25,abs(speed),old,desired)
    held=rollout_speed*(age+.10)
    for initial in (angle,) if angle==prev else (angle,prev):
        poses=rollout(path,span,initial,speed,cfg,max(old,desired),age)
        if poses is None:return 'path_ends_during_prediction'
        status=check_rays(scene,poses,cfg,held)
        if status!='additional_20cm_supported_in_sample':return status
    return status
