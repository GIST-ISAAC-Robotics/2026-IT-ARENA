"""저장 센서 입력용 단독 직선 속도 후보. 경로·조향·정지 상태를 쓰거나 변경하지 않는다."""
import math

import numpy as np

from arena_autonomy.local_path import braking_speed
from arena_autonomy.observation_support import source_valid
from arena_autonomy.turn_preview import check_rays


def wall_run(xy, indices, sign, cfg):
    """차체 가까이서 시작하는 같은 각도 순서의 실제 옆면만. 끊긴 뒤의 먼 면을 합치지 않는다."""
    x, y = xy[:, 0], sign * xy[:, 1]
    near = (x >= 0.) & (x <= .35) & (np.abs(y-cfg.wall_offset_m) <= cfg.boundary_anchor_error_m)
    if near.sum() < 3:
        return None
    side_y = float(np.median(y[near]))
    valid = np.flatnonzero((x >= -.05) & (np.abs(y-side_y) <= cfg.measurement_padding_m))
    if not len(valid):
        return None
    cuts = np.flatnonzero(np.abs(np.diff(indices[valid])) != 1) + 1
    found = []
    for group in np.split(valid, cuts):
        # 최소 하나의 근거리부터 어디까지 실제로 연속 관측했는지 찾는다.
        vals = x[group]
        if len(vals) < 3 or np.min(vals) > .35 or np.count_nonzero(near[group]) < 3:
            continue
        if vals[0] > vals[-1]:
            vals = vals[::-1]
        if np.any(np.maximum.accumulate(vals)-vals > cfg.measurement_padding_m):
            continue
        found.append((float(vals[-1]), float(np.min(y[group]))))
    return max(found) if found else None


def curvature_and_length(path, speed, cfg):
    """pursue_path의 원래 경로/시선점 곡률 한도. 벽 거리로 이 항을 늘리지 않는다."""
    rear = np.array(path, dtype=float, copy=True)
    rear[:, 0] += cfg.wheelbase_m/2
    distance = np.linalg.norm(rear, axis=1)
    nearest = int(np.argmin(distance))
    ahead = np.flatnonzero((rear[:, 0] > .10) & (np.arange(len(rear)) >= nearest))
    if len(ahead) < 3 or max(distance[ahead]) < .25:
        return None
    look = min(.60, max(.30, .30+.12*abs(speed)))
    point = rear[ahead[np.argmin(abs(distance[ahead]-look))]]
    k = abs(float(2*point[1]/max(.01,point@point)))
    ds = np.r_[0., np.cumsum(np.linalg.norm(np.diff(rear,axis=0),axis=1))]
    dx = np.gradient(rear,axis=0)
    ddx = np.gradient(dx,axis=0)
    curve = np.abs(dx[:,0]*ddx[:,1]-dx[:,1]*ddx[:,0])/np.maximum(1e-8,np.linalg.norm(dx,axis=1)**3)
    cap = min(math.sqrt(cfg.lateral_acceleration_mps2/max(.15,k)),
              float(np.sqrt(cfg.lateral_acceleration_mps2/np.maximum(.15,curve[nearest:])+
                            4*np.maximum(0.,ds[nearest:]-ds[nearest]-.2)).min()))
    return cap,float(ds[-1]-ds[nearest])


def eligible(output, previous, measured, cfg):
    v, steer, d = output
    # near-field detector accepts no HOLD, no traffic handoff, no missing-return exception.
    return bool(cfg.solo_route_only and cfg.blend_open_speed_by_length and not cfg.ray_wall_continuity
                and cfg.open_route_speed_mps > 0 and v > cfg.max_speed_mps and measured >= 0
                and math.isfinite(previous) and math.isfinite(steer) and d.get('reason')=='local_follow'
                and d.get('open_route_speed_enabled') and d.get('boundary_from_memory') is False
                and not d.get('corridor_selected') and not d.get('boundary_memory_from_corridor')
                and d.get('partial_observation',{}).get('reason')=='fully_observed')


def estimate(output, scene, previous, measured, cfg):
    v, steer, d = output
    if not eligible(output,previous,measured,cfg):
        return v, 'original_state'
    path = np.asarray(d['path_points'])
    fwd = path[path[:,0] >= 0]
    if len(fwd)<3 or np.max(abs(fwd[:,1]))>cfg.measurement_padding_m:
        return v,'existing_path_curved'
    age = scene.t_ref_s - scene.source_first_s
    if not 0 <= age < .30 or d.get('path_source_s')!=scene.source_first_s:
        return v,'source'
    if any(r.status!='return' and abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))<=math.radians(100)
           for r in scene.rays):
        return v,'partial'
    ordered = [r for r in scene.rays if r.status=='return']
    xy = np.asarray([r.hit_xy_m for r in ordered])
    index = np.asarray([r.source_index for r in ordered])
    if len(index)<30 or np.any(np.diff(index)<=0) or not np.isfinite(xy).all():
        return v,'points'
    both = [wall_run(xy,index,s,cfg) for s in (1,-1)]
    if any(w is None for w in both):
        return v,'two_sides_not_continuous'
    # 보수적인 차체 전단·거리 오차 차감. 길이는 조향점으로 변환하지 않는다.
    span = min(d['observed_search_horizon_m'], min(w[0] for w in both)
               - math.hypot(cfg.length_m,cfg.width_m)/2 - cfg.measurement_padding_m)
    if span <= d['observed_path_ahead_m']:
        return v,'no_longer_budget'
    near = curvature_and_length(path,measured,cfg)
    if near is None:
        return v,'no_original_path'
    cap, remaining = near
    if not math.isclose(remaining,d['observed_path_ahead_m'],abs_tol=1e-8):
        return v,'different_original_path'
    available = max(0.,d['swept_clearance_m']-cfg.standstill_gap_m)
    fixed = min(cap,cfg.open_route_speed_mps,braking_speed(available,age),available/cfg.headway_s)
    def target(distance):
        share=np.clip((distance-1.10)/.60,0.,1.)
        return min(fixed,braking_speed(distance,age),
                   cfg.max_speed_mps+float(share)*(cfg.open_route_speed_mps-cfg.max_speed_mps))
    if not math.isclose(target(remaining),v,abs_tol=1e-8):
        return v,'different_original_limit'
    new = target(span)
    if new <= v+1e-8:
        return v,'other_speed_limit'
    # 제동에 필요한 거리 동안, 현재/직전 조향 원호 둘 모두 차체 폭을 포함해 양 벽 안에 있어야 한다.
    stop = .06+(age+.10)*max(measured,new) + max(measured,new)**2/4
    if stop>span+1e-8:
        return v,'already_faster_than_budget'
    k=max(abs(math.tan(a)) for a in (steer,previous))/cfg.wheelbase_m
    lateral=cfg.width_m/2+k*stop*((cfg.wheelbase_m+cfg.length_m)/2+stop/2)
    margin=.025+cfg.measurement_padding_m+cfg.object_motion_bound_mps*age
    if lateral+margin >= min(w[1] for w in both):
        return v,'arc_leaves_observed_sides'
    # 두 옆면은 속도 검색 길이의 근거일 뿐이다. 전방 표적 뒤 가려진 경로는 허가하지 않는다.
    if not source_valid(scene):
        return v,'invalid_scan'
    samples=np.r_[np.arange(.03,stop,.03),stop]
    held=max(measured,new)*(age+.10)
    for a in (steer,) if steer==previous else (steer,previous):
        curvature=math.tan(a)/cfg.wheelbase_m
        theta=curvature*samples
        # sinc 표현은 직진과 거의 직진인 각에서도 소거 오차가 없다.
        x=samples*np.sinc(theta/np.pi)+cfg.wheelbase_m/2*(np.cos(theta)-1)
        y=samples*np.sinc(theta/(2*np.pi))*np.sin(theta/2)+cfg.wheelbase_m/2*np.sin(theta)
        status=check_rays(scene,np.column_stack((x,y,theta,samples)),vars(cfg),held)
        if status!='observed_turn_clear':
            return v,status
    return new,'straight_budget'
