"""선정 단독 경로의 조향과 원래 곡률 한도를 유지하는 선택형 직선 속도 계산."""
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


def eligible(output, previous, measured, cfg):
    v, steer, d = output
    # 단독의 완전 관측·현재 경로에서만 증속한다. 기존 정지/교통 판단은 유지한다.
    return bool(cfg.solo_route_only and cfg.blend_open_speed_by_length and not cfg.ray_wall_continuity
                and cfg.open_route_speed_mps > 0 and v > cfg.max_speed_mps and measured >= 0
                and math.isfinite(previous) and math.isfinite(steer) and d.get('reason')=='local_follow'
                and d.get('open_route_speed_enabled') and d.get('boundary_from_memory') is False
                and not d.get('corridor_selected') and not d.get('boundary_memory_from_corridor')
                and d.get('partial_observation',{}).get('reason')=='fully_observed')


def evaluate(output, scene, previous, measured, cfg):
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
    # 두 값은 원래 Pure Pursuit가 같은 경로에서 이미 계산했다.
    cap, remaining = d.get('original_curve_speed_cap_mps'), d['observed_path_ahead_m']
    if cap is None or not math.isfinite(cap):
        return v,'no_original_curve_budget'
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
