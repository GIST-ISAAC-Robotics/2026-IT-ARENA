"""선택형 저속 직진 복구의 관측 계약. 기본 전방 게이트의 대체 기본값 아님.

미관측 방향을 free로 채우지 않는다. 현재 명목 차체의 전면부터 새로 진입할
직사각형 영역을 검사한다. 회전/횡미끄럼/미관측 측방 cut-in의 안전 증명이 아니다.
독립 보호와 계획기가 같은 순수 계산을 각자의 센서 입력으로 호출할 수 있다.
"""
import math

import numpy as np


RECOVERY_SPEED_MPS = .2
QUIET_YAW_RATE_RAD_S = .05
QUIET_INTERVAL_S = .1


def clip_halfplane(polygon, origin, direction, sign):
    """닫힌 볼록 polygon을 sign*cross(direction,p-origin)>=0으로 자른다."""
    result = []
    if not polygon:
        return result
    def value(p):
        return sign*(direction[0]*(p[1]-origin[1])-direction[1]*(p[0]-origin[0]))
    previous, pv = polygon[-1], value(polygon[-1])
    for current in polygon:
        cv = value(current)
        if (cv >= 0) != (pv >= 0):
            t = pv/(pv-cv)
            result.append((previous[0]+t*(current[0]-previous[0]),previous[1]+t*(current[1]-previous[1])))
        if cv >= 0:
            result.append(current)
        previous, pv = current, cv
    return result


def sector_rectangle(origin, direction, half_angle, bounds):
    """무한 전방 부채꼴과 직사각형의 교차. 거리값을 만들지 않는다."""
    x0,y0,x1,y1 = bounds
    polygon = [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
    c,s = math.cos(half_angle),math.sin(half_angle)
    lower = (c*direction[0]+s*direction[1],-s*direction[0]+c*direction[1])
    upper = (c*direction[0]-s*direction[1],s*direction[0]+c*direction[1])
    return clip_halfplane(clip_halfplane(polygon,origin,lower,1),origin,upper,-1)


def quiet_rotation(history, now):
    """관측 자이로만 확인한다. 낮은 자이로/모터 값은 실제 차체 정지 증명이 아니다."""
    if not math.isfinite(now) or now < QUIET_INTERVAL_S:
        return False
    stamps, values = history.gyro.available(now)
    first = max(0, int(np.searchsorted(stamps,now-QUIET_INTERVAL_S,side='right'))-1)
    stamps, values = stamps[first:], values[first:]
    if (len(stamps) < 2 or stamps[0] > now-QUIET_INTERVAL_S
            or now-stamps[-1] > history.config.max_extrapolation_s
            or np.max(np.diff(stamps)) > history.config.max_input_gap_s+1e-9):
        return False
    return bool(np.all(np.abs(values) <= QUIET_YAW_RATE_RAD_S))


def straight_path_supported(path, distance, tolerance=.08):
    """현재 실제 반환 기반 경로와의 제한 일치. 지도/노면/추월 판정 아님."""
    points = np.asarray(path,dtype=float)
    if (points.ndim != 2 or points.shape[1] != 2 or not np.all(np.isfinite(points))
            or not math.isfinite(distance) or distance <= 0 or not 0 < tolerance <= .08):
        return False
    selected = points[(points[:,0]>=.05)&(points[:,0]<=distance+.04)]
    return bool(len(selected)>=5 and selected[:,0].min()<=.10
                and selected[:,0].max()>=distance and np.max(np.abs(selected[:,1]))<=tolerance)


def linear_observation(scene, speed, *, length=.20, width=.15, standstill_gap=.16,
                       measurement_padding=.03, object_motion_bound=.5):
    """새 전면 진입 띠의 원래 각 셀/유한 깊이 지지. 0.2 m/s 이하 직진 후보 전용.

    각 셀 전체에 실제 센서의 각도 분해능보다 보수적인 2도 폭을 사용한다.
    유한 반환은 분해능 한계 안의 깊이 근거이지 광선 사이 물체 부재 증명이 아니다.
    side/rear unknown은 그대로 남으며 필요 영역과 겹치면 무조건 거절한다.
    """
    result = dict(supported=False,reason='invalid_linear_observation',unknown_filled=False,
                  free_area_certified=False,dynamic_cut_in_verified=False,actual_body_stop_verified=False)
    inputs = (speed,length,width,standstill_gap,measurement_padding,object_motion_bound)
    if (not all(math.isfinite(v) for v in inputs) or not 0 <= speed <= 2.5
            or min(inputs[1:]) <= 0 or scene.source_first_s is None
            or not 0 <= scene.t_ref_s-scene.source_first_s < .3
            or scene.source_frame_id != 'laser_frame' or len(scene.rays)<180):
        return result
    if [r.source_index for r in scene.rays] != list(range(len(scene.rays))):
        return {**result,'reason':'source_cells_incomplete'}
    age = scene.t_ref_s-scene.source_first_s
    v = max(speed, RECOVERY_SPEED_MPS)
    distance = standstill_gap+.06+v*(age+.10)+v*v/4
    if distance > 2.5:
        return {**result,'reason':'linear_horizon_exceeded'}
    # 0.05 rad/s가 정지까지 유지된다는 제한 가정에서 가능한 측방 이동도 덧붙인다.
    # 실제 조향 응답/슬립을 증명하지 않으므로 운용 전에 별도 회귀가 필요하다.
    stopping_time = age+.10+v/2
    yaw_margin = (distance+length/2)*QUIET_YAW_RATE_RAD_S*stopping_time
    side_margin = .025+measurement_padding+object_motion_bound*age+yaw_margin
    bounds = (length/2,-width/2-side_margin,length/2+distance,width/2+side_margin)
    result.update(required_distance_m=distance,required_bounds_m=list(bounds),
                  assumed_yaw_margin_m=yaw_margin,unknown_irrelevant=0,return_cells_checked=0)
    # 원래 시각/원점으로 확인. 전체 원주 세포의 방위 정렬이 무너진 장면은 거절한다.
    angles = np.unwrap([math.atan2(r.direction_xy[1],r.direction_xy[0]) for r in scene.rays])
    gaps = np.diff(angles)
    seam = 2*math.pi-(angles[-1]-angles[0])
    if (np.any(gaps <= 0) or np.any(gaps > math.radians(2))
            or not 0 < seam <= math.radians(2)):
        return {**result,'reason':'projected_angular_order_or_gap'}
    half = math.radians(1)
    for ray in scene.rays:
        intersection = sector_rectangle(ray.ray_origin_xy_m,ray.direction_xy,half,bounds)
        if not intersection:
            result['unknown_irrelevant'] += ray.status != 'return'
            continue
        if ray.status != 'return':
            return {**result,'reason':'linear_region_unobserved','first_rejected_index':ray.source_index,
                    'first_rejected_status':ray.status}
        radial = max(math.dist(ray.ray_origin_xy_m,p) for p in intersection)
        padding = .025+measurement_padding+object_motion_bound*(scene.t_ref_s-ray.acquired_at_s)
        measured = math.dist(ray.ray_origin_xy_m,ray.hit_xy_m)
        if measured <= radial+padding:
            return {**result,'reason':'linear_region_depth_insufficient','first_rejected_index':ray.source_index,
                    'needed_ray_depth_m':radial+padding,'measured_ray_depth_m':measured}
        result['return_cells_checked'] += 1
    if result['return_cells_checked'] < 2:
        return {**result,'reason':'linear_region_not_covered'}
    # 별개 취득 원점의 이동 때문에 셀 사이의 미세 틈이 생길 수 있다.
    # 가까운/먼 전면과 양 측면의 2cm 표본에 양측 실제 방향 지지도 요구한다.
    x = np.linspace(bounds[0],bounds[2],int(math.ceil(distance/.02))+1)
    y = np.linspace(bounds[1],bounds[3],int(math.ceil((bounds[3]-bounds[1])/.02))+1)
    queries = np.unique(np.vstack(([(a,b) for a in x for b in (bounds[1],bounds[3])],
                                   [(a,b) for a in (bounds[0],bounds[2]) for b in y])),axis=0)
    origins = np.array([r.ray_origin_xy_m for r in scene.rays])
    directions = np.array([r.direction_xy for r in scene.rays])
    delta = queries[:,None]-origins[None]
    errors = np.arctan2(directions[None,:,0]*delta[:,:,1]-directions[None,:,1]*delta[:,:,0],
                        np.sum(directions[None]*delta,axis=2))
    left = np.min(np.where(errors>=0,errors,np.inf),axis=1)
    right = np.min(np.where(errors<=0,-errors,np.inf),axis=1)
    if np.any(left+right > math.radians(2)):
        return {**result,'reason':'linear_boundary_angular_gap'}
    return {**result,'supported':True,'reason':'linear_observation_supported',
            'boundary_queries':len(queries),'continuous_free_space_verified':False}
