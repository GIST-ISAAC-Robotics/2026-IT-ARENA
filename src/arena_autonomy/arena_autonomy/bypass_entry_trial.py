"""G2 직선 진입의 센서 입력 비교. ROS 출력/운영 연결 없음.

관측된 양쪽 벽에서만 기준선을 만들고 현재 명령의 반응·제동 영역을
검사한다. 미래 경로의 반환점 간섭 점수는 자유 공간/우회 성공 증명이 아니다.
정지 물체 ID, 지도, 도로 다각형, 정답 자세는 받지 않는다.
"""
from dataclasses import dataclass
import math

import numpy as np

from arena_autonomy.local_follow import FollowConfig, pursue_path
from arena_autonomy.bypass_wall_fit import tracked_straight_wall
from arena_autonomy.local_path import local_path, swept_limit, braking_speed
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.observation_support import source_valid
from arena_autonomy.observation_support import depth_support
import arena_autonomy.m3_execution_trial as mx
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.pass_release_trial import (SelectorConfig, lateral_config,
    motion_region, region_blocked)


@dataclass(frozen=True)
class EntryConfig:
    """사전 비교값. 기존 측방 상한 .35 m/s 및 조향 변화율 3 rad/s 사용."""
    offsets_m: tuple = (0., -.16, .16, -.20, .20, -.24, .24)
    transitions_m: tuple = (.6, .9, 1.2)
    speed_cap_mps: float = .35
    path_step_m: float = .02
    prefer_earlier_transition: bool = False

    def __post_init__(self):
        if type(self.prefer_earlier_transition) is not bool:
            raise ValueError('explicit transition comparison required')
        if (not isinstance(self.offsets_m, tuple) or len(self.offsets_m) > 9
                or not self.offsets_m or self.offsets_m[0] != 0.
                or len(set(self.offsets_m)) != len(self.offsets_m)
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) or abs(v) > .27 for v in self.offsets_m)):
            raise ValueError('bounded unique entry offsets starting at zero required')
        if (not isinstance(self.transitions_m, tuple) or not 1 <= len(self.transitions_m) <= 4
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) or not .6 <= v <= 1.5 for v in self.transitions_m)
                or tuple(sorted(set(self.transitions_m))) != self.transitions_m):
            raise ValueError('bounded increasing transition lengths required')
        if (any(isinstance(v, bool) or not math.isfinite(v) for v in
                (self.speed_cap_mps, self.path_step_m))
                or not 0 < self.speed_cap_mps <= .35 or not .005 <= self.path_step_m <= .02):
            raise ValueError('invalid entry comparison bounds')


def observed_straight_wall(scene, config, side):
    """지금 보인 한쪽 연결 벽의 직선 부분. 다른 반환점은 제거하지 않는다."""
    points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
    sign = 1 if side == 'left' else -1
    path, detail = local_path(points[sign*points[:, 1] > .10], side,
                              offset=config.wall_offset_m, horizon=1.8)
    if path is None:
        return None, dict(reason='observed_wall_reference_missing', side=side, wall=detail)
    path = np.asarray(path)
    part = path[(path[:, 0] >= -.15) & (path[:, 0] <= 1.5)]
    if len(part) < 12 or np.ptp(part[:, 0]) < .30:
        return None, dict(reason='observed_wall_reference_short', side=side)
    slope, intercept = np.linalg.lstsq(np.c_[part[:, 0], np.ones(len(part))], part[:, 1], rcond=None)[0]
    residual = float(np.max(abs(part[:, 1]-slope*part[:, 0]-intercept)))
    if residual > .01:
        return None, dict(reason='straight_entry_only', side=side, residual_m=residual)
    if abs(math.atan(slope)) > math.radians(30):
        return None, dict(reason='reference_heading_outside_trial', side=side)
    fit = (float(slope), float(intercept), float(part[:, 0].min()), float(part[:, 0].max()))
    return fit, dict(slope=fit[0], intercept_m=fit[1], residual_m=residual, observed_x_m=list(fit[2:]))


def straight_reference(scene, config):
    """두 관측 벽의 지역 경로가 합의한 실제 공통 x 구간만 사용한다."""
    fitted = []
    evidence = {}
    for side in ('left', 'right'):
        fit, detail = observed_straight_wall(scene, config, side)
        if fit is None:
            return None, detail
        fitted.append(fit)
        evidence[side] = detail
    lower, upper = max(f[2] for f in fitted), min(f[3] for f in fitted)
    # 두 벽의 공통 방향/폭 합의는 기존 path_agrees의 25 cm 관측 범위에 맞춘다.
    # 경로 길이는 아래에서 해당 방향 외곽 벽의 실제 관측 구간으로 별도 제한한다.
    if lower > -config.wheelbase_m/2 or upper-max(lower, 0.) < .25:
        return None, dict(reason='reference_does_not_cover_entry', walls=evidence)
    a, b = fitted
    disagreement = max(abs((a[0]-b[0])*x+a[1]-b[1]) for x in (lower, upper))
    if disagreement > config.boundary_agreement_m:
        return None, dict(reason='observed_wall_reference_disagrees', disagreement_m=float(disagreement), walls=evidence)
    return ((a[0]+b[0])/2, (a[1]+b[1])/2, lower, upper), dict(
        reason='current_two_wall_straight_reference', walls=evidence,
        disagreement_m=float(disagreement), wall_offset_assumption_m=config.wall_offset_m)


def entry_path(reference, offset, config, entry, transition_m=None):
    """5차 전이의 관측된 구간만 생성. 끝까지 편이에 도달하지 않는 접두 경로 허용."""
    slope, intercept, lower, upper = reference
    rear = -config.wheelbase_m/2
    transition_m = entry.transitions_m[-1] if transition_m is None else transition_m
    if transition_m not in entry.transitions_m:
        raise ValueError('unregistered transition length')
    if lower > rear or upper-rear < .30:
        raise ValueError('observed entry prefix too short')
    x = np.linspace(rear, upper, math.ceil((upper-rear)/entry.path_step_m)+1)
    t = np.clip((x-rear)/transition_m, 0., 1.)
    blend = 10*t**3-15*t**4+6*t**5
    # offset은 관측 기준선의 법선 거리다. 차체가 기울면 y 절편과 다르다.
    y = blend*(slope*x+intercept+offset*math.sqrt(1+slope*slope))
    return np.c_[x, y]


def preview_clearance(path, points, padding, config):
    """미래 경로의 현재 반환점 간섭 순위만 계산. 관측/미래 충돌 인증 아님."""
    delta = np.gradient(path, axis=0)
    yaw = np.arctan2(delta[:, 1], delta[:, 0])
    # 굽은 경로의 호길이를 쓰면 장애물이 없는 곳에서도 큰 편이가 유리해진다.
    # 직선 기준의 공통 전방 x 진행량으로만 비교한다.
    progress = path[:, 0]-path[0, 0]
    center = path + config.wheelbase_m/2*np.c_[np.cos(yaw), np.sin(yaw)]
    relative = points[None, :, :] - center[:, None, :]
    x = np.cos(yaw[:, None])*relative[:, :, 0]+np.sin(yaw[:, None])*relative[:, :, 1]
    y = -np.sin(yaw[:, None])*relative[:, :, 0]+np.cos(yaw[:, None])*relative[:, :, 1]
    occupied = np.any((abs(x) <= config.length_m/2+padding) &
                      (abs(y) <= config.width_m/2+padding), axis=1)
    hits = np.flatnonzero(occupied)
    return float(progress[max(0, int(hits[0])-1)] if len(hits) else progress[-1])


def preview_point_gap(path, points, padding, config):
    """현재 반환과 미래 표본 차체의 최소 축별 여유. 순위용이며 자유 공간 증명 아님."""
    delta=np.gradient(path,axis=0);yaw=np.arctan2(delta[:,1],delta[:,0])
    center=path+config.wheelbase_m/2*np.c_[np.cos(yaw),np.sin(yaw)]
    relative=points[None,:,:]-center[:,None,:]
    x=np.cos(yaw[:,None])*relative[:,:,0]+np.sin(yaw[:,None])*relative[:,:,1]
    y=-np.sin(yaw[:,None])*relative[:,:,0]+np.cos(yaw[:,None])*relative[:,:,1]
    return float(np.min(np.maximum(abs(x)-config.length_m/2-padding,abs(y)-config.width_m/2-padding)))


def current_outer_wall_padding(scene, config, side, wall, growth_s=0.):
    """현재 긴 공선 반환을 고정 외벽으로 해석하는 명시 시험 가정.

    모든 반환/기본 25 mm와 측정 30 mm 여유는 유지한다. 관측된 연결 길이가
    65 cm 이상인 외곽 면에만 취득 후 물체 이동 항을 적용하지 않는다. 고정 벽
    가정이지 물체 분류 증명은 아니며 함께 정렬된 복수 물체 오인은 잔여 위험이다.
    """
    # growth_s는 같은 스캔을 더 쓰는 동안 늘어날 취득 후 이동 여유다(기본 0).
    padding=per_ray_padding(scene,.025+config.measurement_padding_m,config.object_motion_bound_mps)+config.object_motion_bound_mps*growth_s
    points=np.asarray(scene.occupied_endpoints_xy_m,dtype=float).reshape(-1,2)
    evidence=dict(policy='current_long_outer_boundary_static_assumption',static_return_count=0,
        all_returns_retained=True,static_object_identity_verified=False,accepted_spans_m=[])
    if side not in ('left','right') or not source_valid(scene) or not len(points):
        return padding,evidence
    sign=1 if side=='left' else -1
    slope,intercept=wall['slope'],wall['intercept_m']
    norm=math.sqrt(1+slope*slope)
    actual_intercept=intercept+sign*config.wall_offset_m*norm
    aligned=(abs(points[:,1]-slope*points[:,0]-actual_intercept)/norm<=.015)&(
        points[:,0]>=-.4)&(points[:,0]<=1.5)
    ray_indices=np.asarray([ray.source_index for ray in scene.rays if ray.status=='return'])
    if wall.get('reason')=='current_connected_straight_wall_fit':
        # 새 관측에서 실제 적합에 쓰인 연속 반환만 분류한다. 누락 광선을 메우지 않는다.
        aligned=np.isin(ray_indices,wall['source_ray_indices'])&(points[:,0]>=-.4)&(points[:,0]<=1.5)
    indices=np.flatnonzero(aligned)
    indices=indices[np.argsort(ray_indices[indices] if wall.get('reason')=='current_connected_straight_wall_fit' else points[indices,0])]
    gaps=np.linalg.norm(np.diff(points[indices],axis=0),axis=1)
    limits=.10+.035*np.minimum(np.linalg.norm(points[indices[:-1]],axis=1),np.linalg.norm(points[indices[1:]],axis=1))
    # 누락/다른 면을 넘어서 여러 작은 차량 면을 하나의 긴 벽으로 잇지 않는다.
    breaks=(gaps>limits)|(abs(np.diff(ray_indices[indices]))!=1)
    mask=np.zeros(len(points),dtype=bool)
    for piece in np.split(indices,np.flatnonzero(breaks)+1):
        if len(piece)>=12 and np.ptp(points[piece,0])>=.65:
            mask[piece]=True
            evidence['accepted_spans_m'].append(float(np.ptp(points[piece,0])))
    padding[mask]=.025+config.measurement_padding_m
    evidence['static_return_count']=int(mask.sum())
    return padding,evidence


def entry_motion_region(scene,speed,target,steer,previous,dt,config,lateral,occupied_padding=None):
    """같은 고정 외벽 가정을 점유와 관측 증거에 일관되게 적용한다.

    기본 경로에서는 기존 함수를 그대로 쓴다. 선택형일 때도 미반환/마스크의
    여유는 바꾸지 않으며 실제 끝점을 앞당기거나 관측되지 않은 점을 만들지 않는다.
    """
    context={}
    region=motion_region(scene,speed,target,steer,previous,dt,config,lateral,context=context)
    if occupied_padding is None or 'queries' not in context:
        return region
    pad=mx.ray_padding(scene,lateral,.5)
    returns=np.asarray([r.status=='return' for r in scene.rays])
    original=per_ray_padding(scene,.025+config.measurement_padding_m,config.object_motion_bound_mps)
    fixed=occupied_padding<original
    return_indices=np.flatnonzero(returns)
    pad[return_indices[fixed]]=np.maximum(.025+lateral.budget.range_bias_m,occupied_padding[fixed])
    q=context['queries'];free=depth_support(scene,q,pad)
    masked=np.zeros(len(q),dtype=bool)
    if np.any(~free):masked[~free]=mx.masked_only(scene,q[~free],pad)
    region.update(unobserved=int(np.sum(~free & ~masked)),
        masked_left=int(np.sum(masked & (q[:,1]>0))),masked_right=int(np.sum(masked & (q[:,1]<0))),
        masked_tail_extent_m=0.,masked_front_samples=0)
    if np.any(masked):
        tail=q[masked]
        region['masked_tail_extent_m']=float(np.max(np.maximum(abs(tail[:,1])-config.width_m/2,
            -tail[:,0]-config.length_m/2)))
        region['masked_front_samples']=int(np.sum(tail[:,0]>config.length_m/2))
    region['current_boundary_assumption_applied_to_support']=True
    return region


def compare_entries(scene, now, speed, previous_steering, dt, *, config=None, entry=None):
    return _compare_entries(scene, now, speed, previous_steering, dt, config=config, entry=entry)


def _compare_entries(scene, now, speed, previous_steering, dt, *, config=None, entry=None, guide=None,
                     stationary_boundaries=False, padding_growth_s=0.):
    """순수 비교 출력. selected가 있어도 독립 보호/운영 연결 전까지 구동 허가 아님."""
    config, entry = config or FollowConfig(), entry or EntryConfig()
    # 명령 상한과 측정 입력 범위를 구분한다. 모터축 정수 카운트 차분은
    # 정상 저속에서도 명령 상한을 넘을 수 있다. G1 입력 범위 안의 측정값은
    # 자르지 않고 추종과 반응/제동 외곽 검사에 그대로 전달한다.
    result = dict(reason='invalid_input', candidates=[], selected=None,
        control_authorized=False, unknown_filled=False, full_bypass_verified=False,
        road_surface_verified=False, object_identity_or_stationarity_assumed=False,
        scope='current_sensor_straight_entry_shadow')
    if (not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                for v in (now, speed, previous_steering, dt))
            or not 0 <= speed <= config.max_speed_mps+1e-12 or not 0 < dt <= .2
            or abs(previous_steering) > config.max_steering_rad
            or abs(now-scene.t_ref_s) > 1e-9 or not source_valid(scene)):
        return result
    reference, evidence = straight_reference(scene, config) if guide is None else (True, guide['reference'])
    result['reference'] = evidence
    if reference is None:
        return {**result, 'reason':evidence['reason']}
    points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
    if (isinstance(padding_growth_s, bool) or not isinstance(padding_growth_s, (int, float))
            or not math.isfinite(padding_growth_s) or not 0 <= padding_growth_s <= .3):
        return result
    # 0이 아닌 값은 다음 새 스캔 직전 실행 검사가 볼 여유로 같은 비교를 다시 하는 선택형 점검이다.
    padding = (per_ray_padding(scene, .025+config.measurement_padding_m, config.object_motion_bound_mps)
               + config.object_motion_bound_mps*padding_growth_s)
    if guide is not None and guide.get('stationary_outer_boundary',False):
        side=guide['outer_side']
        padding,classification=current_outer_wall_padding(scene,config,side,evidence['walls'][side],padding_growth_s)
        result['outer_boundary_padding']=classification
    age = now-scene.source_first_s
    lateral = lateral_config(config)
    selector = SelectorConfig()
    choices = ([(offset, transition) for offset in entry.offsets_m for transition in
               (entry.transitions_m[-1:] if offset == 0. else entry.transitions_m)] if guide is None
               else [(guide['offset_m'], guide['transition_m'])])
    # 같은 호출 안에서는 장면·속도·나이·직전 조향·설정이 고정이다.
    # 조향 변화율 제한으로 같은 실제 명령이 된 후보의 검사만 재사용한다.
    # 다음 호출/스캔에는 남기지 않으며 입력·관측 계약을 생략하지 않는다.
    clearances, regions = {}, {}
    padding_by_side={}
    def command_clearance(angle,side):
        key=(angle,side)
        if key not in clearances:
            clearances[key] = swept_limit(points, angle, config.wheelbase_m, horizon=2.5,
                margin=padding, length=config.length_m, width=config.width_m)
        return clearances[key]
    for offset, transition in choices:
        row = dict(offset_m=offset, transition_m=transition, eligible=False, reason='path_unavailable',
                   control_authorized=False, unknown_filled=False)
        try:
            walls = evidence['walls']
            wall_name = ('left' if offset > 0 else 'right') if offset else max(
                walls, key=lambda side:walls[side]['observed_x_m'][1])
            if guide is not None:wall_name=guide['outer_side']
            wall = walls[wall_name]
            if stationary_boundaries:
                if wall_name not in padding_by_side:
                    padding_by_side[wall_name]=current_outer_wall_padding(scene,config,wall_name,wall,padding_growth_s)
                padding,classification=padding_by_side[wall_name]
                row['outer_boundary_padding']=classification
            candidate_reference = (wall['slope'],wall['intercept_m'],*wall['observed_x_m'])
            path = (entry_path(candidate_reference, offset, config, entry, transition) if guide is None
                    else np.asarray(guide['path'], dtype=float))
            row.update(current_outer_wall=wall_name, observed_reference_x_m=wall['observed_x_m'],
                transition_reaches_offset=path[-1,0]-path[0,0] >= transition,
                path_extrapolated_beyond_current_wall=False)
            delta = np.gradient(path, axis=0)
            second = np.gradient(delta, axis=0)
            curvature = np.abs(delta[:, 0]*second[:, 1]-delta[:, 1]*second[:, 0])/np.maximum(
                1e-8, np.linalg.norm(delta, axis=1)**3)
            row['preview_maximum_steering_rad'] = float(np.max(np.arctan(config.wheelbase_m*curvature)))
            if row['preview_maximum_steering_rad'] > config.max_steering_rad:
                raise ValueError('preview_path_exceeds_steering_limit')
            cap, steer, remaining = pursue_path(path, speed, config, age)
        except (ValueError, MotionUnavailable) as error:
            result['candidates'].append({**row, 'reason':str(error)})
            continue
        steer = float(np.clip(steer, previous_steering-3*dt, previous_steering+3*dt))
        padding_key=wall_name if stationary_boundaries else None
        clearance = min(command_clearance(steer,padding_key), command_clearance(previous_steering,padding_key))
        # G1의 앞차 추종 간격(standstill/headway/HOLD)을 저속 횡진입의 벽 원호에
        # 그대로 적용하지 않는다. 기존 반응 .1 s/잔여 .06 m/감속 2 m/s²의
        # 제동식과 모든 반환점 시차 팽창으로 속도를 제한한다. 독립 보호는 별도다.
        target = min(entry.speed_cap_mps, cap, braking_speed(clearance, age))
        old_available = max(0., clearance-config.standstill_gap_m)
        row.update(path_points=path.tolist(), steering_rad=steer, target_speed_mps=target,
            command_clearance_m=clearance, reference_remaining_m=remaining,
            speed_policy='entry_reaction_braking_not_follow_headway',
            comparison_follow_target_mps=min(entry.speed_cap_mps, cap,
                braking_speed(old_available, age),old_available/config.headway_s),
            preview_return_progress_m=preview_clearance(path, points, padding, config),
            preview_point_gap_m=preview_point_gap(path,points,padding,config) if stationary_boundaries else 0.,
            maximum_occupancy_padding_m=float(np.max(padding)), occupied_points=len(points))
        # 현재 속도는 target이 낮아져도 낮아졌다고 가정하지 않는다.
        command_key = (steer, target, padding_key)
        if command_key not in regions:
            regions[command_key] = entry_motion_region(scene, speed, target, steer, previous_steering, dt, config, lateral,
                padding if stationary_boundaries or 'outer_boundary_padding' in result else None)
        region = dict(regions[command_key])
        row['motion_region'] = region
        blocked = region_blocked(region, selector)
        if clearance <= 0. or target <= 0.:
            row['reason'] = 'known_occupied_command_hold'
        elif blocked:
            row['reason'] = blocked
        else:
            row.update(eligible=True, reason='observed_entry_command_candidate',
                masked_tail_risk_accepted=bool(region['masked_left'] or region['masked_right']))
        result['candidates'].append(row)
    eligible = [r for r in result['candidates'] if r['eligible']]
    if not eligible:
        return {**result, 'reason':'no_entry_command_candidate'}
    # 우회 의사결정의 첫 비교다. 미래 전체 경로가 관측되었다는 판정은 아니다.
    # 2 cm 단위 비교로 거의 같은 점수에서는 작은 편이/경로 유지를 우선한다.
    best = max(eligible, key=lambda r:(math.floor(r['preview_return_progress_m']/.02),
                                     math.floor(r['preview_point_gap_m']/.005) if stationary_boundaries else 0,
                                     -abs(r['offset_m']), -r['transition_m'] if entry.prefer_earlier_transition else r['transition_m']))
    result.update(reason='entry_command_candidates_available', selected=dict(
        offset_m=best['offset_m'], transition_m=best['transition_m'],
        speed_mps=best['target_speed_mps'], steering_rad=best['steering_rad'],
        ranking=('current_preview_progress_and_point_gap_not_full_path_free_proof' if stationary_boundaries
                 else 'current_return_preview_then_smaller_offset_not_full_path_free_proof')))
    return result


class EntryIntent:
    """비교 전용 방향 기억. 관측 허가/장애물 부재를 기억하지 않는다.

    첫 현재 후보의 순위가 직진보다 실제로 나을 때 방향을 정하고, 매 호출
    다시 통과한 같은 방향 후보만 선택한다. 막혔다고 반대로 전환하지 않는다.
    전체 우회/복귀의 해제 상태기는 아직 포함하지 않는다.
    """
    def __init__(self): self.reset()

    def reset(self):
        self.sign = None
        self.last_reference = self.last_source = self.epoch = None

    def select(self, scene, comparison):
        result = {**comparison, 'intent_direction':self.sign, 'intent_remembered_permission':False}
        source, now = scene.source_first_s, scene.t_ref_s
        invalid = source is None or not math.isfinite(now) or not math.isfinite(source)
        changed = ((self.epoch is not None and self.epoch != scene.source_epoch)
            or (self.last_reference is not None and now < self.last_reference)
            or (self.last_source is not None and source is not None and source < self.last_source))
        if invalid or changed:
            self.reset()
            return {**result, 'selected':None, 'intent_direction':None, 'reason':'entry_intent_input_reset'}
        self.last_reference, self.last_source, self.epoch = now, source, scene.source_epoch
        eligible = [r for r in comparison['candidates'] if r.get('eligible')]
        if self.sign is None:
            chosen = comparison['selected']
            if chosen is not None and chosen['offset_m'] != 0.:
                straight = [r for r in eligible if r['offset_m'] == 0.]
                selected = next(r for r in eligible if r['offset_m'] == chosen['offset_m']
                                and r['transition_m'] == chosen['transition_m'])
                # 전방 점수 2 cm 격자 두 칸보다 작은 우연 차이로 우회를 시작하지 않는다.
                if straight and selected['preview_return_progress_m']-straight[0]['preview_return_progress_m'] < .04:
                    chosen = dict(offset_m=0., transition_m=straight[0]['transition_m'],
                        speed_mps=straight[0]['target_speed_mps'], steering_rad=straight[0]['steering_rad'],
                        ranking='no_meaningful_current_preview_gain')
                else:
                    self.sign = math.copysign(1., chosen['offset_m'])
            return {**result, 'selected':chosen, 'intent_direction':self.sign}
        same = [r for r in eligible if r['offset_m']*self.sign > 0.]
        if not same:
            return {**result, 'selected':None, 'intent_direction':self.sign,
                    'reason':'entry_committed_side_currently_unavailable'}
        best = max(same, key=lambda r:(math.floor(r['preview_return_progress_m']/.02),
                                      math.floor(r.get('preview_point_gap_m',0.)/.005),
                                      -abs(r['offset_m']), r['transition_m']))
        return {**result, 'intent_direction':self.sign, 'selected':dict(offset_m=best['offset_m'],
            transition_m=best['transition_m'], speed_mps=best['target_speed_mps'],
            steering_rad=best['steering_rad'], ranking='same_intent_currently_rechecked_candidate')}


def reanchor_path_to_wall(path, remembered_wall, current_fit):
    """작은 회전/법선 오차만 현재 벽에 정합한다. 벽 방향 위치는 관측 불가능하다.

    센서 장면/광선/반환점은 움직이지 않는다. 보정한 목표 경로도 현재 점유와
    이동 외곽을 다시 통과해야 한다. 실제 외벽이라는 명시 가정 아래 사용한다.
    """
    path=np.asarray(path,dtype=float)
    old=np.asarray(remembered_wall,dtype=float)
    slope,intercept,_,_=current_fit
    old_slope=(old[1,1]-old[0,1])/(old[1,0]-old[0,0])
    old_intercept=old[0,1]-old_slope*old[0,0]
    heading=math.atan(slope);delta=heading-math.atan(old_slope)
    normal_shift=intercept/math.sqrt(1+slope*slope)-old_intercept/math.sqrt(1+old_slope*old_slope)
    detail=dict(applied=False,rotation_rad=float(delta),normal_shift_m=float(normal_shift),
                along_wall_translation_observed=False,current_scene_transformed=False)
    if abs(delta)>math.radians(2.) or abs(normal_shift)>.03:
        return None,{**detail,'reason':'wall_reanchor_step_exceeds_trial_bounds'}
    rotation=np.array([[math.cos(delta),-math.sin(delta)],[math.sin(delta),math.cos(delta)]])
    normal=np.array([-math.sin(heading),math.cos(heading)])
    corrected=path@rotation.T+normal*normal_shift
    return corrected,{**detail,'applied':True,'reason':'current_wall_normal_and_heading_only'}


class EntryGuide:
    """선택한 진입 곡선을 측정 운동으로 이송하는 비교 전용 추종기.

    최초 두 벽 합의 뒤 바깥 벽을 현재 관측으로 매번 다시 확인한다. 기억은
    조향 목표일 뿐 자유 공간/점유 허가가 아니다. 안쪽의 표적 반환도 제동과
    관측 검사에 그대로 남는다. 중앙 복귀는 명시적으로 선택한 비교에서만 다룬다.
    """
    def __init__(self, config=None, entry=None, *, stationary_outer_boundary=False, rejoin_enabled=False, wall_reanchor=False, tracked_wall_fit=False):
        self.config, self.entry = config or FollowConfig(), entry or EntryConfig()
        if type(stationary_outer_boundary) is not bool:
            raise ValueError('explicit boolean boundary assumption required')
        self.stationary_outer_boundary=stationary_outer_boundary
        if type(rejoin_enabled) is not bool:
            raise ValueError('explicit boolean rejoin comparison required')
        self.rejoin_enabled=rejoin_enabled
        if type(wall_reanchor) is not bool or (wall_reanchor and not stationary_outer_boundary):
            raise ValueError('path reanchor requires explicit stationary outer boundary assumption')
        self.wall_reanchor=wall_reanchor
        if type(tracked_wall_fit) is not bool or (tracked_wall_fit and not wall_reanchor):
            raise ValueError('tracked wall fit requires explicit wall reanchor trial')
        self.tracked_wall_fit=tracked_wall_fit
        self.reset()

    def reset(self):
        self.intent = EntryIntent()
        self.path = self.wall_line = self.reference_s = None
        self.offset = self.transition = self.side = None
        self.last_source = self.epoch = None
        self.wall_confirmations=0
        self.start_walls=self.start_reference_s=None
        self.start_confirmations=0
        self.phase='entry'
        self.rejoin_confirmations=self.center_confirmations=0

    def _hold(self, reason, **detail):
        return dict(reason=reason, candidates=[], selected=None, control_authorized=False,
            unknown_filled=False, full_bypass_verified=False, road_surface_verified=False,
            intent_direction=self.intent.sign, intent_remembered_permission=False, **detail)

    def _confirm_initial_boundaries(self,scene,history,fresh):
        reference,evidence=straight_reference(scene,self.config)
        if reference is None:
            self.start_walls=self.start_reference_s=None
            self.start_confirmations=0
            return False,evidence
        consistent=(self.start_walls is not None and
                    0 <= scene.t_ref_s-self.start_reference_s < self.config.memory_ttl_s)
        if consistent:
            try:
                for side,old in self.start_walls.items():
                    old=transport_points(old,history,self.start_reference_s,scene.t_ref_s)
                    wall=evidence['walls'][side]
                    lo=max(float(np.min(old[:,0])),wall['observed_x_m'][0])
                    hi=min(float(np.max(old[:,0])),wall['observed_x_m'][1])
                    if np.ptp(old[:,0])<.25 or hi-max(lo,0.)<.25:
                        consistent=False;break
                    slope=(old[1,1]-old[0,1])/(old[1,0]-old[0,0])
                    intercept=old[0,1]-slope*old[0,0]
                    if max(abs((slope-wall['slope'])*x+intercept-wall['intercept_m']) for x in (lo,hi))>.015:
                        consistent=False;break
            except (ValueError,MotionUnavailable):consistent=False
        if fresh:
            self.start_confirmations=self.start_confirmations+1 if consistent else 1
            self.start_walls={}
            for side,wall in evidence['walls'].items():
                x=np.asarray(wall['observed_x_m'])
                self.start_walls[side]=np.c_[x,wall['slope']*x+wall['intercept_m']]
            self.start_reference_s=scene.t_ref_s
        elif not consistent:
            self.start_confirmations=0
        return self.start_confirmations>=2,evidence

    def _rejoin_band(self,scene,fit):
        """현재 옆면에서 중앙 방향 띠의 양의 관측. 뒤차 부재/미래 경로 인증 아님."""
        slope,intercept,_,_=fit
        inward=-1. if self.side=='left' else 1.
        offset=abs(intercept)/math.sqrt(1+slope*slope)
        x=np.arange(-self.config.length_m/2,self.config.length_m/2+.401,.02)
        y=inward*np.arange(self.config.width_m/2+.015,offset+self.config.width_m/2+.026,.02)
        if not len(y):
            return dict(supported=False,reason='no_lateral_return_band',samples=0,unobserved=0)
        q=np.stack(np.meshgrid(x,y,indexing='ij'),axis=-1).reshape(-1,2)
        pad=mx.ray_padding(scene,lateral_config(self.config),.5)
        supported=depth_support(scene,q,pad)
        # 복귀 시도 신호와 실제 차체 이동 허가를 구별한다. 후반 띠의 가림은
        # 그대로 기록하고, 전반 띠의 양의 관측+전체 띠의 현재 점유 부재로
        # 후보만 만든다. 후보 명령의 뒤 모서리는 기존 이동 영역에서 다시 검사한다.
        front=q[:,0]>=-1e-9
        points=np.asarray(scene.occupied_endpoints_xy_m,dtype=float).reshape(-1,2)
        occupied_pad=per_ray_padding(scene,.025+self.config.measurement_padding_m,
                                     self.config.object_motion_bound_mps)
        occupied=(points[:,0]+occupied_pad>=x.min())&(points[:,0]-occupied_pad<=x.max())&(
            points[:,1]+occupied_pad>=y.min())&(points[:,1]-occupied_pad<=y.max())
        return dict(supported=bool(np.all(supported[front])) and not bool(np.any(occupied)),
            reason='current_inner_front_band_candidate_only',
            samples=len(q),unobserved=int(np.sum(~supported)),
            front_unobserved=int(np.sum(~supported[front])),current_occupied_returns=int(np.sum(occupied)),
            rear_unobserved=int(np.sum(~supported[~front])),
            behind_vehicle_not_certified=True,actual_motion_still_requires_current_support=True)

    def evaluate(self, scene, history, speed, previous_steering, dt):
        now, source = scene.t_ref_s, scene.source_first_s
        invalid = (not source_valid(scene) or not all(isinstance(v,(int,float)) and
            not isinstance(v,bool) and math.isfinite(v) for v in (speed,previous_steering,dt))
            or not 0 <= speed <= self.config.max_speed_mps+1e-12 or not 0 < dt <= .2
            or abs(previous_steering)>self.config.max_steering_rad)
        changed = ((self.epoch is not None and self.epoch != scene.source_epoch)
            or (self.reference_s is not None and now < self.reference_s)
            or (self.last_source is not None and source is not None and source < self.last_source))
        if invalid or changed:
            self.reset()
            return self._hold('entry_guide_input_reset')
        fresh=self.last_source is None or source>self.last_source
        self.epoch, self.last_source = scene.source_epoch, source
        if self.path is None:
            if self.stationary_outer_boundary:
                ready,evidence=self._confirm_initial_boundaries(scene,history,fresh)
                if not ready:
                    return self._hold('entry_initial_distinct_boundaries_wait',reference=evidence,
                        boundary_confirmations=self.start_confirmations)
            result = self.intent.select(scene, _compare_entries(scene,now,speed,previous_steering,dt,
                config=self.config,entry=self.entry,stationary_boundaries=self.stationary_outer_boundary))
            chosen=result['selected']
            if chosen is not None and chosen['offset_m'] != 0.:
                row=next(r for r in result['candidates'] if r.get('eligible') and
                    r['offset_m']==chosen['offset_m'] and r['transition_m']==chosen['transition_m'])
                if not row['transition_reaches_offset']:
                    return self._hold('entry_guide_complete_transition_unavailable')
                self.path=np.asarray(row['path_points'],dtype=float)
                self.offset,self.transition=chosen['offset_m'],chosen['transition_m']
                self.side='left' if self.offset>0 else 'right'
                wall=result['reference']['walls'][self.side]
                x=np.asarray(wall['observed_x_m'])
                self.wall_line=np.c_[x,wall['slope']*x+wall['intercept_m']]
                self.reference_s=now
                self.wall_confirmations=2 if self.stationary_outer_boundary else 1
            return {**result,'guide_mode':'new_observed_entry', 'guide_permission_remembered':False}
        try:
            path=transport_points(self.path,history,self.reference_s,now)
            remembered_wall=transport_points(self.wall_line,history,self.reference_s,now)
        except (ValueError,MotionUnavailable) as error:
            self.reset()
            return self._hold('entry_guide_motion_unavailable',error=str(error))
        # 이송은 원점을 갱신할 뿐 새 관측 시각/허가를 만들지 않는다.
        self.path,self.wall_line,self.reference_s=path,remembered_wall,now
        fit,wall=(tracked_straight_wall(scene,self.config,self.side,remembered_wall) if self.tracked_wall_fit
                  else observed_straight_wall(scene,self.config,self.side))
        if fit is None:
            self.wall_confirmations=0
            return self._hold('entry_guide_current_outer_wall_unavailable',reference=wall)
        slope,intercept,lower,upper=fit
        if np.ptp(remembered_wall[:,0]) < .25:
            return self._hold('entry_guide_wall_direction_lost')
        old_slope=(remembered_wall[1,1]-remembered_wall[0,1])/(remembered_wall[1,0]-remembered_wall[0,0])
        old_intercept=remembered_wall[0,1]-old_slope*remembered_wall[0,0]
        lo=max(lower,float(np.min(remembered_wall[:,0])))
        hi=min(upper,float(np.max(remembered_wall[:,0])))
        disagreement=max(abs((old_slope-slope)*x+old_intercept-intercept) for x in (lo,hi))
        if hi-max(lo,0.) < .25 or disagreement>self.config.boundary_agreement_m:
            self.wall_confirmations=0
            return self._hold('entry_guide_current_wall_disagrees',disagreement_m=disagreement)
        if fresh:
            self.wall_confirmations=self.wall_confirmations+1 if disagreement<=.015 else 0
        if lower > -self.config.wheelbase_m/2 or upper < .30:
            return self._hold('entry_guide_current_wall_too_short')
        correction=dict(applied=False,reason='disabled_or_reused_scan')
        if self.wall_reanchor and fresh:
            corrected,correction=reanchor_path_to_wall(path,remembered_wall,fit)
            if corrected is None:
                return self._hold('entry_guide_wall_reanchor_rejected',path_reanchor=correction)
            path=corrected
        rejoin=None
        if self.rejoin_enabled:
            offset_now=abs(intercept)/math.sqrt(1+slope*slope)
            aligned=abs(math.atan(slope))<=.08
            if self.phase=='entry' and aligned and abs(offset_now-abs(self.offset))<=.03:
                rejoin=self._rejoin_band(scene,fit)
                if not rejoin['supported']:self.rejoin_confirmations=0
                elif fresh:self.rejoin_confirmations+=1
                if self.rejoin_confirmations>=2:
                    proposed=entry_path(fit,0.,self.config,self.entry,self.entry.transitions_m[-1])
                    if proposed[-1,0]-proposed[0,0]>=self.entry.transitions_m[-1]:
                        path=proposed
                        self.offset=0.;self.transition=self.entry.transitions_m[-1]
                        self.phase='rejoin'
            elif self.phase=='rejoin':
                if aligned and offset_now<=.025:
                    if fresh:self.center_confirmations+=1
                    if self.center_confirmations>=2:self.phase='centered'
                else:self.center_confirmations=0
            elif self.phase=='entry':
                self.rejoin_confirmations=0
        if np.any(np.diff(path[:,0])<=0):
            return self._hold('entry_guide_forward_representation_lost')
        # 초기 전이를 끝낸 경로의 끝만 현재 관측한 같은 편이선으로 늘린다.
        # 새 목표를 현재 후륜축부터 다시 완만하게 시작하지 않는다.
        if upper>path[-1,0]+self.entry.path_step_m:
            x=np.linspace(path[-1,0],upper,math.ceil((upper-path[-1,0])/self.entry.path_step_m)+1)
            y=slope*x+intercept+self.offset*math.sqrt(1+slope*slope)
            difference=path[-1,1]-y[0]
            if abs(difference)>self.config.boundary_agreement_m:
                return self._hold('entry_guide_extension_disagrees',disagreement_m=abs(difference))
            y+=difference*np.clip(1-(x-x[0])/.30,0.,1.)
            path=np.vstack((path,np.c_[x[1:],y[1:]]))
        path=path[(path[:,0]>=max(lower,-.15)) & (path[:,0]<=upper)]
        if len(path)<5 or path[-1,0]-path[0,0]<.30:
            return self._hold('entry_guide_current_prefix_short')
        self.path=path
        # 같은 원시 스캔 재사용을 새 벽 기준 갱신으로 세지 않는다.
        if fresh or not self.wall_reanchor:
            self.wall_line=np.array([[lower,slope*lower+intercept],[upper,slope*upper+intercept]])
        reference=dict(reason='current_outer_wall_rechecked_for_remembered_guide',walls={self.side:wall},
            inner_points_still_occupied=True,initial_two_wall_agreement=True,
            wall_offset_assumption_m=self.config.wall_offset_m,disagreement_m=disagreement)
        result=_compare_entries(scene,now,speed,previous_steering,dt,config=self.config,entry=self.entry,
            guide=dict(reference=reference,path=path,offset_m=self.offset,transition_m=self.transition,
                outer_side=self.side,
                stationary_outer_boundary=self.stationary_outer_boundary and self.wall_confirmations>=2))
        return {**result,'intent_direction':self.intent.sign,'intent_remembered_permission':False,
            'guide_mode':'measured_motion_transport_current_outer_wall', 'guide_permission_remembered':False,
            'outer_wall_fresh_confirmations':self.wall_confirmations,
            'stationary_outer_boundary_requested':self.stationary_outer_boundary,
            'path_reanchor':correction,'entry_phase':self.phase,'rejoin_band':rejoin,'rejoin_confirmations':self.rejoin_confirmations,
            'center_confirmations':self.center_confirmations}
