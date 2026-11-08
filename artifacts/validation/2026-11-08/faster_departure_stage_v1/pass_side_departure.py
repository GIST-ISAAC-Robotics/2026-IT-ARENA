"""빠른 추월의 이미 확보한 편이 경로에서 현재 바깥 측방 반환에 비접근하는 저속 전진."""
import math

import numpy as np

from arena_autonomy.bypass_entry_trial import entry_motion_region, lateral_config, region_blocked, SelectorConfig
from arena_autonomy.local_path import braking_speed
from arena_autonomy.moving_pass_wall_padding import current_outer_and_rear_padding
from arena_autonomy.parallel_lane_trial import lateral_excursion
from arena_autonomy.parallel_side_departure import limited_clearance


def outer_overlap(points, padding, side, wall, cfg):
    """겹친 모든 반환이 바깥쪽 실제 벽선과 맞는지 확인. 고정 물체라고 분류하지 않는다."""
    if (side not in ('left','right') or wall.get('reason')!='current_connected_straight_wall_fit'
            or wall.get('actual_return_count',0)<12 or not len(points)):
        return False
    a=wall['slope']; b=wall['intercept_m']; norm=math.hypot(1.,a)
    sign=1 if side=='left' else -1
    overlap=((abs(points[:,0])<=cfg.length_m/2+padding)
             &(abs(points[:,1])<=cfg.width_m/2+padding))
    selected=points[overlap]
    if not len(selected): return False
    return bool(np.all((selected[:,1]*sign>cfg.width_m/2+.025+cfg.measurement_padding_m)
                       &(abs((selected[:,1]-a*selected[:,0]-b)/norm-sign*cfg.wall_offset_m)<=.015)))


def compare_pass_side_departure(result, scene, config, speed, previous, dt, *, target_observation,
                                raw_scan, sectors, cap):
    """기존 경로가 점유로 거절된 경우에만. 현재 정면/옆차 및 기존 관측 거절은 보존."""
    if (result.get('selected') is not None or result.get('entry_phase')!='entry'
            or not result.get('stationary_outer_boundary_requested')
            or result.get('outer_wall_fresh_confirmations',0)<2
            or not target_observation.get('observed') or not target_observation.get('ahead_clear')
            or raw_scan is None or not sectors or len(raw_scan.ranges)!=len(scene.rays)):
        return result
    candidates=result.get('candidates',[])
    if len(candidates)!=1: return result
    row=candidates[0]; side=row.get('current_outer_wall')
    if (row.get('reason')!='known_occupied_command_hold' or row.get('eligible')
            or not row.get('offset_m') or row.get('command_clearance_m',1.)>1e-9
            or not all(math.isfinite(v) for v in (speed,previous,dt,cap))
            or speed<0 or not 0<dt<=.2 or cap<=0):
        return result
    wall=result.get('reference',{}).get('walls',{}).get(side,{})
    if wall.get('reason')!='current_connected_straight_wall_fit': return result
    pts=np.asarray(scene.occupied_endpoints_xy_m)
    pad,_=current_outer_and_rear_padding(scene,config,side,wall,raw_scan=raw_scan,sectors=sectors)
    if not outer_overlap(pts,pad,side,wall,config): return result
    angle=row['steering_rad']; age=scene.t_ref_s-scene.source_first_s
    clear,proof=limited_clearance(pts,(angle,previous),pad,speed,age,dt,config)
    reason=proof['reason']
    def deny(why):
        return {**result,'pass_side_departure':{**proof,'accepted':False,'decision_reason':why}}
    if clear<=0 or proof['measured_stopping_m']+.06>clear:
        return deny(reason if clear<=0 else 'measured_stop_outside_clearance')
    # 경로 길이·곡률 제한을 원래 pursue_path에서 다시 계산한다. 조향은 같은 실제 후보 그대로다.
    from arena_autonomy.local_follow import pursue_path
    from arena_autonomy.lidar_motion import MotionUnavailable
    try:
        normal_cap,_,_=pursue_path(np.asarray(row['path_points']), speed,config,age,speed_cap=cap)
    except (ValueError, MotionUnavailable):
        return deny('current_path_unavailable')
    target=min(cap,normal_cap,.35,braking_speed(clear,age))
    if target<=0 or lateral_excursion(max(speed,target),age,angle,previous,dt,config)>.01:
        return deny('lateral_excursion_or_speed')
    region=entry_motion_region(scene,speed,target,angle,previous,dt,config,lateral_config(config),pad)
    blocked=region_blocked(region,SelectorConfig())
    if blocked is not None: return deny(blocked)
    chosen=dict(offset_m=row['offset_m'],transition_m=row['transition_m'],speed_mps=target,
                steering_rad=angle,ranking='existing_offset_current_outer_nonapproaching')
    return {**result,'selected':chosen,'reason':'observed_outer_nonapproaching_departure',
            'pass_side_departure':{**proof,'accepted':True,'existing_entry_steering':angle,
                'original_clearance_m':row['command_clearance_m'],'lateral_cap_mps':.35,
                'current_target_ahead':True,'motion_region':region,
                'unknown_filled':False,'static_identity_verified':False}}
