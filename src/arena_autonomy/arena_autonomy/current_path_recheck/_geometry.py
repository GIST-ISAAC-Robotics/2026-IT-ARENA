"""현재 입력의 경로 재검사. 명시 선택용 순수 함수이며 상태 전이/발행은 없다.

원 함수: 2026-11-11/follow_rebase_r126_v1/rebase.py
이관/동등성 근거: current_recheck_module_r176_v1.
"""
import math
import numpy as np
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_follow import pursue_path
from arena_autonomy.local_path import braking_speed, swept_limit
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.observation_support import source_valid
from arena_autonomy.scene_motion import transport_points


def finite(value):
    return type(value) in (int,float) and math.isfinite(value)


def rebase(output, submitted, current, scene, history, cfg):
    """None은 적용 대상 아님, dict는 새 현재 후보(0 포함). 원 명령 재발행 아님."""
    speed, steering, detail = output
    normal=detail.get('normal') or {}
    if (not finite(speed) or speed<=0 or detail.get('handoff_state')!='FOLLOW'
            or normal.get('reason')!='local_follow'
            or normal.get('selector_mode')!='SOLO'
            or normal.get('selector_choice')!='baseline'):
        return None
    now=current['reference_s']; origin=normal.get('path_source_s')
    original_ref=submitted['reference_s']
    def stop(reason):
        return dict(speed_mps=0.,steering_rad=current['previous_steering_rad'],reason=reason,
            reference_s=now,path_source_s=origin,original_reference_s=original_ref,
            independent_guard_required=True,runtime_authorized=False)

    if (not all(finite(v) for v in (now,origin,original_ref))
            or not 0<=origin<=original_ref<=now or now-origin>=cfg.memory_ttl_s):
        return stop('rebase_path_source_expired_or_invalid')
    raw=current['scan'];old=submitted['scan']
    if (raw.get('representation')!='native_measurements_v1'
            or old.get('representation')!='native_measurements_v1'):
        return stop('rebase_native_input_required')
    if (current['epoch']!=submitted['epoch'] or current['side']!=submitted['side']
            or raw['measurements']['source_epoch']!=old['measurements']['source_epoch']):
        return stop('rebase_epoch_or_side_changed')
    if (current['previous_steering_rad']!=submitted['previous_steering_rad']
            or not current['timing_verified'] or not source_valid(scene)
            or abs(scene.t_ref_s-now)>1e-9):
        return stop('rebase_current_input_invalid')
    dt=current['dt_s']; measured=current['speed_mps']; previous=current['previous_steering_rad']
    if not all(finite(v) for v in (dt,measured,previous)) or not 0<dt<=.1:
        return stop('rebase_current_motion_invalid')
    try:
        path=np.asarray(normal.get('path_points',[]),dtype=float).reshape(-1,2)
        if len(path)<6 or not np.all(np.isfinite(path)):
            return stop('rebase_path_invalid')
        moved=transport_points(path,history,original_ref,now)
        if np.max(np.linalg.norm(moved-path,axis=1))>cfg.memory_motion_limit_m:
            return stop('rebase_path_motion_exceeded')
        scan_age=now-scene.source_first_s
        cap,desired,remaining=pursue_path(moved,abs(measured),cfg,max(scan_age,now-origin),speed_cap=speed)
        proposed=float(np.clip(desired,previous-3*dt,previous+3*dt))
        points=np.asarray(scene.occupied_endpoints_xy_m,dtype=float).reshape(-1,2)
        padding=per_ray_padding(scene,.025+cfg.measurement_padding_m,cfg.object_motion_bound_mps)
        clearance=min(swept_limit(points,angle,cfg.wheelbase_m,horizon=2.5,margin=padding,
            length=cfg.length_m,width=cfg.width_m) for angle in (proposed,previous))
        available=max(0.,clearance-cfg.standstill_gap_m)
        target=min(speed,cap,braking_speed(available,scan_age),available/cfg.headway_s)
        if clearance<=cfg.hold_clearance_m or target<=0:
            target=0.
        return dict(speed_mps=float(target),steering_rad=proposed,
            reason='rebase_current_follow' if target>0 else 'rebase_current_gap_hold',
            reference_s=now,path_source_s=origin,original_reference_s=original_ref,
            source_first_s=scene.source_first_s,path_age_s=now-origin,
            original_speed_cap_mps=speed,clearance_m=clearance,
            observed_path_ahead_m=remaining,path_points=moved.tolist(),
            independent_guard_required=True,runtime_authorized=False,
            observation_policy_scope='existing independent partial/structure policy remains required')
    except (MotionUnavailable,ValueError) as error:
        return {**stop('rebase_geometry_unavailable'),'error':str(error)}
