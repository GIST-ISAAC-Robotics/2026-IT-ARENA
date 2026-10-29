"""진단 전용: 같은 G1 감속 외곽 안의 좁은 측방 무반환 위험을 재검사한다."""
import inspect
import math
import types
import numpy as np
from arena_autonomy import rear_fringe_trial
from probe_joint_missing_sweep import sweep_rectangles


def scope_check(scene, queries, length, width, proof, command):
    """과거 통과 이력 대신, 이번 센서/명령의 기존 검사를 사용한다."""
    if (not isinstance(proof,dict) or proof.get('allowed') is not True
            or proof.get('reason') not in ('bounded_side_missing','bounded_lateral_metric_missing')):
        return 'partial_not_allowed'
    checked=proof.get('timed_missing_diagnostic') or {}
    keys=('reference_s','source_first_s','checked_speed_mps','checked_steering_rad',
          'checked_previous_steering_rad','checked_age_s','wheelbase_m','length_m','width_m')
    ckeys=('now','speed','target','steering','previous')
    if (any(k not in checked or not math.isfinite(checked[k]) for k in keys)
            or any(k not in command or not math.isfinite(command[k]) for k in ckeys)):
        return 'checked_motion_missing'
    if (abs(checked['reference_s']-scene.t_ref_s)>1e-9
            or abs(command['now']-scene.t_ref_s)>1e-9
            or abs(checked['source_first_s']-scene.source_first_s)>1e-9
            or abs(checked['length_m']-length)>1e-9 or abs(checked['width_m']-width)>1e-9):
        return 'checked_scene_mismatch'
    if (not 0 < command['target'] <= .35 or command['speed'] < 0
            or max(command['target'],command['speed'])>checked['checked_speed_mps']+1e-9
            or abs(command['steering']-checked['checked_steering_rad'])>1e-9
            or abs(command['previous']-checked['checked_previous_steering_rad'])>1e-9):
        return 'different_or_larger_motion'
    inside=np.zeros(len(queries),dtype=bool)
    for center,axes,half in sweep_rectangles(checked['checked_speed_mps'],checked['checked_age_s'],
            checked['checked_steering_rad'],checked['checked_previous_steering_rad'],
            checked['wheelbase_m'],length,width):
        local=(queries-center)@axes.T
        inside|=np.all(abs(local)<=half+1e-12,axis=1)
    if not np.all(inside):return 'outside_checked_sweep'
    # 현재 차체 기준 정면/후방 결측은 이 후보로 확대하지 않는다.
    unknown=[r for r in scene.rays if r.status=='unknown']
    if any(not math.radians(60)<=abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))<=math.radians(100)
           for r in unknown):
        return 'unknown_outside_side_scope'
    return None


def candidate():
    """기존 광선/점유 검사를 보존하고 후반1 mm 영역 조건만 동일 경로 계약으로 교체."""
    source=inspect.getsource(rear_fringe_trial)
    source=source.replace('def bounded_rear_fringe(scene, raw_scan, queries, padding, length, width):',
        'def bounded_same_path_gap(scene, raw_scan, queries, padding, length, width, proof, command):')
    start=source.index('    # 전방/뒤 범퍼를 넘는 진행 영역은 제외.')
    end=source.index('    try:',start)
    source=source[:start]+'''    result['maximum_exterior_m'] = None
    result['queries'] = len(q)
    reason = scope_check(scene, q, length, width, proof, command)
    if reason is not None:
        return finish(reason)
'''+source[end:]
    source=source.replace("'bounded_rear_nonreturn_risk'","'bounded_same_path_nonreturn_risk'")
    module=types.ModuleType('same_path_gap_diagnostic');module.__dict__['scope_check']=scope_check
    exec(compile(source,rear_fringe_trial.__file__,'exec'),module.__dict__)
    return module.bounded_same_path_gap,source
