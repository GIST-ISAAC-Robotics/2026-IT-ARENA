"""같은 G1 명령의 좁은 측방 무반환 시험. 다른 경로로 허용을 전파하지 않는다."""
import math
import numpy as np
from arena_autonomy.observation_support import source_valid
from arena_autonomy.recovery_geometry import sweep_rectangles
from arena_autonomy.rear_fringe_trial import _bounded_nonreturn

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
    # 같은 명령의 bounded_side_missing은 정면30도 밖의 좁은 공백이다.
    # 종전60도 보조 조건은 이 증거보다 좁아서 코너의54도 결측을 다시 막았다.
    # 광선 셀 반폭까지 정면 경계에서 제외한다. 넓은 측방 결측 후보(v4)의
    #60도 조건은 원래 proof 평가에서 유지하며 여기서 다시 완화하지 않는다.
    checked_step=math.pi/len(scene.rays)
    unknown=[r for r in scene.rays if r.status=='unknown']
    if any(not math.radians(30)+checked_step<abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))<=math.radians(100)
           for r in unknown):
        return 'unknown_outside_side_scope'
    return None


def bounded_same_path_gap(scene, raw_scan, queries, padding, length, width, proof, command):
    result = dict(accepted=False, reason='invalid_input', unknown_filled=False,
        residual_hidden_object_risk=True, maximum_exterior_m=None,
        maximum_gap_rad=math.radians(3), minimum_neighbor_range_m=.6)
    if raw_scan is None:
        return {**result, 'reason':'raw_scan_required'}
    q = np.asarray(queries, dtype=float).reshape(-1,2)
    pad = np.asarray(padding, dtype=float)
    if not len(q) or not np.all(np.isfinite(q)) or not source_valid(scene):
        return {**result, 'reason':'invalid_scene_or_queries'}
    if pad.shape != (len(scene.rays),) or not np.all(np.isfinite(pad)) or np.any(pad<0):
        return {**result, 'reason':'invalid_padding'}
    result['queries']=len(q)
    reason=scope_check(scene,q,length,width,proof,command)
    if reason is not None:
        return {**result, 'reason':reason}
    answer=_bounded_nonreturn(scene,raw_scan,q,pad,result)
    if answer['accepted']:
        answer['reason']='bounded_same_path_nonreturn_risk'
    return answer
