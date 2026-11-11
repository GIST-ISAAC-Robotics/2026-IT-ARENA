"""같은 G1 명령의 기존 부분 관측 검사 연결. 다른 경로로 허용을 전파하지 않는다."""
import math
import numpy as np
from arena_autonomy.observation_support import source_valid
from arena_autonomy.recovery_geometry import sweep_rectangles
from arena_autonomy.rear_fringe_trial import _bounded_nonreturn
from arena_autonomy.scan_input import scan_angles, scan_cell_widths, scan_max_gap


def scoped_forward_width(scene, raw_scan, proof):
    """v4의 전방/측방 합산 범위를 현재 원천 광선으로 재검사한다.

    전방에 닿는 연속 공백은 전체를 센다. source/현재 방향 중 하나라도
    전방이면 포함하며, 따로 떨어진 측방 공백만 전방 1.5도 예산에서 뺀다.
    실제 질의의 이웃 반환/점유/원시 +inf 검사는 뒤에서 다시 수행한다.
    """
    fields=('forward_gap_deg','forward_gap_lidar_yaw_rad')
    if (raw_scan is None or any(type(proof.get(k)) not in (int,float)
            or not math.isfinite(proof[k]) for k in fields)):
        return False
    angles=(scan_angles(raw_scan)+proof['forward_gap_lidar_yaw_rad']+math.pi)%math.tau-math.pi
    if len(angles)!=len(scene.rays):return False
    cells=scan_cell_widths(raw_scan)
    step=scan_max_gap(raw_scan)
    order=np.argsort(angles)
    missing=np.asarray([r.status=='unknown' for r in scene.rays])[order]
    ids=np.flatnonzero(missing)
    if not len(ids):return False
    current=np.asarray([math.atan2(r.direction_xy[1],r.direction_xy[0]) for r in scene.rays])
    unknown=order[ids]
    if (np.any(abs(angles[unknown])>math.radians(100)+1e-6)
            or np.any(abs(current[unknown])>math.radians(100)+1e-6)):
        return False
    runs=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1)
    central=(abs(angles)<=math.radians(30)+step/2)|(abs(current)<=math.radians(30)+step/2)
    forward=sum(float(sum(cells[order[run]])) for run in runs if np.any(central[order[run]]))
    total=float(sum(cells[unknown]))
    return (0 < proof['forward_gap_deg'] <= proof['total_gap_deg'] <= 6.+1e-7
            and proof['forward_gap_deg'] <= 1.5+1e-7
            and 0 < forward <= math.radians(proof['forward_gap_deg'])+1e-7
            and total <= math.radians(proof['total_gap_deg'])+1e-7)

def scope_check(scene, queries, length, width, proof, command, raw_scan=None):
    """과거 통과 이력 대신, 이번 센서/명령의 기존 검사를 사용한다."""
    if (not isinstance(proof,dict) or proof.get('allowed') is not True
            or proof.get('reason') not in ('bounded_side_missing','bounded_lateral_metric_missing','bounded_forward_missing')):
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
    if proof['reason']=='bounded_forward_missing':
        scoped=proof.get('forward_gap_scope')=='whole_runs_source_or_reference_v1'
        fields=('total_gap_deg','forward_neighbor_required_m','minimum_gap_neighbor_m','history_distance_m','speed_cap_mps')
        if any(not isinstance(proof.get(k),(int,float)) or isinstance(proof.get(k),bool)
               or not math.isfinite(proof[k]) for k in fields):
            return 'forward_proof_missing'
        if (not 0 < command['target'] <= proof['speed_cap_mps'] <= .2
                or not 0 < proof['total_gap_deg'] <= (6. if scoped else 1.5)+1e-7
                or proof['forward_neighbor_required_m'] < max(1.,proof['history_distance_m'])
                or proof['minimum_gap_neighbor_m'] < proof['forward_neighbor_required_m']):
            return 'forward_proof_outside_scope'
        if scoped:
            if not scoped_forward_width(scene,raw_scan,proof):
                return 'unknown_outside_forward_proof'
        elif (any(abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))>math.radians(100) for r in unknown)
                or len(unknown)*2*checked_step > math.radians(1.5)+1e-7):
            return 'unknown_outside_forward_proof'
    elif any(not math.radians(30)+checked_step<abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))<=math.radians(100)
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
    reason=scope_check(scene,q,length,width,proof,command,raw_scan)
    if reason is not None:
        if reason != 'unknown_outside_side_scope':
            return {**result, 'reason':reason}
        # 측방 증거는 전방 결측을 허가하지 않는다. 후방 결측도 실제 이동
        # 영역에 필요하면 허가하지 않는다. 다만 필요하지 않은 후방 광선까지
        # 정면 회전의 같은-경로 증거를 무효화하는 데 쓰지 않는다.
        half_step=math.pi/len(scene.rays)
        unknown=[r for r in scene.rays if r.status=='unknown']
        if any(abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))<=math.radians(30)+half_step
               for r in unknown):
            return {**result, 'reason':reason}
        checked=_bounded_nonreturn(scene,raw_scan,q,pad,result)
        if not checked['accepted']:
            return {**result, 'reason':reason}
        needed={i for gap in checked['gaps'] for i in range(gap['first'],gap['last']+1)}
        if not needed or any(not math.radians(30)+half_step<abs(math.atan2(
                scene.rays[i].direction_xy[1],scene.rays[i].direction_xy[0]))<=math.radians(100)
                for i in needed):
            return {**result, 'reason':reason}
        # 모든 이동 질의가 실제 양옆 반환/원시 +inf/팽창/좁은 각도 검사를
        # 통과했다. 범위 밖 광선을 자유로 바꾸거나 해당 방향의 이동을 허가하지 않는다.
        return {**checked,'reason':'bounded_same_path_nonreturn_risk',
            'query_local_side_scope':True,'unqueried_rear_unknown_count':sum(
                abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))>math.radians(100) for r in unknown)}
    if proof['reason']=='bounded_forward_missing':
        # 무반환 셀 전체1.5도는 기존 G1 값이다. 이웃 반환 사이에는 한 셀 폭이 더 붙는다.
        step=(scan_max_gap(raw_scan) if proof.get('forward_gap_scope')=='whole_runs_source_or_reference_v1'
              else 2*math.pi/len(scene.rays))
        result.update(maximum_gap_rad=math.radians(1.5)+step,
            minimum_neighbor_range_m=max(1.,proof['forward_neighbor_required_m']),
            proof_scope='existing_forward_proof_same_or_smaller_sweep')
    answer=_bounded_nonreturn(scene,raw_scan,q,pad,result)
    if answer['accepted']:
        answer['reason']='bounded_same_path_nonreturn_risk'
    return answer
