"""과거 경로는 선호 기하로만 사용하고 현재 진입 영역을 새로 검사하는 후보."""
import math
import candidate_policy
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.local_follow import forward_planning_speed


def rebase(output, submitted, current, scene, history, cfg, *, require_current_region=False):
    result=candidate_policy.rebase(output,submitted,current,scene,history,cfg)
    if result is None or result['speed_mps']<=0:
        return result
    source=output[2]['normal'].get('path_source_s')
    age=current['reference_s']-source
    if age<cfg.memory_ttl_s and not require_current_region:
        return result
    # 과거 path_source/원 관측을 최신이라고 쓰지 않는다. 원 경로 수명보다
    # 한 제어 주기 오래된 선호 기하를 쓰려면 현재 자료만으로 새 검사를 통과해야 한다.
    speed=forward_planning_speed(cfg,current['speed_mps'])
    if not math.isfinite(speed) or speed<0:
        return {**result,'speed_mps':0.,'reason':'route_current_motion_invalid'}
    region=ps.motion_region(scene,speed,result['speed_mps'],result['steering_rad'],
        current['previous_steering_rad'],current['dt_s'],cfg,ps.lateral_config(cfg))
    reason=('empty_current_region' if region['queries']==0 else
            ps.region_blocked(region,ps.SelectorConfig()))
    proof=dict(policy='current_route_geometry_v1',original_path_source_s=source,
               original_path_age_s=age,current_evidence_first_s=scene.source_first_s,
               current_reference_s=current['reference_s'],region=region,
               accepted=reason is None,unknown_filled=False,source_timestamp_renewed=False,
               old_occupancy_permission_reused=False,route_geometry_age_limit_s=cfg.memory_ttl_s+.1)
    return {**result,'speed_mps':result['speed_mps'] if reason is None else 0.,
            'reason':result['reason'] if reason is None else 'route_current_'+reason,
            'current_route_proof':proof}
