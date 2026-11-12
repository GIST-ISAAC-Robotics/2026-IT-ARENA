"""현재 영역에도 이미 선택한 저속 공백 판정을 현재 입력으로 적용하는 후보."""
import math
import candidate_policy
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.local_follow import forward_planning_speed
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.follow_trace import thaw
from arena_autonomy.released_body_gap import check


def rebase(output,submitted,current,scene,history,cfg,*,require_current_region=False):
    result=candidate_policy.rebase(output,submitted,current,scene,history,cfg)
    if result is None or result['speed_mps']<=0:return result
    source=output[2]['normal'].get('path_source_s');age=current['reference_s']-source
    if age<cfg.memory_ttl_s and not require_current_region:return result
    speed=forward_planning_speed(cfg,current['speed_mps'])
    if not math.isfinite(speed) or speed<0:
        return {**result,'speed_mps':0.,'reason':'route_current_motion_invalid'}
    lateral=ps.lateral_config(cfg);selector=ps.SelectorConfig()
    region=ps.motion_region(scene,speed,result['speed_mps'],result['steering_rad'],
        current['previous_steering_rad'],current['dt_s'],cfg,lateral)
    reason='empty_current_region' if region['queries']==0 else ps.region_blocked(region,selector)
    bounded=None
    mode=result.get('rebased_selector_mode')
    enabled=((mode=='RELEASED' and cfg.released_body_gap_trial) or
             (mode=='GUARD' and cfg.traffic_recovery_trial and cfg.guard_observation_policy=='selected_r181'))
    if reason=='large_motion_region_unobserved' and enabled and 0<=speed<=.35 and 0<result['speed_mps']<=.35:
        # 저장 허가를 재사용하지 않는다. 현 광선·속도·조향·시각에서 기존 전체 검사를 재실행한다.
        raw=MeasurementScan(**thaw(current['scan']['measurements']))
        bounded=check(scene,raw,speed,result['speed_mps'],result['steering_rad'],
            current['previous_steering_rad'],current['dt_s'],cfg,lateral,selector)
        if bounded['accepted']:
            assert bounded['region']==region
            reason=None
    proof=dict(policy='current_route_geometry_v1',original_path_source_s=source,
        original_path_age_s=age,current_evidence_first_s=scene.source_first_s,
        current_reference_s=current['reference_s'],region=region,accepted=reason is None,
        unknown_filled=False,source_timestamp_renewed=False,old_occupancy_permission_reused=False,
        route_geometry_age_limit_s=.5)
    if bounded is not None:proof['bounded_current_risk']=bounded
    return {**result,'speed_mps':result['speed_mps'] if reason is None else 0.,
        'reason':result['reason'] if reason is None else 'route_current_'+reason,
        'current_route_proof':proof}
