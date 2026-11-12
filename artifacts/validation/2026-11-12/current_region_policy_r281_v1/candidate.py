"""현재 영역에도 이미 선택한 저속 공백 판정을 현재 입력으로 적용하는 후보."""
import math
import candidate_policy
from arena_autonomy import pass_release_trial as ps
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
    # 아래 두 반환형은 방금 candidate_policy.rebase가 동일 현재 scene, 실제
    # 속도, 최종 목표/조향, previous/dt로 만든 영역이다. 과거 output의 영역은
    # 읽지 않는다. 해당 경로가 없으면 이전과 똑같이 전체 영역을 계산한다.
    body=result.get('current_body_gap') or {}
    if (result.get('reason')=='rebase_current_released' and body.get('accepted') is True
            and body.get('measured_speed_mps')==speed and body.get('target_mps')==result['speed_mps']):
        region=body['region']
    elif result.get('reason')=='rebase_current_guard' and 'current_gap_proof' in result:
        region=result['region']
    else:
        region=ps.motion_region(scene,speed,result['speed_mps'],result['steering_rad'],
            current['previous_steering_rad'],current['dt_s'],cfg,lateral)
    reason='empty_current_region' if region['queries']==0 else ps.region_blocked(region,selector)
    bounded=None
    mode=result.get('rebased_selector_mode')
    enabled=((mode=='RELEASED' and cfg.released_body_gap_trial) or
             (mode=='GUARD' and cfg.traffic_recovery_trial and cfg.guard_observation_policy=='selected_r181'))
    if reason=='large_motion_region_unobserved' and enabled and 0<=speed<=.35 and 0<result['speed_mps']<=.35:
        # 같은 호출의 candidate_policy.rebase가 방금 현 입력으로 검사한 결과만 재사용한다.
        # output(과거 계획)에 들어 있던 허가나 다른 호출의 결과는 여기서 읽지 않는다.
        body=result.get('current_body_gap') or {}
        gap=result.get('current_gap_proof') or {}
        if (body.get('accepted') is True and body.get('region')==region
                and body.get('measured_speed_mps')==speed and body.get('target_mps')==result['speed_mps']):
            bounded=body
        elif (gap.get('accepted') is True and result.get('region')==region
                and gap.get('policy') in ('guard_low_speed_body_nonreturn_r136','guard_adjacent_structure_gap_r149')):
            bounded=dict(accepted=True,proof=gap,region=region,blocked=None,
                measured_speed_mps=speed,target_mps=result['speed_mps'],unknown_filled=False)
        else:
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
