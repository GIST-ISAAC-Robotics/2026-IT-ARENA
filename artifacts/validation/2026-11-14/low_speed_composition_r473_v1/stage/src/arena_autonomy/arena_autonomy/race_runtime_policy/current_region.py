"""현재 영역에도 이미 선택한 저속 공백 판정을 현재 입력으로 적용하는 후보."""
import math
from . import observation_policy as candidate_policy
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.local_follow import forward_planning_speed
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.follow_trace import thaw
from arena_autonomy.released_body_gap import check
from arena_autonomy.low_speed_observation import within_scope, encoder_headroom


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
    context={}
    if (result.get('reason')=='rebase_current_released' and body.get('accepted') is True
            and body.get('measured_speed_mps')==speed and body.get('target_mps')==result['speed_mps']):
        region=body['region']
    elif result.get('reason')=='rebase_current_guard' and 'current_gap_proof' in result:
        region=result['region']
    else:
        region=ps.motion_region(scene,speed,result['speed_mps'],result['steering_rad'],
            current['previous_steering_rad'],current['dt_s'],cfg,lateral,context=context)
    reason='empty_current_region' if region['queries']==0 else ps.region_blocked(region,selector)
    structure=None
    normal=output[2].get('normal') or {}
    structure_mode=(normal.get('selector_mode')=='SOLO' or
        (normal.get('selector_mode')=='RELEASED' and cfg.traffic_recovery_trial
         and result.get('reason')=='rebase_current_released'
         and result.get('current_side_presence')=={'left':False,'right':False}))
    if (reason in ('large_motion_masked_toward_left','large_motion_masked_toward_right')
            and cfg.known_structure_policy=='structure_aware_v1'
            and output[2].get('handoff_state')=='FOLLOW'
            and structure_mode and normal.get('selector_choice')=='baseline'
            and region['unobserved']==0 and region['masked_front_samples']==0 and context):
        from arena_autonomy.known_structure import current_mask_shadow, parse_sectors
        from arena_autonomy.crop_edge_selection import SIMULATION_SCOPE, ESTIMATED_SCOPE
        from .profile_evidence import profile_evidence
        raw=MeasurementScan(**thaw(current['scan']['measurements']))
        sectors=parse_sectors(cfg.known_structure_sectors_json)
        try:
            policy=current.get('timing_policy','verified_only')
            scope=ESTIMATED_SCOPE if policy=='host_estimated_v1' else SIMULATION_SCOPE
            profile_evidence(scene,raw,sectors,current['rear_half_angle_deg'],current['epoch'],
                current['timing_verified'],observation_scope=scope,timing_policy=policy)
            structure=current_mask_shadow(scene,raw,context['queries'][context['masked']],
                                          context['padding'],sectors)
        except (ValueError,TypeError,AttributeError,OverflowError) as error:
            structure=dict(accepted=False,reason='current_structure_profile_invalid',error=str(error),
                           unknown_filled=False)
        if structure['accepted']:
            reason=None
    bounded=None
    # 위 RELEASED current가 같은 현재 입력에서 합성 검사한 결과만 재사용한다.
    if (reason in ('large_motion_masked_toward_left','large_motion_masked_toward_right')
            and body.get('accepted') is True and body.get('region') == region
            and body.get('measured_speed_mps') == speed and body.get('target_mps') == result['speed_mps']
            and (body.get('structure_proof') or {}).get('accepted') is True):
        bounded=body
        reason=None
    mode=result.get('rebased_selector_mode')
    enabled=((mode=='RELEASED' and cfg.released_body_gap_trial) or
             (mode=='GUARD' and cfg.traffic_recovery_trial and cfg.guard_observation_policy=='selected_r181'))
    if (reason=='large_motion_region_unobserved' and enabled
            and within_scope(speed,result['speed_mps'],.35,encoder_headroom(cfg))):
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
    if structure is not None:proof['current_structure_risk']=structure
    return {**result,'speed_mps':result['speed_mps'] if reason is None else 0.,
        'reason':result['reason'] if reason is None else 'route_current_'+reason,
        'current_route_proof':proof}
