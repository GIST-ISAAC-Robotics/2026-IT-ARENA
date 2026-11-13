"""동일 crop 성분 재검사와 거의 정지한 자기 가림의 단일 저속 재검사. 기본/실물 미선택."""
import math
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.follow_trace import thaw
from arena_autonomy.measurement_scan import MeasurementScan
from .crop_current_scope import same_scan_scope
from .crop_current_proof import check
from arena_autonomy.crop_edge_selection import SIMULATION_SCOPE, ESTIMATED_SCOPE, validate_observation_scope

DIAGNOSTIC='current_crop_component_retry'


def try_from_current_result(before,data,scene,history,cfg,*,registered_environment,core_rebase,
                            observation_scope=SIMULATION_SCOPE):
    validate_observation_scope(observation_scope)
    if observation_scope == ESTIMATED_SCOPE:
        # 원0의 관측 보완에 단일 목표만 검사한다. 정상 양수/실측속도를 바꾸지 않는다.
        return _try_once(before,data,scene,history,cfg,registered_environment=registered_environment,
            core_rebase=core_rebase,target_cap=.1,observation_scope=observation_scope)
    first=_try_once(before,data,scene,history,cfg,
        registered_environment=registered_environment,core_rebase=core_rebase,target_cap=.2)
    if first is None or first is before or first['speed_mps']!=0:return first
    diagnostic=first.get(DIAGNOSTIC) or {}
    region=diagnostic.get('region') or {}
    # 이미 통과한 양수 판단과 crop 관측 자체의 실패에는 새 계산을 추가하지 않는다.
    # 측정 속도는 그대로 유지하고, 자기 후측방 가림만 남은 거의 정지 상태에 한정한다.
    if (diagnostic.get('applied') is not False
            or diagnostic.get('reason')!='current_crop_full_region_rejected'
            or (diagnostic.get('proof') or {}).get('accepted') is not True
            or diagnostic.get('blocked') not in
               ('large_motion_masked_toward_left','large_motion_masked_toward_right')
            or region.get('queries',0)<=0 or region.get('masked_front_samples')!=0
            or not 0<=diagnostic['measured_speed_mps']<=.15
            or diagnostic['requested_target_mps']<=.1):
        return first
    # 첫 proof를 재사용하지 않는다. 현재 점유/제동·전체 이동 외곽·crop 관측을 모두 다시 검사한다.
    second=_try_once(before,data,scene,history,cfg,
        registered_environment=registered_environment,core_rebase=core_rebase,target_cap=.1)
    retry=second.get(DIAGNOSTIC) or {}
    accepted=retry.get('applied') is True
    combined={**(retry if accepted else diagnostic),
        'attempted_targets':2,
        'final_full_checks':diagnostic.get('final_full_checks',0)+retry.get('final_full_checks',0),
        'crawl_retry':dict(applied=accepted,cap_mps=.1,maximum_measured_speed_mps=.15,
            previous_target_mps=diagnostic['requested_target_mps'],
            previous_blocked=diagnostic['blocked'],
            previous_masked_tail_extent_m=region.get('masked_tail_extent_m'),
            result=retry)}
    if not accepted:return {**first,DIAGNOSTIC:combined}
    route={**second['current_route_proof'],'bounded_current_risk':combined}
    return {**second,'current_route_proof':route,DIAGNOSTIC:combined}


def _try_once(before,data,scene,history,cfg,*,registered_environment,core_rebase,target_cap,
              observation_scope=SIMULATION_SCOPE):
    if (before is None or before['speed_mps']!=0
            or before.get('reason')!='route_current_large_motion_region_unobserved'
            or not same_scan_scope(data,cfg,registered_environment=registered_environment,
                                   observation_scope=observation_scope)):
        return before
    output,current=data['output'],data['current']
    measured,previous,dt=current['speed_mps'],current['previous_steering_rad'],current['dt_s']
    values=(output[0],output[1],measured,previous,dt,before['steering_rad'])
    maximum_measured = .15 if observation_scope == ESTIMATED_SCOPE else .6
    if (not all(type(v) in (float,int) and math.isfinite(v) for v in values)
            or not 0<=measured<=maximum_measured or not 0<dt<=.1 or output[0]<=0
            or max(abs(output[1]),abs(previous),abs(before['steering_rad']))>cfg.max_steering_rad):
        return before
    target=min(output[0],target_cap)
    diagnostic=dict(applied=False,original_target_mps=output[0],trial_target_cap_mps=target_cap,
        requested_target_mps=target,measured_speed_mps=measured,maximum_measured_speed_mps=maximum_measured,
        attempted_targets=1,final_full_checks=0,unknown_filled=False,source_timestamp_renewed=False,
        old_occupancy_permission_reused=False,state_transition=False,continuous_sweep_certified=False,
        residual_hidden_object_risk=True,scope=('estimated_native_same_scan_crop_only'
            if observation_scope==ESTIMATED_SCOPE else 'verified_native_simulation_same_scan_crop_only'))
    if observation_scope==ESTIMATED_SCOPE:
        diagnostic.update(observation_input_scope=observation_scope,
            timing_verified=current['timing_verified'],timing_policy=current['timing_policy'],
            time_basis=current['scan']['measurements']['time_basis'])
    def reject(reason):return {**before,DIAGNOSTIC:{**diagnostic,'reason':reason}}
    core=core_rebase((target,output[1],output[2]),data['submitted'],current,scene,history,cfg)
    diagnostic['core_result']=(None if core is None else {key:core.get(key) for key in
        ('speed_mps','steering_rad','reason','reference_s','clearance_m')})
    if (core is None or type(core.get('speed_mps')) not in (int,float) or not math.isfinite(core['speed_mps'])
            or not 0<core['speed_mps']<=target or core.get('reason')!='rebase_current_follow'
            or any(core.get(k)!=before.get(k) for k in
                   ('steering_rad','reference_s','path_source_s','original_reference_s','path_points'))):
        return reject('current_crop_core_rejected')
    context={}
    region=ps.motion_region(scene,measured,core['speed_mps'],core['steering_rad'],previous,dt,cfg,
                            ps.lateral_config(cfg),context=context)
    diagnostic.update(final_full_checks=1,checked_target_mps=core['speed_mps'],region=region)
    proof=dict(accepted=False,reason='no_unknown_queries',unknown_filled=False)
    if region['unobserved'] and context:
        raw=MeasurementScan(**thaw(current['scan']['measurements']))
        queries=context['queries'][~context['free'] & ~context['masked']]
        component=output[2]['normal']['partial_observation']['crop_edge_component']
        proof=check(scene,raw,queries,context['padding'],component,
                    all_queries=context['queries'],motion_epoch=current['epoch'],
                    **(dict(observation_scope=observation_scope,timing_verified=current['timing_verified'],
                            timing_policy=current['timing_policy']) if observation_scope==ESTIMATED_SCOPE else {}))
    blocked=ps.region_blocked(region,ps.SelectorConfig(),bounded_unknown_risk=proof['accepted'])
    diagnostic.update(proof=proof,blocked=blocked)
    if region['queries']<=0 or blocked is not None or (region['unobserved'] and not proof['accepted']):
        return reject('current_crop_full_region_rejected')
    diagnostic.update(applied=True,reason='single_current_crop_reach_accepted',current_full_recheck=True)
    route={**before['current_route_proof'],'region':region,'accepted':True,'bounded_current_risk':diagnostic}
    return {**core,'current_route_proof':route,DIAGNOSTIC:diagnostic}
