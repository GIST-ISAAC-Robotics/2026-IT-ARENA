"""명시 선택형 현재 계산. 이전 결과를 받으며 기존 wrapper를 다시 호출하거나 발행하지 않는다."""
import math
from .crop_current_scope import partial_retry_scope
import time

from arena_autonomy import pass_release_trial as ps
from arena_autonomy.follow_trace import thaw
from arena_autonomy.measurement_scan import MeasurementScan
from .corner_budget import propose,elapsed

DIAGNOSTIC='current_corner_distance_budget'


def finite(value):
    return type(value) in (int,float) and math.isfinite(value)


def eligible(before,output,current,cfg,require_current_region,*,crop_current_scope=False):
    if before is None or before['speed_mps']>0:return False
    normal=output[2].get('normal') or {};partial=normal.get('partial_observation') or {}
    retry=before.get('current_relative_body_retry') or {};proof=retry.get('proof') or {}
    values=(output[0],output[1],current['speed_mps'],current['previous_steering_rad'],current['dt_s'],before['steering_rad'])
    return (require_current_region is True and all(finite(v) for v in values)
        and 0<=current['speed_mps']<=.15 and 0<current['dt_s']<=.1 and output[0]>.15
        and max(abs(output[1]),abs(current['previous_steering_rad']),abs(before['steering_rad']))<=cfg.max_steering_rad
        and output[2].get('handoff_state')=='FOLLOW' and normal.get('reason')=='local_follow'
        and normal.get('selector_mode')=='SOLO' and normal.get('selector_choice')=='baseline'
        and normal.get('boundary_from_memory') is False
        and partial_retry_scope(partial,crop_current_scope=crop_current_scope)
        and before.get('reason')=='route_current_large_motion_region_unobserved'
        and retry.get('applied') is False and retry.get('cap_mps')==.10
        and proof.get('reason')=='outside_small_body_side')


def try_from_current_result(before,output,submitted,current,scene,history,cfg,*,core_rebase,proof_callback,
                            require_current_region=False,costs=None,crop_current_scope=False):
    """계산된 기준을 그대로 받아 거리 제안 1회 + 최종 현재 검사 1회만 한다."""
    if not eligible(before,output,current,cfg,require_current_region,crop_current_scope=crop_current_scope):return before
    measured=current['speed_mps'];previous=current['previous_steering_rad'];dt=current['dt_s']
    proposal=propose(scene,measured,before['steering_rad'],previous,dt,cfg,costs=costs)
    diagnostic=dict(applied=False,proposal=proposal,original_reason=before['reason'],original_target_mps=output[0],
        measured_speed_mps=measured,unknown_filled=False,source_timestamp_renewed=False,
        state_transition=False,default_changed=False,additional_target_attempts=0,final_full_checks=0,
        continuous_sweep_certified=False,proposal_is_permission=False)
    def reject(reason):return {**before,DIAGNOSTIC:{**diagnostic,'reason':reason}}
    if not proposal['accepted']:return reject(proposal['reason'])
    target=proposal['proposed_target_mps']
    start=time.perf_counter_ns()
    core=core_rebase((target,output[1],output[2]),submitted,current,scene,history,cfg)
    elapsed(costs,'final_core_ms',start)
    diagnostic['additional_target_attempts']=1
    if (core is None or not finite(core.get('speed_mps')) or not 0<core['speed_mps']<=target
            or core.get('reason')!='rebase_current_follow'
            or any(core.get(k)!=before.get(k) for k in (
                'steering_rad','reference_s','path_source_s','original_reference_s','path_points'))):
        return reject('final_current_geometry_or_identity_rejected')
    # 다시 생성하는 최종 linspace 질의가 접두 표본의 부분집합이라고 가정하지 않는다.
    context={};start=time.perf_counter_ns()
    region=ps.motion_region(scene,measured,core['speed_mps'],core['steering_rad'],previous,dt,cfg,
        ps.lateral_config(cfg),context=context)
    elapsed(costs,'final_region_ms',start)
    diagnostic.update(final_full_checks=1,region=region,target_mps=core['speed_mps'])
    if region['distance_m']>proposal['allowed_distance_m']:
        return reject('final_distance_exceeds_budget')
    proof=dict(accepted=False,reason='no_unknown_context',unknown_filled=False)
    start=time.perf_counter_ns()
    if region['unobserved'] and context:
        raw=MeasurementScan(**thaw(current['scan']['measurements']))
        q=context['queries'][~context['free'] & ~context['masked']]
        proof=proof_callback(scene,raw,q,context['padding'],cfg.length_m,cfg.width_m,
            measured,core['speed_mps'])
    blocked=ps.region_blocked(region,ps.SelectorConfig(),bounded_unknown_risk=proof['accepted'])
    elapsed(costs,'final_gap_ms',start)
    diagnostic.update(proof=proof,blocked=blocked)
    if not (region['queries']>0 and blocked is None and (not region['unobserved'] or proof['accepted'])):
        return reject('final_current_region_rejected')
    diagnostic.update(applied=True,reason='single_current_distance_budget_accepted',current_full_recheck=True)
    route={**before['current_route_proof'],'region':region,'accepted':True,'bounded_current_risk':diagnostic}
    return {**core,'current_route_proof':route,DIAGNOSTIC:diagnostic}
