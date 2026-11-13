"""기존 current0의 동일 crop 성분에만 한 목표를 재검사. 기본/실물 미선택."""
import math
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.follow_trace import thaw
from arena_autonomy.measurement_scan import MeasurementScan
from .crop_current_scope import same_scan_scope
from .crop_current_proof import check

DIAGNOSTIC='current_crop_component_retry'


def try_from_current_result(before,data,scene,history,cfg,*,registered_environment,core_rebase):
    if (before is None or before['speed_mps']!=0
            or before.get('reason')!='route_current_large_motion_region_unobserved'
            or not same_scan_scope(data,cfg,registered_environment=registered_environment)):
        return before
    output,current=data['output'],data['current']
    measured,previous,dt=current['speed_mps'],current['previous_steering_rad'],current['dt_s']
    values=(output[0],output[1],measured,previous,dt,before['steering_rad'])
    if (not all(type(v) in (float,int) and math.isfinite(v) for v in values)
            or not 0<=measured<=.6 or not 0<dt<=.1 or output[0]<=0
            or max(abs(output[1]),abs(previous),abs(before['steering_rad']))>cfg.max_steering_rad):
        return before
    target=min(output[0],.2)
    diagnostic=dict(applied=False,original_target_mps=output[0],trial_target_cap_mps=.2,
        requested_target_mps=target,measured_speed_mps=measured,maximum_measured_speed_mps=.6,
        attempted_targets=1,final_full_checks=0,unknown_filled=False,source_timestamp_renewed=False,
        old_occupancy_permission_reused=False,state_transition=False,continuous_sweep_certified=False,
        residual_hidden_object_risk=True,scope='verified_native_simulation_same_scan_crop_only')
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
                    all_queries=context['queries'],motion_epoch=current['epoch'])
    blocked=ps.region_blocked(region,ps.SelectorConfig(),bounded_unknown_risk=proof['accepted'])
    diagnostic.update(proof=proof,blocked=blocked)
    if region['queries']<=0 or blocked is not None or (region['unobserved'] and not proof['accepted']):
        return reject('current_crop_full_region_rejected')
    diagnostic.update(applied=True,reason='single_current_crop_reach_accepted',current_full_recheck=True)
    route={**before['current_route_proof'],'region':region,'accepted':True,'bounded_current_risk':diagnostic}
    return {**core,'current_route_proof':route,DIAGNOSTIC:diagnostic}
