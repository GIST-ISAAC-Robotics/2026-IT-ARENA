"""SOLO 현재 검사에만 명시 연결하는 저속 상대 거리 공백 후보. 기본 미연결."""
import math

from . import solo_crawl as prior
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.current_path_recheck._gaps import bounded_body_nonreturn
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.follow_trace import thaw

CAP=.10


def rebase(output,submitted,current,scene,history,cfg,*,require_current_region=False):
    before=prior.rebase(output,submitted,current,scene,history,cfg,require_current_region=require_current_region)
    if before is None or before['speed_mps']>0:return before
    normal=output[2].get('normal') or {}
    partial=normal.get('partial_observation') or {}
    speed=current['speed_mps']
    prior_gap=before.get('current_solo_gap_retry') or {}
    if (before.get('reason')!='route_current_large_motion_region_unobserved'
            or not math.isfinite(speed) or not 0<=speed<=.15 or output[0]<=.15
            or output[2].get('handoff_state')!='FOLLOW' or normal.get('selector_mode')!='SOLO'
            or normal.get('boundary_from_memory') is not False
            or partial.get('allowed') is not True or partial.get('reason')!='fully_observed'
            or prior_gap.get('applied') is not False or prior_gap.get('cap_mps')!=.15):
        return before
    core=prior.prior.prior.base.candidate_policy.rebase((CAP,output[1],output[2]),submitted,current,scene,history,cfg)
    if (core is None or not 0<core['speed_mps']<=CAP
            or core['steering_rad']!=before['steering_rad'] or core['reference_s']!=before['reference_s']):
        return before
    context={}
    region=ps.motion_region(scene,speed,core['speed_mps'],core['steering_rad'],current['previous_steering_rad'],
                            current['dt_s'],cfg,ps.lateral_config(cfg),context=context)
    proof=dict(accepted=False,reason='no_unknown_context')
    if region['unobserved'] and context:
        q=context['queries'][~context['free'] & ~context['masked']]
        raw=MeasurementScan(**thaw(current['scan']['measurements']))
        proof=bounded_body_nonreturn(scene,raw,q,context['padding'],cfg.length_m,cfg.width_m,
                                    speed,core['speed_mps'],relative_neighbor=True)
    blocked=ps.region_blocked(region,ps.SelectorConfig(),bounded_unknown_risk=proof['accepted'])
    accepted=region['queries']>0 and proof['accepted'] and blocked is None
    diagnostic=dict(applied=accepted,cap_mps=CAP,proof=proof,region=region,blocked=blocked,
        actual_speed_mps=speed,unknown_filled=False,source_timestamp_renewed=False,same_steering=True)
    if not accepted:return {**before,'current_relative_body_retry':diagnostic}
    route=dict(before['current_route_proof'],region=region,accepted=True,bounded_current_risk=diagnostic)
    return {**core,'current_route_proof':route,'current_relative_body_retry':diagnostic}
