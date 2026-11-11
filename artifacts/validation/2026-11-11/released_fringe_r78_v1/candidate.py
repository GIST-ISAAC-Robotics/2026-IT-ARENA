"""기존 저속 후방 검사를 통과 확인 완료 상태에도 쓰는 격리 연결 후보."""
import math
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.released_rear_sweep import bounded_released_rear_sweep
ORIGINAL_SELECT=ps.PassReleaseSelector.select

def reconsider(owner,scene,baseline,selected,speed,previous,dt,raw_scan):
    cfg=owner.follow_config;b_speed,b_steer,b_detail=baseline;detail=selected[2]
    partial=b_detail.get('partial_observation') or {}
    if not (cfg.traffic_recovery_trial and cfg.released_rear_sweep_trial
            and owner.mode=='RELEASED' and not detail.get('retained_passage',True)
            and all(not s['present'] for s in owner.sides.values())
            and detail.get('selector_reason')=='large_motion_region_unobserved'
            and b_detail.get('reason') in ps.GUARD_BASELINE_REASONS
            and partial.get('allowed') is True and partial.get('reason')=='fully_observed'
            and raw_scan is not None):
        return None
    if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
               for v in (speed,b_speed,b_steer,previous,dt)) or not 0<=speed<=.35 or b_speed<=0:
        return None
    target=min(b_speed,owner.config.guard_speed_cap_mps)
    context={}
    region=ps.motion_region(scene,speed,target,b_steer,previous,dt,cfg,owner.lat,context=context)
    if not region['unobserved'] or not context:return None
    q=context['queries'][~context['free'] & ~context['masked']]
    proof=bounded_released_rear_sweep(scene,raw_scan,q,context['padding'],cfg.length_m,cfg.width_m,
        speed,target,owner.config.small_excursion_m)
    if not proof['accepted'] or ps.region_blocked(region,owner.config,bounded_unknown_risk=True) is not None:
        return None
    return target,b_steer,{**detail,**b_detail,'selector_choice':'baseline','parallel_hold_selected':False,
        'selector_reason':'released_confirmed_rear_fringe',
        'selector_region':region,'selector_released_rear_sweep':proof,
        'selector_released_rear_sweep_risk_accepted':True,
        'selector_bounded_rear_fringe_risk_accepted':False,
        'selector_masked_tail_risk_accepted':bool(region['masked_left'] or region['masked_right']),
        'selector_excursion_m':ps.lateral_excursion(max(speed,target),scene.t_ref_s-scene.source_first_s,
            b_steer,previous,dt,cfg),
        'selector_confirmed_rear_recheck':dict(original_target_mps=b_speed,target_mps=target,
            measured_speed_mps=speed,unknown_filled=False)}

def select(self,scene,baseline,parallel,speed,previous,dt,*,raw_scan=None):
    out=ORIGINAL_SELECT(self,scene,baseline,parallel,speed,previous,dt,raw_scan=raw_scan)
    refined=reconsider(self,scene,baseline,out,speed,previous,dt,raw_scan)
    if refined is not None:
        out=refined;self.last_choice=out[2].get('selector_choice')
    return out
