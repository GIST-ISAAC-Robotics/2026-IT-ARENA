"""측방 통과 뒤 저속의 작은 후반부 무반환 이동. 자유 공간 증명이 아니다.

통과 이력 유지 및 통과 확인 완료 뒤 저속 재검사에서 호출한다. 앞부분·범퍼 너머·현재 점유는 유지한다.
1 cm 영역은 공백 하나 3도, 그중 1 mm 영역만 기존 전체 공백 6도 예산을
한 공백에도 적용한다. 가려진 돌출물·차체/운동 오차의 위험은 남는다.
"""
import math
import numpy as np
from arena_autonomy.observation_support import source_valid
from arena_autonomy.rear_fringe_trial import _bounded_nonreturn


def bounded_released_rear_sweep(scene, raw_scan, queries, padding, length, width,
                               speed, target, maximum_exterior_m):
    result=dict(accepted=False, reason='invalid_input', unknown_filled=False,
        residual_hidden_object_risk=True, maximum_exterior_m=maximum_exterior_m,
        maximum_gap_rad=math.radians(3), minimum_neighbor_range_m=.6,
        policy='released_low_speed_rear_sweep_v1')
    if (raw_scan is None or not source_valid(scene) or
            any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x)
                for x in (speed,target,length,width,maximum_exterior_m)) or
            not 0<=speed<=.35 or not 0<target<=.35 or length<=0 or width<=0 or
            not 0<maximum_exterior_m<=.01):
        return result
    q=np.asarray(queries,dtype=float).reshape(-1,2);pad=np.asarray(padding,dtype=float)
    if (not len(q) or not np.all(np.isfinite(q)) or pad.shape!=(len(scene.rays),)
            or not np.all(np.isfinite(pad)) or np.any(pad<0)):
        return result
    exterior=np.abs(q[:,1])-width/2
    result.update(queries=len(q),exterior_max_m=float(exterior.max()))
    if (np.any(q[:,0]<-length/2) or np.any(q[:,0]>0) or np.any(exterior<=0)
            or np.any(exterior>maximum_exterior_m)):
        return {**result,'reason':'outside_released_rear_sweep'}
    answer=_bounded_nonreturn(scene,raw_scan,q,pad,result)
    if answer['accepted']:answer['reason']='bounded_released_rear_sweep_risk'
    elif answer['reason']=='wide_nonreturn' and float(exterior.max())<=.001:
        # 이미 쓰는 1 mm 후반부 기하와 전체 6도 예산의 조합이다. 공백의 수/합계,
        # 양옆 실제 반환·팽창·시각·원시 +inf는 같은 함수로 다시 검사한다.
        retry=_bounded_nonreturn(scene,raw_scan,q,pad,{**result,
            'maximum_gap_rad':math.radians(6),
            'maximum_exterior_m':min(.001,maximum_exterior_m),
            'policy':'released_low_speed_body_fringe_v2'})
        if retry['accepted']:
            retry.update(reason='bounded_released_body_fringe_risk',
                         previous_rejection='wide_nonreturn')
            return retry
    return answer


def reconsider_confirmed_rear_fringe(owner,scene,baseline,selected,speed,previous,dt,raw_scan):
    from arena_autonomy import pass_release_trial as ps
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
