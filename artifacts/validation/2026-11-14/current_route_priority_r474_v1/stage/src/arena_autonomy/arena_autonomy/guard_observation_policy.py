"""명시 선택형 기존 GUARD 관측/저속 정책. 전역 patch나 artifact import 없음.

r136/r139/r148/r149/r178/r181의 검사 순서와 반환 근거를 보존한다.
기본 original_v1은 이 모듈을 호출하지 않는다. 현재 경로 재검사/평가 지연/
최근 참조 보존과 독립이며 미완주 후보의 기본 채택이나 실차 승인이 아니다.
"""
import math
from arena_autonomy import current_path_recheck as common
from arena_autonomy.local_follow import forward_planning_speed
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.current_path_recheck._gaps import bounded_body_nonreturn, bounded_adjacent_structure
from arena_autonomy.known_structure import parse_sectors

CAPS = (.25, .15)


def _same_path_with_body(owner,scene,raw_scan,region,context,speed,target,steer,previous,baseline):
    old=ps._same_path_risk_original(owner,scene,raw_scan,region,context,speed,target,steer,previous,baseline)
    partial=baseline.get('partial_observation') or {}
    if (old['accepted'] or not context or not region['unobserved']
            or not owner.follow_config.traffic_recovery_trial
            or owner.mode!='GUARD' or any(s['present'] for s in owner.sides.values())
            or partial.get('allowed') is not True or partial.get('reason')!='fully_observed'):
        return old
    q=context['queries'][~context['free'] & ~context['masked']]
    proof=bounded_body_nonreturn(scene,raw_scan,q,context['padding'],owner.follow_config.length_m,
                                owner.follow_config.width_m,speed,target)
    return proof if proof['accepted'] else {**old,'body_nonreturn_recheck':proof}


def same_path_risk(owner,scene,raw_scan,region,context,speed,target,steer,previous,baseline):
    old=_same_path_with_body(owner,scene,raw_scan,region,context,speed,target,steer,previous,baseline)
    check=old.get('body_nonreturn_recheck') or {}
    if (old.get('accepted') or check.get('reason') not in ('unbounded_nonreturn','mask_boundary')
            or not context or owner.follow_config.known_structure_policy!='structure_aware_v1'):
        return old
    q=context['queries'][~context['free'] & ~context['masked']]
    proof=bounded_adjacent_structure(scene,raw_scan,q,context['padding'],owner.follow_config.length_m,
        owner.follow_config.width_m,speed,target,parse_sectors(owner.follow_config.known_structure_sectors_json))
    return proof if proof['accepted'] else {**old,'adjacent_structure_recheck':proof}


def guard_recheck(owner,scene,view,baseline,parallel,speed,previous,dt,age,detail,raw_scan):
    args=(scene,view,baseline,parallel,speed,previous,dt,age,detail,raw_scan)
    result=ps.PassReleaseSelector._guard_curve_original(owner,*args)
    if result is None or not owner.follow_config.traffic_recovery_trial:return result
    info=result[2].get('selector_guard_candidate') or {}
    region=info.get('region') or {};proof=info.get('same_path_gap') or {}
    partial=baseline[2].get('partial_observation') or {}
    target=info.get('target_speed_mps',0.)
    if (result[2].get('selector_reason') not in ('guard_curve_rejected_large_motion_masked_toward_left',
                                               'guard_curve_rejected_large_motion_masked_toward_right')
            or not region.get('unobserved') or region.get('masked_front_samples')!=0
            or partial.get('allowed') is not True or partial.get('reason')!='fully_observed'
            or proof.get('policy')!='guard_low_speed_body_nonreturn_r136' or proof.get('accepted') is not True
            or target<=.15 or speed>=target):
        return result
    attempts=[]
    for cap in (.25,.15):
        if cap>=target or cap>owner.config.guard_speed_cap_mps:continue
        candidate=owner._guard_curve_at_cap(*args,target_cap=cap)
        if candidate is None:continue
        checked=candidate[2].get('selector_guard_candidate') or {}
        accepted=candidate[0]>0 and checked.get('accepted') is True
        attempts.append(dict(target_mps=cap,accepted=accepted,reason=candidate[2].get('selector_reason'),
            region=checked.get('region'),same_path_gap=checked.get('same_path_gap')))
        if accepted:result=candidate;break
    return result[0],result[1],{**result[2],'selector_guard_speed_recheck':dict(
        applied=result[0]>0,original_target_mps=target,original_region=region,attempts=attempts,
        measured_motion_preserved=True,unknown_filled=False,policy='bounded_body_full_guard_recheck_r139')}


def guard_horizon(owner,scene,view,baseline,parallel,speed,previous,dt,age,detail,raw_scan):
    args=(scene,view,baseline,parallel,speed,previous,dt,age,detail,raw_scan)
    original=guard_recheck(owner,*args)
    if original is None or original[0]>0 or not owner.follow_config.traffic_recovery_trial:
        return original
    b=baseline[2]
    info=original[2].get('selector_guard_candidate') or {}
    body=(info.get('same_path_gap') or {}).get('body_nonreturn_recheck') or {}
    partial=b.get('partial_observation') or {}
    target=info.get('target_speed_mps',0.)
    if (original[2].get('selector_reason')!='guard_curve_rejected_large_motion_region_unobserved'
            or body.get('policy')!='guard_low_speed_body_nonreturn_r136'
            or body.get('reason')!='outside_small_body_side'
            or b.get('boundary_from_memory') is not False
            or b.get('path_source_s')!=scene.source_first_s
            or partial.get('allowed') is not True or partial.get('reason')!='fully_observed'
            or any(s['present'] for s in owner.sides.values())
            or not 0<=speed<target or target<=.15):
        return original
    attempts=[]
    answer=original
    for cap in (.25,.15):
        if cap>=target or cap>owner.config.guard_speed_cap_mps:
            continue
        checked=owner._guard_curve_at_cap(*args,target_cap=cap)
        if checked is None:
            continue
        evidence=checked[2].get('selector_guard_candidate') or {}
        accepted=checked[0]>0 and evidence.get('accepted') is True
        attempts.append(dict(target_mps=cap, accepted=accepted,
                             reason=checked[2].get('selector_reason'),
                             region=evidence.get('region'), same_path_gap=evidence.get('same_path_gap')))
        if accepted:
            answer=checked
            break
    return answer[0],answer[1],{**answer[2],'selector_horizon_recheck':dict(
        applied=answer[0]>0, original_target_mps=target, original_region=info.get('region'),
        attempts=attempts, measured_motion_preserved=True, unknown_filled=False,
        policy='guard_full_lower_horizon_r148')}


def eligible_adjacent(owner, scene, baseline, speed, result):
    if result is None or result[0] > 0 or not owner.follow_config.traffic_recovery_trial:
        return False
    b = baseline[2]; detail = result[2]
    proof = detail.get('selector_guard_candidate') or {}
    gap = proof.get('same_path_gap') or {}
    body = gap.get('body_nonreturn_recheck') or {}
    adjacent = gap.get('adjacent_structure_recheck') or {}
    partial = b.get('partial_observation') or {}
    attempts = (detail.get('selector_horizon_recheck') or {}).get('attempts') or []
    return (owner.mode == 'GUARD'
            and isinstance(speed, (float, int)) and not isinstance(speed, bool)
            and math.isfinite(speed) and 0 <= speed <= .15
            and owner.config.guard_speed_cap_mps >= .15
            and baseline[0] > 0 and b.get('boundary_from_memory') is False
            and b.get('path_source_s') == scene.source_first_s
            and partial.get('allowed') is True and partial.get('reason') == 'fully_observed'
            and not any(side['present'] for side in owner.sides.values())
            and detail.get('selector_reason') == 'guard_curve_rejected_large_motion_region_unobserved'
            and body.get('policy') == 'guard_low_speed_body_nonreturn_r136'
            and body.get('reason') in ('unbounded_nonreturn', 'mask_boundary')
            and adjacent.get('policy') == 'guard_adjacent_structure_gap_r149'
            and adjacent.get('reason') == 'adjacent_structure_outside_scope'
            and proof.get('target_speed_mps', 0.) > .15
            and not any(a.get('target_mps') == .15 for a in attempts))


def guard_adjacent(owner, scene, view, baseline, parallel, speed, previous, dt, age, detail, raw_scan):
    args = (scene, view, baseline, parallel, speed, previous, dt, age, detail, raw_scan)
    original = guard_horizon(owner, *args)
    if not eligible_adjacent(owner, scene, baseline, speed, original):
        return original
    checked = owner._guard_curve_at_cap(*args, target_cap=.15)
    proof = {} if checked is None else checked[2].get('selector_guard_candidate') or {}
    accepted = checked is not None and checked[0] > 0 and proof.get('accepted') is True
    result = checked if accepted else original
    return result[0], result[1], {**result[2], 'selector_adjacent_low_retry': {
        'applied': accepted, 'target_cap_mps': .15, 'measured_speed_mps': speed,
        'current_full_guard_rechecked': True, 'unknown_filled': False,
        'low_reason': None if checked is None else checked[2].get('selector_reason'),
        'low_guard': proof}}


def eligible_approach(owner, scene, baseline, speed, result):
    if result is None or result[0] > 0:
        return False
    normal = baseline[2]
    partial = normal.get('partial_observation') or {}
    return (owner.mode == 'GUARD' and owner.follow_config.traffic_recovery_trial
            and math.isfinite(speed) and 0 <= speed <= CAPS[0]
            and baseline[0] > 0 and normal.get('boundary_from_memory') is False
            and normal.get('path_source_s') == scene.source_first_s
            and partial.get('allowed') is True and partial.get('reason') == 'fully_observed'
            and result[2].get('selector_reason') in ('guard_curve_rejected_toward_left', 'guard_curve_rejected_toward_right'))


def guard_curve(owner, scene, view, baseline, parallel, speed, previous, dt, age, detail, raw_scan):
    args = (scene, view, baseline, parallel, speed, previous, dt, age, detail, raw_scan)
    original = guard_adjacent(owner, *args)
    if not eligible_approach(owner, scene, baseline, speed, original): return original
    original_cap = (original[2].get('selector_guard_candidate') or {}).get('target_speed_mps', 0.)
    attempts = []
    result = original
    for cap in CAPS:
        if cap >= original_cap or speed > cap or cap > owner.config.guard_speed_cap_mps: continue
        checked = owner._guard_curve_at_cap(*args, target_cap=cap)
        proof = {} if checked is None else checked[2].get('selector_guard_candidate') or {}
        accepted = checked is not None and checked[0] > 0 and proof.get('accepted') is True
        attempts.append(dict(cap_mps=cap, accepted=accepted,
                             reason=None if checked is None else checked[2].get('selector_reason')))
        if accepted:
            result = checked; break
    if not attempts: return original
    return result[0], result[1], {**result[2], 'selector_approach_low_retry': {
        'applied': result[0] > 0, 'attempts': attempts, 'measured_speed_mps': speed,
        'current_full_guard_rechecked': True, 'unknown_filled': False}}


def supported(output):
    return common.supported(output)


def rebase(output, submitted, current, scene, history, cfg):
    original = common.rebase(output, submitted, current, scene, history, cfg)
    if (original is None or original['speed_mps'] > 0
            or original['reason'] not in ('guard_current_approach_not_observed', 'guard_rebase_toward_unreleased_side')):
        return original
    speed = forward_planning_speed(cfg, current['speed_mps'])
    if not math.isfinite(speed) or not 0 <= speed <= CAPS[0]: return original
    attempts = []
    result = original
    for cap in CAPS:
        if cap >= output[0] or speed > cap: continue
        # 원 경로/허가/이력은 그대로 두고 허용 상한만 줄여 전체 함수를 재실행한다.
        low = (cap, output[1], output[2])
        checked = common.rebase(low, submitted, current, scene, history, cfg)
        accepted = checked is not None and checked['speed_mps'] > 0
        attempts.append(dict(cap_mps=cap, accepted=accepted,
                             reason=None if checked is None else checked['reason']))
        if accepted:
            result = checked; break
    if not attempts: return original
    return {**result, 'current_approach_low_retry': {
        'applied': result['speed_mps'] > 0, 'original_reason': original['reason'],
        'attempts': attempts, 'original_speed_cap_mps': output[0],
        'measured_speed_mps': current['speed_mps'], 'full_current_check': True,
        'unknown_filled': False}}
