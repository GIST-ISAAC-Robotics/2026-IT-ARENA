"""목표 속도를 낮춘 전체 GUARD 검사를 비교한다. 기존 위험 상한은 유지한다."""
import importlib.util
from pathlib import Path

import arena_autonomy.pass_release_trial as ps

P=Path(__file__).resolve().parents[2]/'2026-11-11/guard_recheck_r139_v1/candidate.py'
spec=importlib.util.spec_from_file_location('r148_previous',P)
prior=importlib.util.module_from_spec(spec);spec.loader.exec_module(prior)


def guard_horizon(owner,scene,view,baseline,parallel,speed,previous,dt,age,detail,raw_scan):
    args=(scene,view,baseline,parallel,speed,previous,dt,age,detail,raw_scan)
    original=prior.guard_recheck(owner,*args)
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


def install():
    prior.install()
    ps.PassReleaseSelector._guard_curve=guard_horizon


def uninstall():
    prior.uninstall()


def worker_entry(*args):
    install()
    from arena_autonomy.decision_process import _worker
    return _worker(*args)
