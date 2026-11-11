"""검사된 좁은 무반환도 낮은 목표에서 전체 GUARD 재검사를 받도록 연결한다."""
import importlib.util
from pathlib import Path
import arena_autonomy.pass_release_trial as ps

P=Path(__file__).resolve().parent.parent/'body_nonreturn_r136_v1/candidate.py'
spec=importlib.util.spec_from_file_location('r136_body_risk',P)
body=importlib.util.module_from_spec(spec);spec.loader.exec_module(body)
ORIGINAL=ps.PassReleaseSelector._guard_curve


def guard_recheck(owner,scene,view,baseline,parallel,speed,previous,dt,age,detail,raw_scan):
    args=(scene,view,baseline,parallel,speed,previous,dt,age,detail,raw_scan)
    result=ORIGINAL(owner,*args)
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


def install():
    body.install();ps.PassReleaseSelector._guard_curve=guard_recheck


def uninstall():
    body.uninstall();ps.PassReleaseSelector._guard_curve=ORIGINAL


def worker_entry(*args):
    install()
    from arena_autonomy.decision_process import _worker
    return _worker(*args)
