"""r149의 기존 0.15 m/s 검사를 누락한 경우만 전체 저속 검사를 수행한다."""
import importlib.util
import math
from pathlib import Path

import arena_autonomy.pass_release_trial as ps

P = Path(__file__).resolve().parent.parent/'adjacent_live_r152_v1/candidate.py'
spec = importlib.util.spec_from_file_location('r178_prior', P)
prior = importlib.util.module_from_spec(spec); spec.loader.exec_module(prior)
_old_guard = None


def eligible(owner, scene, baseline, speed, result):
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


def guard(owner, scene, view, baseline, parallel, speed, previous, dt, age, detail, raw_scan):
    args = (scene, view, baseline, parallel, speed, previous, dt, age, detail, raw_scan)
    original = _old_guard(owner, *args)
    if not eligible(owner, scene, baseline, speed, original):
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


def install():
    global _old_guard
    if ps.PassReleaseSelector._guard_curve is guard:
        return
    prior.install()
    _old_guard = ps.PassReleaseSelector._guard_curve
    ps.PassReleaseSelector._guard_curve = guard


def uninstall():
    if ps.PassReleaseSelector._guard_curve is guard:
        ps.PassReleaseSelector._guard_curve = _old_guard
