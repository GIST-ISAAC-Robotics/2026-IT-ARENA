"""기존 접근 관측 판정은 유지하고 낮은 목표의 전체 GUARD를 재검사한다."""
import importlib.util
import math
from pathlib import Path
import arena_autonomy.pass_release_trial as ps
from arena_autonomy import current_path_recheck as common
from arena_autonomy.local_follow import forward_planning_speed

P = Path(__file__).resolve().parent.parent/'adjacent_low_retry_r178_v1/candidate.py'
spec = importlib.util.spec_from_file_location('r181_prior', P)
prior = importlib.util.module_from_spec(spec); spec.loader.exec_module(prior)
_old_guard = None
CAPS = (.25, .15)


def eligible(owner, scene, baseline, speed, result):
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


def guard(owner, scene, view, baseline, parallel, speed, previous, dt, age, detail, raw_scan):
    args = (scene, view, baseline, parallel, speed, previous, dt, age, detail, raw_scan)
    original = _old_guard(owner, *args)
    if not eligible(owner, scene, baseline, speed, original): return original
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


def install():
    global _old_guard
    if ps.PassReleaseSelector._guard_curve is guard: return
    prior.install()
    _old_guard = ps.PassReleaseSelector._guard_curve
    ps.PassReleaseSelector._guard_curve = guard


def uninstall():
    if ps.PassReleaseSelector._guard_curve is guard:
        ps.PassReleaseSelector._guard_curve = _old_guard
