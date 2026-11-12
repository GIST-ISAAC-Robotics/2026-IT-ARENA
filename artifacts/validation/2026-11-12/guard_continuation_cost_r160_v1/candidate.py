"""명시 비교용: 현재 검사를 통과한 저속 GUARD를 먼저 검사한다."""
import importlib.util
import math
from pathlib import Path

import arena_autonomy.pass_release_trial as ps

path = Path(__file__).resolve().parent.parent / 'adjacent_mount_gap_r149_v1/candidate.py'
spec = importlib.util.spec_from_file_location('r160_previous', path)
prior = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior)

PROBE_INTERVAL_S = .3
LOW_TARGET_MPS = .15
_old_guard = None
_old_reset = None


def reset(owner):
    _old_reset(owner)
    owner._guard_cost_state = None


def guard_continuation(owner, scene, view, baseline, parallel, speed, previous,
                       dt, age, detail, raw_scan):
    args = (scene, view, baseline, parallel, speed, previous, dt, age, detail, raw_scan)
    state = getattr(owner, '_guard_cost_state', None)
    b = baseline[2]
    partial = b.get('partial_observation') or {}
    current = (owner.mode == 'GUARD' and owner.last_choice == 'guard_baseline'
        and owner.follow_config.traffic_recovery_trial
        and 0 <= speed <= LOW_TARGET_MPS and baseline[0] > 0
        and b.get('boundary_from_memory') is False
        and b.get('path_source_s') == scene.source_first_s
        and partial.get('allowed') is True and partial.get('reason') == 'fully_observed'
        and not any(side['present'] for side in owner.sides.values()))
    if (state is not None and current and scene.source_epoch == state['epoch']
            and state['source_s'] <= scene.source_first_s
            and state['started_s'] <= scene.t_ref_s < state['probe_at_s']):
        # 기억하는 것은 탐색 순서뿐이다. 현재의 전체 GUARD 판정을 다시 수행한다.
        result = owner._guard_curve_at_cap(*args, target_cap=LOW_TARGET_MPS)
        if result is not None:
            proof = result[2].get('selector_guard_candidate') or {}
            if result[0] > 0 and proof.get('accepted') is True:
                owner._guard_cost_state = {**state, 'source_s': scene.source_first_s}
                return result[0], result[1], {**result[2], 'selector_guard_continuation': {
                    'applied': True, 'current_full_guard_rechecked': True,
                    'target_cap_mps': LOW_TARGET_MPS, 'next_upward_probe_s': state['probe_at_s'],
                    'source_s': scene.source_first_s, 'unknown_filled': False}}
    # 새 위협/기억 경로/시간 변경/현재 검사 실패는 원 탐색으로 돌아간다.
    owner._guard_cost_state = None
    result = _old_guard(owner, *args)
    if result is not None:
        recheck = result[2].get('selector_horizon_recheck') or {}
        proof = result[2].get('selector_guard_candidate') or {}
        if (recheck.get('applied') is True and proof.get('accepted') is True
                and result[0] == LOW_TARGET_MPS and math.isfinite(scene.t_ref_s)):
            owner._guard_cost_state = dict(epoch=scene.source_epoch,
                source_s=scene.source_first_s, started_s=scene.t_ref_s,
                probe_at_s=scene.t_ref_s + PROBE_INTERVAL_S)
    return result


def install():
    global _old_guard, _old_reset
    if ps.PassReleaseSelector._guard_curve is guard_continuation:
        return
    prior.install()
    _old_guard = ps.PassReleaseSelector._guard_curve
    _old_reset = ps.PassReleaseSelector.reset
    ps.PassReleaseSelector._guard_curve = guard_continuation
    ps.PassReleaseSelector.reset = reset


def uninstall():
    if ps.PassReleaseSelector._guard_curve is guard_continuation:
        ps.PassReleaseSelector._guard_curve = _old_guard
        ps.PassReleaseSelector.reset = _old_reset
    prior.uninstall()
