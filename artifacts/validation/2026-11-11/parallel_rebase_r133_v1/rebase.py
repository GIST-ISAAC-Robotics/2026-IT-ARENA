"""선택된 직선 차로 유지 경로의 현재 대조. 명시 시험용이며 운영 미연결."""
import importlib.util
from pathlib import Path

import numpy as np

from arena_autonomy.parallel_lane_trial import lateral_excursion

_path = Path(__file__).resolve().parent.parent/'follow_rebase_r126_v1/rebase.py'
_spec = importlib.util.spec_from_file_location('baseline_rebase_r126', _path)
_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_module)


def supported(output):
    speed, _, detail = output
    normal = detail.get('normal') or {}
    common = (_module.finite(speed) and speed > 0 and detail.get('handoff_state') == 'FOLLOW')
    baseline = (normal.get('reason') == 'local_follow' and normal.get('selector_mode') == 'SOLO'
                and normal.get('selector_choice') == 'baseline')
    parallel = (normal.get('reason') == 'parallel_hold_follow'
                and normal.get('selector_mode') in ('SOLO', 'GUARD')
                and normal.get('selector_choice') == 'parallel'
                and normal.get('parallel_hold_selected') is True
                and (normal.get('parallel_hold_trial') or {}).get('eligible') is True
                and (normal.get('parallel_hold_trial') or {}).get('reason') == 'parallel_current_position')
    return bool(common and (baseline or parallel))


def rebase(output, submitted, current, scene, history, cfg):
    if not supported(output):
        return None
    normal = output[2]['normal']
    if normal.get('selector_choice') == 'baseline':
        return _module.rebase(output, submitted, current, scene, history, cfg)
    parallel = normal['parallel_hold_trial']
    diagnostic = dict(rebased_selector_mode=normal['selector_mode'], rebased_selector_choice='parallel',
                      lateral_envelope_limit_m=.01, lane_recentering=False)

    def stop(reason):
        return dict(speed_mps=0., steering_rad=current['previous_steering_rad'], reason=reason,
                    reference_s=current['reference_s'], path_source_s=normal.get('path_source_s'),
                    original_reference_s=submitted['reference_s'], independent_guard_required=True,
                    runtime_authorized=False, **diagnostic)

    partial = normal.get('partial_observation') or {}
    if (parallel.get('straight_trial_only') is not True
            or partial.get('allowed') is not True or partial.get('reason') != 'fully_observed'
            or (parallel.get('side_departure') or {}).get('applied') is True):
        return stop('rebase_parallel_original_scope_invalid')
    try:
        actual_path = np.asarray(normal.get('path_points', []), dtype=float).reshape(-1, 2)
        selected_path = np.asarray(parallel.get('path_points', []), dtype=float).reshape(-1, 2)
        if (actual_path.shape != selected_path.shape or len(actual_path) < 6
                or not np.array_equal(actual_path, selected_path)):
            return stop('rebase_parallel_selected_path_mismatch')
    except (TypeError, ValueError):
        return stop('rebase_parallel_selected_path_mismatch')
    # 현재 횡위치로 다시 중앙을 정하지 않는다. 당시 선택된 같은 경로만 운반한다.
    surrogate = (output[0], output[1], {**output[2], 'normal': {
        **normal, 'reason': 'local_follow', 'selector_mode': 'SOLO', 'selector_choice': 'baseline'}})
    result = _module.rebase(surrogate, submitted, current, scene, history, cfg)
    if result is None:
        raise RuntimeError('supported parallel rebase unexpectedly unavailable')
    result.update(diagnostic)
    if result['speed_mps'] <= 0:
        return result
    speed = abs(current['speed_mps'])
    age = current['reference_s']-scene.source_first_s
    previous = current['previous_steering_rad']
    steering = result['steering_rad']
    dt = current['dt_s']
    excursion = lateral_excursion(max(speed, result['speed_mps']), age, steering, previous, dt, cfg)
    if excursion > .01:
        # 목표속도를 줄여도 이미 빠른 실제 운동을 가렸다고 처리하지 않는다.
        if lateral_excursion(speed, age, steering, previous, dt, cfg) > .01:
            return stop('rebase_parallel_lateral_motion_exceeded')
        low, high = 0., result['speed_mps']
        for _ in range(20):
            mid = (low+high)/2
            if lateral_excursion(max(speed, mid), age, steering, previous, dt, cfg) <= .01:
                low = mid
            else:
                high = mid
        result['speed_mps'] = low
        excursion = lateral_excursion(max(speed, low), age, steering, previous, dt, cfg)
    result.update(maximum_body_corner_lateral_m=excursion, reason='rebase_current_parallel')
    return result
