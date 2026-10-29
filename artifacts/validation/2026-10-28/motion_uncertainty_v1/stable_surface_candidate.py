"""미연결 수치 수렴 후보. 기존 기하/지지/대응 거리 조건은 바꾸지 않는다."""
import math
import numpy as np
from arena_autonomy.partial_surface_alignment import surface_translation as original_translation


def stable_cycle(result):
    if result.get('accepted') or result.get('reason') != 'unsettled_or_high_residual':
        return None
    steps = result.get('steps', [])
    if len(steps) < 6:
        return None
    try:
        delta = np.asarray([row['step_m'] for row in steps], dtype=float)
        residual = np.asarray([row['residual_rms_m'] for row in steps], dtype=float)
        support = np.asarray([[row['fraction'], row['normal_eigen_ratio']] for row in steps], dtype=float)
    except (TypeError, ValueError, KeyError):
        return None
    if (delta.shape != (len(steps), 2) or not np.all(np.isfinite(delta))
            or not np.all(np.isfinite(residual)) or not np.all(np.isfinite(support))
            or np.any(residual < 0) or np.any(support[:, 0] < .65)
            or np.any(support[:, 1] < .02) or residual[-1] > .01):
        return None
    tail = delta[-3:]
    positions = np.vstack((np.zeros(2), np.cumsum(tail, axis=0)))
    diameter = float(np.max(np.linalg.norm(positions[:, None] - positions[None], axis=2)))
    spread = float(np.ptp(residual[-3:]))
    shift = delta.sum(axis=0)
    norm = float(np.linalg.norm(shift))
    # 0.1 mm의 추정 해 진동만 다룬다. 1 cm 잔차 한도·6 cm 이동 한도와
    # 2 mm 초과 이동의 개선율 조건을 완화하지 않는다.
    if diameter > .0001 or spread > .00001 or norm > .06:
        return None
    if norm > .002 and residual[-1] > residual[0] * .6:
        return None
    return dict(translation_m=shift.tolist(), numerical_cycle_diameter_m=diameter,
        numerical_residual_spread_m=spread, numerical_window_iterations=3,
        numerical_maximum_diameter_m=.0001, numerical_maximum_residual_spread_m=.00001)


def surface_translation(previous_now, current, source_indices):
    result = original_translation(previous_now, current, source_indices)
    cycle = stable_cycle(result)
    if cycle is None:
        return result
    return {**result, **cycle, 'accepted': True, 'reason': 'bounded_surface_numerical_cycle',
        'scope': 'correspondence_only_not_independent_motion'}
