"""양쪽 벽 오차 구간의 진단용 교집합. 주행 허가·벽 분류·도로 보장을 하지 않는다.

입력은 같은 횡단 구간에서 평행한 ±wall_half_m 벽이라고 가정한 반환의 횡좌표다.
그 가정이 틀리거나 다른 차의 면이면 구간이 존재해도 벽/도로라고 결론 낼 수 없다.
기존 제어기에는 연결하지 않는다. 오차 평균 상쇄·독립 확률·알 수 없는 면 보간을 쓰지 않는다.
"""
import math
import numpy as np


def center_interval(left_l, right_l, *, wall_half_m=.425, error_m=.03):
    """모든 반환을 각각 ±error로 설명하는 공통 중심의 폐구간. 비어 있으면 None."""
    if (not all(math.isfinite(v) for v in (wall_half_m, error_m)) or
            wall_half_m <= 0 or error_m < 0):
        raise ValueError('finite positive wall half-width and nonnegative error required')
    left, right = np.asarray(left_l, dtype=float), np.asarray(right_l, dtype=float)
    if left.ndim != 1 or right.ndim != 1 or not np.isfinite(left).all() or not np.isfinite(right).all():
        raise ValueError('finite one-dimensional point coordinates required')
    if not len(left) or not len(right):
        return None
    centers = np.r_[left-wall_half_m, right+wall_half_m]
    lo, hi = float(np.max(centers-error_m)), float(np.min(centers+error_m))
    # 반올림으로 모순인 관측을 통과시키지 않는다. 빈 구간은 보수적으로 미확정이다.
    return (lo, hi) if lo <= hi else None


def binned_pair_probe(frame, points, s_lo, s_hi, cfg):
    """단면별 모형 호환성만 기록한다. 빈 bin은 전후 벽으로 채우지 않는다."""
    if not math.isfinite(s_lo) or not math.isfinite(s_hi) or s_hi <= s_lo:
        raise ValueError('ordered finite observed interval required')
    s, l = frame.project(np.asarray(points, dtype=float).reshape(-1, 2))
    edges = np.r_[np.arange(s_lo, s_hi, cfg.wall_bin_m), s_hi]
    rows = []
    for a, b in zip(edges[:-1], edges[1:]):
        longitudinal = (s >= a) & ((s < b) if b < s_hi else (s <= b))
        # 탐색용 띠: 단일 벽 경로 오차와 반대 벽 오차를 합친 범위.
        # 이 띠에 들어왔다는 이유로 벽으로 분류하거나 주행 조건을 완화하지 않는다.
        sides = [l[longitudinal & (abs(l-sign*cfg.wall_offset_m) <= 2*cfg.budget.range_bias_m)]
                 for sign in (1., -1.)]
        interval = center_interval(*sides, wall_half_m=cfg.wall_offset_m, error_m=cfg.budget.range_bias_m)
        rows.append(dict(s_lo=float(a), s_hi=float(b), left_points=len(sides[0]), right_points=len(sides[1]),
            center_interval_m=interval, status='compatible' if interval is not None else
            'missing_side' if any(not len(x) for x in sides) else 'inconsistent'))
    return dict(bins=rows, compatible_bins=sum(r['status']=='compatible' for r in rows),
        missing_bins=sum(r['status']=='missing_side' for r in rows),
        inconsistent_bins=sum(r['status']=='inconsistent' for r in rows),
        scope='conditional piecewise-parallel wall compatibility only', driving_authorized=False)
