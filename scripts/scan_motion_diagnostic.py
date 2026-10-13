"""기록 전용 2D 스캔/모터축 이동 일치성 후보. ROS/제어기에서 import하지 않는다.

Point-to-line 반복 정합은 원시 두 스캔만 사용한다. 엔코더는 정합 완료 후
잔차 비교에만 사용한다. 정적 구조 가정·미보정 순차 스캔 오차 예산이 필요하며
compatible도 실제 정지/정확한 속도 확인값이 아니다.
"""
from dataclasses import asdict, dataclass
import math
import time

import numpy as np


@dataclass(frozen=True)
class Settings:
    max_range_m: float = 3.
    min_range_m: float = .08
    field_half_deg: float = 130.
    min_points: int = 35
    max_points: int = 180
    normal_half_window: int = 8
    segment_max_m: float = .20
    correspondence_m: float = .24
    huber_m: float = .025
    max_rmse_m: float = .04
    min_fraction: float = .65
    min_x_support: int = 12
    min_x_support_span_m: float = .35
    min_eigen_ratio: float = .003
    max_translation_m: float = .55
    max_yaw_rad: float = .16
    max_iterations: int = 16
    seed_offset_m: float = .18
    bidirectional_m: float = .04
    bidirectional_rad: float = .025
    range_uncertainty_m: float = .03
    # 아래는 시험용 가정 상한이며 센서 사양/차체 실제 상한이 아니다.
    sweep_speed_bound_mps: float = 1.5
    sweep_yaw_bound_rad_s: float = .5
    encoder_distance_tolerance_m: float = .002
    rear_to_lidar_m: float = .1275

    def __post_init__(self):
        for key, value in asdict(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'invalid {key}')
        for key in ('min_points', 'max_points', 'min_x_support', 'max_iterations', 'normal_half_window'):
            if type(getattr(self, key)) is not int:
                raise ValueError(f'noninteger {key}')
        if self.min_points > self.max_points or self.min_fraction > 1 or self.field_half_deg >= 180:
            raise ValueError('inconsistent settings')


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def prepare(scan, cfg):
    """빈/무응답을 잇거나 자유 공간으로 채우지 않는다. 완전 회전 메타데이터가 필요하다."""
    for key in ('stamp_s', 'time_increment', 'angle_min', 'angle_increment'):
        if key not in scan or not math.isfinite(scan[key]):
            raise ValueError('scan_metadata')
    r = np.asarray(scan['ranges'], dtype=float)
    n = len(r)
    span = scan['time_increment']*(n-1)
    if (r.ndim != 1 or not 100 <= n <= 2000 or scan['stamp_s'] < 0
            or scan['time_increment'] <= 0 or scan['angle_increment'] <= 0
            or abs(scan['angle_increment']*n-math.tau) > .03 or not .05 <= span <= .15):
        raise ValueError('scan_metadata')
    a = scan['angle_min']+np.arange(n)*scan['angle_increment']
    a = np.arctan2(np.sin(a), np.cos(a))
    good = np.isfinite(r) & (r >= cfg.min_range_m) & (r <= cfg.max_range_m)
    good &= np.abs(a) <= math.radians(cfg.field_half_deg)
    p = np.full((n, 2), np.nan)
    p[good] = r[good, None]*np.column_stack((np.cos(a[good]), np.sin(a[good])))
    indices = np.flatnonzero(good)
    points = p[indices[::max(1, math.ceil(len(indices)/cfg.max_points))]]
    adjacent = good[:-1] & good[1:]
    delta = p[1:]-p[:-1]
    lengths = np.linalg.norm(delta, axis=1)
    adjacent &= (lengths > .001) & (lengths <= cfg.segment_max_m)
    segments, normal_list = [], []
    # 잡음이 있는 두 점만으로 법선을 만들면 평행 벽도 가짜 완전 관측성을 얻는다.
    # 같은 연속 반환의 작은 이웃을 PCA로 검사하되 점 자체를 보간/대체하지 않는다.
    for i in np.flatnonzero(adjacent):
        lo, hi = max(0, i-cfg.normal_half_window), min(n, i+cfg.normal_half_window+2)
        if not np.all(good[lo:hi]) or not np.all(adjacent[lo:hi-1]):
            continue
        local = p[lo:hi]
        centered = local-np.mean(local, axis=0)
        eigen, basis = np.linalg.eigh(centered.T@centered/len(local))
        if eigen[1] < .0001 or eigen[0]/eigen[1] > .05:
            continue
        segments.append(i)
        normal_list.append(basis[:, 0])
    starts, vectors = p[:-1][segments], delta[segments]
    normals = np.asarray(normal_list).reshape(-1, 2)
    if len(points) < cfg.min_points or len(starts) < cfg.min_points:
        raise ValueError('insufficient_returns')
    return {'points': points, 'starts': starts, 'vectors': vectors, 'normals': normals,
            'duration_s': span, 'reference_s': scan['stamp_s']+span/2}


def correspond(points, target, transform, cfg):
    rotated = points @ rotation(transform[2]).T
    q = rotated+transform[:2]
    diff = q[:, None, :]-target['starts'][None, :, :]
    vector = target['vectors']
    u = np.einsum('ijk,jk->ij', diff, vector)/np.sum(vector*vector, axis=1)
    closest = target['starts'][None, :, :]+np.clip(u, 0., 1.)[:, :, None]*vector
    d2 = np.sum((q[:, None, :]-closest)**2, axis=2)
    match = np.argmin(d2, axis=1)
    index = np.arange(len(points))
    keep = d2[index, match] <= cfg.correspondence_m**2
    normal = target['normals'][match][keep]
    residual = np.sum(normal*(q[keep]-closest[index, match][keep]), axis=1)
    jac = np.column_stack((normal, -normal[:, 0]*rotated[keep, 1]+normal[:, 1]*rotated[keep, 0]))
    return residual, jac, q[keep], keep


def fit(source, target, cfg, seed):
    transform = np.array([seed, 0., 0.])
    converged = False
    for iteration in range(cfg.max_iterations):
        residual, jac, q, keep = correspond(source['points'], target, transform, cfg)
        if len(residual) < cfg.min_points:
            return None
        weights = np.minimum(1., cfg.huber_m/np.maximum(np.abs(residual), 1e-12))
        step = np.linalg.lstsq(jac*np.sqrt(weights[:, None]), -residual*np.sqrt(weights), rcond=1e-8)[0]
        # 먼 초기값도 작은 신뢰 구간 안에서 수렴시킨다. 첫 보정량이 크다는
        # 이유만으로 버리면 정지 장면도 0 초기값 하나만 살아남는다.
        scale = min(1., .10/max(np.linalg.norm(step[:2]), 1e-12), .04/max(abs(step[2]), 1e-12))
        step *= scale
        transform += step
        if np.linalg.norm(transform[:2]) > cfg.max_translation_m or abs(transform[2]) > cfg.max_yaw_rad:
            return None
        if np.linalg.norm(step) < 1e-5:
            converged = True
            break
    residual, jac, q, keep = correspond(source['points'], target, transform, cfg)
    inlier = np.abs(residual) <= cfg.max_rmse_m*2
    if np.count_nonzero(inlier) < cfg.min_points:
        return None
    jac, q, residual = jac[inlier], q[inlier], residual[inlier]
    eigen = np.linalg.eigvalsh(jac.T@jac/len(jac))
    x_support = np.abs(jac[:, 0]) > .5
    x_span = float(np.ptp(q[x_support, 1])) if np.any(x_support) else 0.
    sectors = set(np.floor((np.arctan2(q[:, 1], q[:, 0])+math.pi)/(math.pi/4)).astype(int))
    return {'transform': transform.tolist(), 'rmse_m': float(np.sqrt(np.mean(residual**2))),
            'inlier_fraction': float(len(residual)/len(source['points'])), 'inliers': len(residual),
            'eigen_ratio': float(eigen[0]/max(eigen[-1], 1e-12)),
            'x_support': int(np.count_nonzero(x_support)), 'x_support_span_m': x_span,
            'angular_sectors': len(sectors), 'converged': converged, 'iterations': iteration+1}


def quality(result, cfg):
    if result is None:
        return 'registration_failed'
    if not result['converged']:
        return 'not_converged'
    if result['rmse_m'] > cfg.max_rmse_m or result['inlier_fraction'] < cfg.min_fraction:
        return 'poor_overlap_or_fit'
    if result['eigen_ratio'] < cfg.min_eigen_ratio:
        return 'geometric_degeneracy'
    if (result['x_support'] < cfg.min_x_support or result['x_support_span_m'] < cfg.min_x_support_span_m
            or result['angular_sectors'] < 3):
        return 'localized_motion_support'
    return None


def register(source, target, cfg):
    results = [fit(source, target, cfg, cfg.seed_offset_m*seed) for seed in (-2, -1, 0, 1, 2)]
    available = [r for r in results if r is not None]
    if not available:
        return None, 'registration_failed'
    valid = [r for r in available if quality(r, cfg) is None]
    if not valid:
        best = min(available, key=lambda r: r['rmse_m'])
        return best, quality(best, cfg)
    best = min(valid, key=lambda r: r['rmse_m'])
    if len(valid) < 2:
        return best, 'insufficient_multistart_agreement'
    if any(np.linalg.norm(np.array(r['transform'])-best['transform']) > .05 for r in valid):
        return best, 'ambiguous_registration'
    return best, None


def compare_scans(previous, current, encoder_travel_m, cfg=None):
    """current 센서→previous 센서 변환. 정답/지도/명령 속도 인자를 받지 않는다."""
    started = time.perf_counter()
    cfg = cfg or Settings()
    result = {'classification': 'unknown', 'reason': None, 'stationary_confirmed': False,
              'static_scene_assumption_verified': False, 'forward': None, 'reverse': None}
    def finish(reason):
        result['reason'] = reason
        result['processing_ms'] = (time.perf_counter()-started)*1000
        return result
    try:
        a, b = prepare(previous, cfg), prepare(current, cfg)
    except (ValueError, TypeError, KeyError) as error:
        return finish(str(error))
    interval = b['reference_s']-a['reference_s']
    if not .07 <= interval <= .35:
        return finish('pair_time_gap')
    result['reference_interval_s'] = interval
    if encoder_travel_m is None or not math.isfinite(encoder_travel_m):
        return finish('encoder_history_missing')
    forward, reason = register(b, a, cfg)
    result['forward'] = forward
    if reason:
        return finish(reason)
    reverse, reason = register(a, b, cfg)
    result['reverse'] = reverse
    if reason:
        return finish(reason)
    f, r = np.array(forward['transform']), np.array(reverse['transform'])
    closure = f[:2]+rotation(f[2])@r[:2]
    if np.linalg.norm(closure) > cfg.bidirectional_m or abs(f[2]+r[2]) > cfg.bidirectional_rad:
        return finish('bidirectional_disagreement')
    # 후륜 중심과 LiDAR 원점의 회전 레버암 차이. 부호 있는 전진 이동만 비교한다.
    x = f[0]+cfg.rear_to_lidar_m*(1-math.cos(f[2]))
    if abs(f[2]) > .12:
        return finish('turning_without_gyro_history')
    if abs(x)/interval > cfg.sweep_speed_bound_mps or abs(f[2])/interval > cfg.sweep_yaw_bound_rad_s:
        return finish('outside_assumed_motion_envelope')
    sweep = .5*(a['duration_s']+b['duration_s'])*(cfg.sweep_speed_bound_mps+
                                               cfg.max_range_m*cfg.sweep_yaw_bound_rad_s)
    fit_budget = 2*max(forward['rmse_m'], reverse['rmse_m'])
    budget = 2*cfg.range_uncertainty_m+sweep+fit_budget+cfg.encoder_distance_tolerance_m
    residual = x-encoder_travel_m
    result.update(encoder_travel_m=float(encoder_travel_m), scan_forward_travel_m=float(x),
                  distance_residual_m=float(residual), comparison_budget_m=budget,
                  raw_sweep_budget_m=sweep, range_pair_budget_m=2*cfg.range_uncertainty_m,
                  fit_budget_m=fit_budget)
    result['classification'] = 'inconsistent_candidate' if abs(residual) > budget else 'compatible_under_assumptions'
    return finish('residual_exceeds_budget' if abs(residual) > budget else 'residual_within_budget_not_motion_proof')
