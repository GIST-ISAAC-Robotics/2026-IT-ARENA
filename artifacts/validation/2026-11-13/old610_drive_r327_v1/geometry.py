"""관측 경로의 작은 평행 이동 비교. 구동 연결 전 선택형 기하 시험."""
import math
import numpy as np
from arena_autonomy.local_follow import pursue_path, per_ray_padding
from arena_autonomy.local_path import swept_limit


def distances(points, path):
    a = path[:-1]
    v = np.diff(path, axis=0)
    delta = points[:, None, :] - a
    t = np.clip(np.sum(delta*v, axis=2)/np.maximum(1e-12, np.sum(v*v, axis=1)), 0, 1)
    return np.min(np.linalg.norm(delta-t[:, :, None]*v, axis=2), axis=1)


def selection_padding(scene, cfg, refresh_budget_s=.12):
    # 10 Hz 갱신 + 20 ms 명령 간격의 시험 예산. 미래 입력 수신을 허가하지 않는다.
    # 선택 비용만 같은 스캔의 말미 기준으로 맞춘다. 원 점유 검사는 실제 나이 그대로다.
    if not math.isfinite(refresh_budget_s) or refresh_budget_s < 0:
        raise ValueError('invalid selection refresh budget')
    padding=per_ray_padding(scene,.025+cfg.measurement_padding_m,cfg.object_motion_bound_mps)
    return padding+cfg.object_motion_bound_mps*max(0.,scene.source_last_s+refresh_budget_s-scene.t_ref_s)


def choose(path, points, padding, cfg, speed, age, speed_cap=None):
    path = np.asarray(path, dtype=float)
    points = np.asarray(points, dtype=float)
    padding = np.asarray(padding, dtype=float)
    detail = dict(applied=False, reason='invalid_geometry', unknown_filled=False,
                  occupancy_preserved=True)
    if (path.ndim != 2 or path.shape[1] != 2 or len(path) < 4 or
            points.ndim != 2 or points.shape[1] != 2 or not len(points) or
            padding.shape != (len(points),) or not np.all(np.isfinite(path)) or
            not np.all(np.isfinite(points)) or not np.all(np.isfinite(padding)) or
            np.any(padding < 0) or not math.isfinite(speed) or speed < 0 or
            not math.isfinite(age) or age < 0):
        return path, detail
    rear = path + [cfg.wheelbase_m/2, 0.]
    nearest = int(np.argmin(np.linalg.norm(rear, axis=1)))
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    end = min(len(path), int(np.searchsorted(arc, arc[nearest]+1.0))+1)
    start = max(0, nearest-1)
    if end-start < 3:
        return path, {**detail, 'reason': 'short_prefix'}
    threshold = cfg.width_m/2+padding+.01
    before = distances(points, path[start:end])
    if np.all(before >= threshold):
        return path, {**detail, 'reason': 'no_intrusion'}
    tangent = np.gradient(path, axis=0)
    norm = np.linalg.norm(tangent, axis=1)
    if np.any(norm < 1e-6):
        return path, {**detail, 'reason': 'degenerate_path'}
    normals = np.column_stack((-tangent[:, 1], tangent[:, 0]))/norm[:, None]
    clearance = lambda angle: swept_limit(points, angle, cfg.wheelbase_m, horizon=2.5,
                                         margin=padding, length=cfg.length_m, width=cfg.width_m)
    original = pursue_path(path, speed, cfg, age, speed_cap=speed_cap)
    original_gap = clearance(original[1])
    desired_gap = max(cfg.resume_clearance_m, cfg.standstill_gap_m+speed*age+speed**2/4)
    detail.update(reason='no_feasible_offset', original_angle_rad=original[1],
                  original_clearance_m=original_gap, required_clearance_m=desired_gap)
    if original_gap >= desired_gap:
        return path, {**detail, 'reason': 'original_arc_clear'}
    attempts = []
    for magnitude in (.01, .02, .03, .04, .05, .06):
        if magnitude > cfg.boundary_agreement_m:
            break
        options = []
        for offset in (-magnitude, magnitude):
            shifted = path+offset*normals
            if np.any(np.sum(np.diff(path, axis=0)*np.diff(shifted, axis=0), axis=1) <= 0):
                continue
            after = distances(points, shifted[start:end])
            # 반대 벽/별도 물체 쪽으로 밀어 최악 여유가 줄어들면 후보에서 제외한다.
            if float(np.min(after-threshold)) < float(np.min(before-threshold))-1e-9:
                continue
            out = pursue_path(shifted, speed, cfg, age, speed_cap=speed_cap)
            gap = clearance(out[1])
            attempts.append(dict(offset_m=offset, angle_rad=out[1], clearance_m=gap,
                                 minimum_path_margin_m=float(np.min(after-threshold))))
            if gap >= desired_gap:
                options.append((gap, shifted, offset, out))
        if options:
            gap, shifted, offset, out = max(options, key=lambda x: x[0])
            return shifted, {**detail, 'applied': True, 'reason': 'observed_return_path_offset',
                             'offset_m': offset, 'angle_rad': out[1], 'clearance_m': gap,
                             'original_cap_mps': original[0], 'attempts': attempts}
    return path, {**detail, 'attempts': attempts}
