"""M3 후보의 동일 시각 미래 외곽 대조. 운영 제어/권한 부여에 사용하지 않는 진단이다.

현재 base_link에 고정한 관측과 미래 후보를 비교한다. 전진·도로 회전을 임의로 빼거나 기준 경로의
전체 시간 합집합으로 가리지 않는다. 관측 격자 표본·숨은 상자 이동 가정일 뿐 연속 안전 증명이 아니다.
"""
import math
import numpy as np

from arena_autonomy.lateral_intent import GEOMETRY_EPS_M
from arena_autonomy.observation_support import depth_support


def interpolate_pose(poses, times, at):
    poses = np.asarray(poses, dtype=float).reshape(-1, 3)
    times = np.asarray(times, dtype=float)
    at = np.asarray(at, dtype=float)
    if (not len(times) or len(poses) != len(times) or not np.all(np.isfinite(poses)) or
            not np.all(np.isfinite(times)) or not np.all(np.isfinite(at)) or np.any(np.diff(times) <= 0) or
            np.any(at < times[0]-1e-12) or np.any(at > times[-1]+1e-12)):
        raise ValueError('time-aligned finite trajectory required; extrapolation forbidden')
    return np.column_stack([np.interp(at, times, poses[:, 0]), np.interp(at, times, poses[:, 1]),
                            np.interp(at, times, np.unwrap(poses[:, 2]))])


def footprint_mask(x, y, pose, length, width):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    dx, dy = x-pose[0], y-pose[1]
    return ((abs(c*dx+s*dy) <= length/2+GEOMETRY_EPS_M) &
            (abs(-s*dx+c*dy) <= width/2+GEOMETRY_EPS_M))


def synchronized_sweep(grid, candidate, reference, speed, cfg, *, horizon_s=None):
    """같은 시각 기준 차체 바깥에 있는 후보 차체 표본과 첫 시각을 구한다.

    한 번 들어왔던 곳이라는 이유로 이후 다른 시각의 진입을 면제하지 않는다. 결과의 first는
    진단 요약용이며 시간에 따라 움직이는 물체와의 검사는 masks/times 전체를 사용해야 한다.
    """
    if not math.isfinite(speed) or speed < 0:
        raise ValueError('forward finite speed required')
    v = max(cfg.planned_speed_floor_mps, speed)
    t = np.asarray(candidate['progress'])/v
    rt = np.asarray(reference['progress'])/v
    horizon = cfg.commit_horizon_s if horizon_s is None else horizon_s
    if not math.isfinite(horizon) or horizon <= 0:
        raise ValueError('positive finite horizon required')
    end = min(horizon, t[-1], rt[-1])
    # 세 궤적의 끝/절점 및 1 cm 또는 20 ms 간격 중 더 촘촘한 시각.
    dt = min(.02, grid.r/max(v, 1e-6))
    times = np.unique(np.r_[0., t[t <= end], rt[rt <= end], np.arange(0., end, dt), end])
    cp = interpolate_pose(candidate['poses'], t, times)
    rp = interpolate_pose(reference['poses'], rt, times)
    x, y = grid.centers()
    masks, reference_masks, extra_masks = [], [], []
    first = np.full(x.shape, np.inf)
    for a, b, at in zip(cp, rp, times):
        ca = footprint_mask(x, y, a, cfg.length_m, cfg.width_m)
        rb = footprint_mask(x, y, b, cfg.length_m, cfg.width_m)
        masks.append(ca)
        reference_masks.append(rb)
        extra = ca & ~rb
        extra_masks.append(extra)
        first[extra] = np.minimum(first[extra], at)
    return {'times': times, 'candidate_poses': cp, 'reference_poses': rp, 'candidate_masks': masks,
            'reference_masks': reference_masks, 'extra_masks': extra_masks, 'first_extra': first,
            'truncated_horizon': end < horizon-1e-12, 'horizon_s': end}


def hidden_arrival(seeds, cells, cfg, speed):
    """현재 기준에 고정한 숨은 상자의 도달 시간 하한. 자차 차단을 빼지 않는 보수적 비교.

    종방향은 |자차 속도|+상대 상계, 횡방향은 기존 횡속도 가정이다. 최소 크기·방향 고정·속도 상계가
    실제 다른 차를 보증하지 않으며, 차가 자차를 통과할 수 없다는 이유로 도달을 늦추지 않는다.
    """
    seeds = np.asarray(seeds).reshape(-1, 2)
    cells = np.asarray(cells).reshape(-1, 2)
    result = np.full(len(cells), np.inf)
    vx, vy = abs(speed)+cfg.hidden_relative_speed_mps, cfg.hidden_lateral_speed_mps
    for start in range(0, len(cells), 128):
        if not len(seeds):
            break
        p = cells[start:start+128]
        dx = np.maximum(0., abs(p[:, None, 0]-seeds[None, :, 0])-cfg.hidden_vehicle_length_m/2)
        dy = np.maximum(0., abs(p[:, None, 1]-seeds[None, :, 1])-cfg.hidden_vehicle_width_m/2)
        result[start:start+128] = np.min(np.maximum(dx/vx, dy/vy), axis=1)
    return result


def review_sweep(scene, grid, candidate, reference, seeds, speed, cfg, *, horizon_s=None):
    sweep = synchronized_sweep(grid, candidate, reference, speed, cfg, horizon_s=horizon_s)
    x, y = grid.centers()
    active = np.logical_or.reduce(sweep['candidate_masks']+sweep['reference_masks'])
    cells = np.column_stack((x[active], y[active]))
    padding = np.array([cfg.budget.range_bias_m+cfg.compact_point_motion_bound_mps*(scene.t_ref_s-r.acquired_at_s)
                        for r in scene.rays])
    supported = depth_support(scene, cells, padding)
    # 현재 자기 차체는 별도 기하 정보다. 현재 관측 검사에서만 제외하고, 미래 동적 도달에서는
    # 제외하지 않는다. 자차가 있던 자리에 상대차가 나중에 들어올 가능성까지 없애면 안 된다.
    current_body = footprint_mask(x, y, (0., 0., 0.), cfg.length_m, cfg.width_m)[active]
    arrival = hidden_arrival(seeds, cells, cfg, speed)
    age = scene.t_ref_s-scene.source_first_s
    stats = {k: np.zeros(len(cells), dtype=bool) for k in ('candidate_unknown', 'reference_unknown', 'extra_unknown',
             'candidate_hidden', 'reference_hidden', 'extra_hidden')}
    for i, t in enumerate(sweep['times']):
        reach = arrival <= t+age
        for k in ('candidate', 'reference', 'extra'):
            mask = sweep[k+'_masks'][i][active]
            stats[k+'_unknown'] |= mask & ~supported & ~current_body
            stats[k+'_hidden'] |= mask & reach
    return {**{k: int(v.sum()) for k, v in stats.items()},
            'extra_cells': int(np.isfinite(sweep['first_extra']).sum()), 'time_samples': len(sweep['times']),
            'truncated_horizon': bool(sweep['truncated_horizon']), 'horizon_s': float(sweep['horizon_s']),
            'control_authorization': False, 'continuous_sweep_certified': False,
            'assumptions': 'sampled poses, axis-aligned hidden box, bounded speeds, no ego-blocking shortcut'}
