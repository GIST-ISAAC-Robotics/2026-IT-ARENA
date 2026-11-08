"""현재 우회 경로의 짧은 목표점 대조. 비교 함수를 주입하며 자체 구동 허가는 만들지 않는다."""
import math
import numpy as np
from arena_autonomy.local_follow import pursue_path
from arena_autonomy.stopped_entry_centering import stopped_shaft_window


def shorter_pursuit(path, speed, config, age, *, lookahead_m, speed_cap=None):
    """원 경로의 길이/곡률 속도 예산을 유지하고 같은 경로의 가까운 점을 추종한다."""
    if lookahead_m not in (.24, .18, .12):
        raise ValueError('registered short entry preview required')
    cap, _, remaining = pursue_path(path, speed, config, age, speed_cap=speed_cap)
    rear = np.asarray(path, dtype=float).copy()
    rear[:, 0] += config.wheelbase_m/2
    distances = np.linalg.norm(rear, axis=1)
    nearest = int(np.argmin(distances))
    ahead = np.flatnonzero((rear[:, 0] > .10) & (np.arange(len(rear)) >= nearest))
    target = rear[ahead[np.argmin(abs(distances[ahead]-lookahead_m))]]
    curvature = float(2*target[1]/max(.01, target@target))
    steer = float(np.clip(math.atan(config.wheelbase_m*curvature), -config.max_steering_rad, config.max_steering_rad))
    return min(cap, math.sqrt(config.lateral_acceleration_mps2/max(.15, abs(curvature)))), steer, remaining


def compare_shorter_previews(primary, compare, *, history, now, speed, previous, dt):
    """compare(주시거리, 직전각)는 원 점유/관측/제동 검사를 포함한 같은 장면의 비교다.

    양수 후보는 실제 직전각 검사에서만 받는다. 정지 정렬 가설은 속도 0의 조향 요청만
    반환한다. 실물 서보 도달/차체 미끄러짐에 관한 증명은 아니다.
    """
    rows = primary.get('candidates', [])
    if (primary.get('selected') is not None or primary.get('reason') != 'no_entry_command_candidate'
            or len(rows) != 1 or rows[0].get('reason') != 'known_occupied_command_hold'
            or not all(math.isfinite(v) for v in (now, speed, previous, dt)) or not 0 < dt <= .2 or speed < 0):
        return primary
    quiet = speed == 0 and stopped_shaft_window(history, now)
    attempts = []
    for lookahead in (.24, .18, .12):
        alternative = compare(lookahead, previous)
        candidates = alternative.get('candidates', [])
        attempts.append(dict(lookahead_m=lookahead,reason=alternative.get('reason'),selected=alternative.get('selected') is not None))
        if alternative.get('selected') is not None:
            return dict(alternative,adaptive_preview=dict(applied=True,lookahead_m=lookahead,alignment_only=False,attempts=attempts))
        if not quiet or len(candidates) != 1:
            continue
        # 비교기는 실제 변화율을 적용하기 전 같은 경로의 목표각도 함께 남긴다.
        desired = candidates[0].get('preview_unclipped_steering_rad')
        if not isinstance(desired,(int,float)) or isinstance(desired,bool) or not math.isfinite(desired):
            continue
        aligned = compare(lookahead, desired)
        if aligned.get('selected') is None:
            continue
        request = float(np.clip(desired,previous-3*dt,previous+3*dt))
        return dict(primary,adaptive_preview=dict(applied=True,lookahead_m=lookahead,alignment_only=True,attempts=attempts),
                    preview_alignment=dict(previous_rad=previous,desired_rad=desired,requested_rad=request,
                        new_motion_authorized=False,shaft_quiet_window_s=.20))
    return dict(primary,adaptive_preview=dict(applied=False,alignment_only=False,attempts=attempts))
