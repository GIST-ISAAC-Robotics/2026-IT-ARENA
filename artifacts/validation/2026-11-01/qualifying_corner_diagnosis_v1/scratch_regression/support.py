"""후보 A 스크래치 회귀 공용 도구. ARENA_CANDIDATE_TREE의 사본만 import한다."""
import copy
import gzip
import json
import math
import os
import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

TREE = Path(os.environ['ARENA_CANDIDATE_TREE'])
for pkg in ('arena_autonomy', 'arena_vehicle_interface'):
    sys.path.insert(0, str(TREE / 'src' / pkg))
import numpy as np
from arena_autonomy import local_follow as lf
from arena_autonomy.follow_trace import replay_call, restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.lidar_observation import audit_front_observation
from arena_autonomy.local_path import braking_speed, mask_scan, swept_limit
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy

STATE = None

HERE = Path(__file__).resolve().parent
RUN = HERE.parents[1] / 'qualifying_profile_v1/run1/follow_decisions.jsonl'
STOP_S, END_S = 46.30, 47.12     # 실제 차체가 멈춘 구간의 저장 입력


def prepare():
    """정지 직전까지 저장 입력을 재생해 제어기·보호층 상태를 만든다. 기록과 다르면 실패한다."""
    follow = policy = motion = None
    calls, mismatch = [], 0
    with RUN.open(encoding='utf-8') as f:
        for line in f:
            r = json.loads(line)
            kind, d = r['kind'], r['data']
            if kind == 'header':
                header = d
                motion = MotionConfig(**d['motion_config'])
                follow = lf.LocalFollow(replace(lf.FollowConfig(**d['follow_config']), low_speed_arc_reacquire=True))
                policy = new_guard_policy()
            elif kind == 'reset':
                follow.reset()
            elif kind == 'call':
                t = d['input']['reference_s']
                if t >= END_S:
                    break
                if t >= STOP_S:
                    calls.append(d['input'])
                    continue
                v, s, _ = replay_call(follow, d['input'], motion)
                mismatch += abs(v-d['output'][0]) > 1e-9 or abs(s-d['output'][1]) > 1e-9
                if t >= STOP_S-3.:
                    guard(policy, d['input'], d['output'][0], d['output'][1], d['input']['previous_steering_rad'], motion, follow.config)
    return SimpleNamespace(header=header, motion=motion, follow=follow, policy=policy, calls=calls, mismatch=mismatch)


def state():
    global STATE
    if STATE is None:
        STATE = prepare()
    return STATE


def controller(flag, **changes):
    follow = copy.deepcopy(state().follow)
    follow.config = replace(follow.config, low_speed_arc_reacquire=flag, **changes)
    return follow


def play(flag, speed=None, inject=None, **changes):
    """저장 정지 장면을 정지 직전 상태에서 이어 재생한다. 조향만 발행 변화율로 되먹임한다."""
    follow, previous, rows = controller(flag, **changes), 0., []
    for call in state().calls:
        data = copy.deepcopy(call)
        data['previous_steering_rad'] = previous
        if speed is not None:
            data['speed_mps'] = speed
        if inject is not None:
            inject(data)
        v, s, detail = replay_call(follow, data, state().motion)
        previous = max(previous-.06, min(previous+.06, s))
        rows.append(SimpleNamespace(t=data['reference_s'], v=v, s=s, published=previous, detail=detail, data=data,
                                    reason=detail.get('reason'), bilateral=detail.get('bilateral_reference'),
                                    fresh=detail.get('fresh_boundary_accepted')))
    return rows


def guard(policy, data, speed, steering, previous, motion=None, config=None):
    """lidar_safety.control의 양수 속도 결정 순서를 노드 없이 그대로 계산한다."""
    motion = motion or state().motion
    config = config or state().follow.config
    data, scan, history = restore_call(data, motion)
    now = data['reference_s']
    masked = mask_scan(scan, data['rear_half_angle_deg'])
    observation = audit_front_observation(masked.ranges, masked.angle_min, masked.angle_increment,
                                          masked.range_min, masked.range_max, history.config.lidar_yaw_rad)
    if observation.reason not in ('ok', 'front_unobserved'):
        return 0., observation.reason, 0.
    first = masked.header.stamp.sec + masked.header.stamp.nanosec*1e-9
    points, ids, _ = history.project(masked.ranges, masked.angle_min, masked.angle_increment, masked.range_min,
                                     masked.range_max, first, masked.time_increment, now, mode='both', timing_verified=True)
    measured = float(history.wheels.available(now)[1][-1])
    partial = policy.evaluate(masked, points, history, now, max(speed, abs(measured)), steering, previous, config.wheelbase_m, config.length_m, config.width_m)
    if not partial['allowed']:
        return 0., partial['reason'], 0.
    if partial['speed_cap_mps'] is not None:
        speed = min(speed, partial['speed_cap_mps'])
    clearance = swept_limit(points, steering, config.wheelbase_m, length=config.length_m, width=config.width_m)
    cap = braking_speed(clearance, now-first)
    return float(min(speed, cap)), 'clear' if cap >= speed else 'lidar_braking', float(clearance)


def new_guard_policy():
    return RecoveryPolicy(allow_narrow_forward=True, surface_translation_enabled=True, lateral_metric_enabled=True)


# 평면 벽만 있는 합성 정지 장면. 무반환은 inf로 남기며 자유 공간으로 채우지 않는다.
ANGLE_MIN, COUNT, STEP = -math.pi, 500, math.tau/500


def ranges_for(segments, blind=()):
    motion = state().motion
    origin = np.array([motion.lidar_x_m, motion.lidar_y_m])
    angles = ANGLE_MIN+np.arange(COUNT)*STEP
    out = np.full(COUNT, np.inf)
    for (ax, ay, bx, by) in segments:
        a, d = np.array([ax, ay], dtype=float)-origin, np.array([bx-ax, by-ay], dtype=float)
        ux, uy = np.cos(angles), np.sin(angles)
        det = ux*(-d[1])+d[0]*uy
        safe = np.where(np.abs(det) < 1e-12, np.nan, det)
        t = (a[0]*(-d[1])+d[0]*a[1])/safe
        s = (ux*a[1]-uy*a[0])/safe
        hit = np.nan_to_num(np.where((t > .05) & (s >= 0) & (s <= 1), t, np.inf), nan=np.inf)
        out = np.minimum(out, hit)
    for lo, hi in blind:
        out[(angles >= lo) & (angles <= hi)] = np.inf
    return out.tolist()


def play_scene(segments, flag, *, side='right', blind=(), steps=30, speed=0.):
    follow = lf.LocalFollow(replace(state().follow.config, low_speed_arc_reacquire=flag))
    history = MotionHistory(state().motion)
    t0, previous, rows = 10., 0., []
    for k in range(int(3.*100)):
        history.wheels.times.append(t0-2.+k*.01), history.wheels.values.append(0.)
    for k in range(int(3.*200)):
        history.gyro.times.append(t0-2.+k*.005), history.gyro.values.append(0.)
    for k in range(steps):
        now = round(t0+k*.02, 6)
        first = round(math.floor((now-.102)*10)/10+.002, 6)
        scan = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=int(first), nanosec=int(round((first-int(first))*1e9))),
                                                      frame_id='laser_frame'), ranges=ranges_for(segments, blind),
                               angle_min=ANGLE_MIN, angle_increment=STEP, range_min=.05, range_max=12., time_increment=.0002)
        scan = mask_scan(scan, 30.)
        scene = motion_scene(scan, history, now, timing_verified=True, epoch=0, rear_half_angle_deg=30.)
        v, s, detail = follow.command(scene, history, speed, side, previous, .02, raw_scan=scan)
        previous = max(previous-.06, min(previous+.06, s))
        rows.append(SimpleNamespace(t=now, v=v, s=s, reason=detail.get('reason'), bilateral=detail.get('bilateral_reference')))
    return rows


def corridor(extra=(), half=.425, length=3.):
    return [(-1., -half, length, -half), (-1., half, length, half), (-1., -half, -1., half), *extra]


def fork(start_x, half=.425, spread=math.radians(45), arm=2.5):
    """start_x까지 곧은 뒤 좌우 벽이 벌어지고 가운데 코가 있는 Y자 갈림."""
    dx, dy = arm*math.cos(spread), arm*math.sin(spread)
    nose = start_x+half/math.sin(spread)+.45
    return [(-1., -half, start_x, -half), (start_x, -half, start_x+dx, -half-dy),
            (-1., half, start_x, half), (start_x, half, start_x+dx, half+dy),
            (nose, 0., nose+dx, -dy), (nose, 0., nose+dx, dy), (-1., -half, -1., half)]
