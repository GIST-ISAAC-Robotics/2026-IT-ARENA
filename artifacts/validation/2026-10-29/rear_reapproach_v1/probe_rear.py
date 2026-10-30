"""옆→뒤→후방 사각 소실과 재접근의 합성 폐루프 비교(현재 통합 G1 후보).

기존 G2 합성 평가기(scripts/probe_bypass_entry_sequence.py)와 같은 순차 광선·이상
bicycle·가상 모터축/gyro·독립 보호 함수(ShadowSafety)를 쓴다. 제어기는 Gazebo 도로 안
시험 헤더와 같은 FollowConfig의 LocalFollow(평행/통과 선택기·GUARD 곡선·교통/경로 회복)다.
정답 자세·actor 이름·일정은 광선 합성과 평가에만 쓰며 제어기에는 마스크된 스캔,
가상 모터축/gyro 이력, 벽 쪽('left'), 직전 조향만 전달한다. ROS/Gazebo/실차 검증이 아니다.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
from shapely.geometry import Polygon

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface'), str(ROOT/'scripts')]
import probe_bypass_entry_sequence as probe  # noqa: E402  (ShadowSafety 경로 설정 포함)
from arena_autonomy.follow_trace import snapshot_call, FollowTrace, replay_events  # noqa: E402
from arena_autonomy.lateral_intent import LateralIntentConfig, wheel_contact_corners  # noqa: E402
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory, MotionUnavailable  # noqa: E402
from arena_autonomy.local_follow import FollowConfig, LocalFollow  # noqa: E402
from arena_autonomy.local_path import mask_scan  # noqa: E402
from arena_autonomy.scene_motion import motion_scene  # noqa: E402
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy  # noqa: E402
from bypass_probe_sensors import ProbeSensors  # noqa: E402
from g0_geometry import TruthBox, Footprint, ray_box_distance  # noqa: E402
from g2_tracking_probe import bicycle_step  # noqa: E402
from safety_shadow import ShadowSafety  # noqa: E402

# Gazebo m3_openframe_v1/drive_left 판단 기록 헤더와 같은 값(제어기 설정 복사본).
HEADER_FOLLOW = dict(partial_observation_policy='bounded_local_v4', parallel_hold_trial=True,
    pass_release_trial=True, guard_curve_trial=True, traffic_recovery_trial=True,
    traffic_path_recovery_trial=True, corridor_fallback_enabled=True, short_boundary_prefix_enabled=True,
    recent_observation_enabled=False, boundary_evidence_mode='corner_distance_candidate',
    occupancy_timing_mode='per_ray_candidate', max_steering_rad=0.3710583448410034,
    lateral_acceleration_mps2=3.0)
HEADER_MOTION = dict(wheel_radius_m=.0325, rear_axle_x_m=-.0675, lidar_x_m=.06, history_s=2.)
GREEN_S = .5
EGO_Y = .09          # 기존 도로 안 시험의 자차 시작 횡위치 크기(옆차 반대쪽)
SIDE_Y = .20         # v2 등록: 옆차 횡위치(왼쪽 두 바퀴 잔디). v1 0.13 m는 자차가 앞서지 못해 장면 미성립
HALF_ROAD = .225
LEAD = [(0., 12., .35)]
LEAD_STOP = [(0., 4., .35), (4., 9., 0.), (9., 15., .35)]

# 이름: 옆차 부호(+1 왼쪽), 시작 상대 x, 옆차 일정, 주앞차 일정, 길이, 추가 정지 상자, 스캔 공백
CASES = {
    'A_recede_left': dict(sign=1., side_dx=0., side=[(0., 12., .15)], lead=LEAD, duration=12.),
    'A_recede_right': dict(sign=-1., side_dx=0., side=[(0., 12., .15)], lead=LEAD, duration=12.),
    'B_reapproach_left': dict(sign=1., side_dx=0., side=[(0., 4.5, .15), (4.5, 12., .6)], lead=LEAD, duration=12.),
    'B_reapproach_right': dict(sign=-1., side_dx=0., side=[(0., 4.5, .15), (4.5, 12., .6)], lead=LEAD, duration=12.),
    'C_lead_stop_left': dict(sign=1., side_dx=0., side=[(0., 4., .15), (4., 9., 0.), (9., 15., .15)],
                             lead=LEAD_STOP, duration=15.),
    'D_opposite_residual_left': dict(sign=1., side_dx=0., side=[(0., 12., .15)], lead=LEAD, duration=12.,
                                     residual=[(2.2, -.36)]),
    'E_time_gap_left': dict(sign=1., side_dx=0., side=[(0., 12., .15)], lead=LEAD, duration=12.,
                            scan_gap=(2.6, 3.2)),
    # 사후 진단(판정 아님): 앞차가 빨라 자차가 해제 뒤 가속할 수 있을 때 차선 유지 후보의 속도 비용.
    'G_fast_lead_left': dict(sign=1., side_dx=0., side=[(0., 8., .15)], lead=[(0., 8., .8)], duration=8.),
    # v3: 자차가 정지에서 출발하므로 옆차도 0.3 s 뒤 같은 속도로 출발해 뒤 옆 관측을 유지한다(v2는 옆차가 먼저 지나감).
    'F_visible_trailing_left': dict(sign=1., side_dx=-.40, side=[(0., .3, 0.), (.3, 12., .35)], lead=LEAD,
                                    duration=12., side_y=.13),
    'F_visible_trailing_left_y20': dict(sign=1., side_dx=-.40, side=[(0., .3, 0.), (.3, 12., .35)], lead=LEAD,
                                        duration=12.),
}


def travel(schedule, elapsed):
    distance = 0.
    for start, end, speed in schedule:
        distance += speed*max(0., min(elapsed, end)-start)
    return distance


def actor_speed(schedule, elapsed):
    return next((v for a, b, v in schedule if a <= elapsed < b), 0.)


def world(at, case):
    """평가 전용 정답 상자. (label, TruthBox) 목록."""
    elapsed = max(0., at-GREEN_S)
    # 끝벽은 C1 최대 거리 12 m 안에 둔다(17 m 끝벽은 정면 무반환을 만들어 장면과 무관한 부분 관측 경로로 들어갔다).
    boxes = [('wall', TruthBox(3., sign*.45, .4, Footprint(12., .05), label='wall')) for sign in (-1, 1)]
    boxes += [('wall', TruthBox(x, 0., .4, Footprint(.05, 1.), label='wall')) for x in (-3.025, 9.025)]
    boxes.append(('lead', TruthBox(1.6+travel(case['lead'], elapsed), 0., .12, label='lead')))
    boxes.append(('side', TruthBox(case['side_dx']+travel(case['side'], elapsed), case['sign']*case.get('side_y', SIDE_Y), .12,
                                   label='side')))
    for x, y in case.get('residual', ()):
        boxes.append(('residual', TruthBox(x, y, .12, label='residual')))
    return boxes


def render_scan(rows, now, case, sensors):
    """순차 500광선. 마지막 광선은 발행보다 한 간격 앞선다. 광선별 정답 라벨은 평가 전용."""
    first_ns = round((now-.1)*1e9)
    first = first_ns*1e-9
    ranges, labels = [], []
    for i in range(500):
        at = first+i*.0002
        rx, ry, yaw = probe.physical_pose(rows, at)
        cx, cy = rx+.0675*math.cos(yaw), ry+.0675*math.sin(yaw)
        bearing = yaw-math.pi+i*math.tau/500+sensors.actual_lidar_yaw
        origin = (cx+.06*math.cos(yaw), cy+.06*math.sin(yaw), .095)
        direction = (math.cos(bearing), math.sin(bearing), 0.)
        best, label = math.inf, None
        for name, box in world(at, case):
            distance = ray_box_distance(origin, direction, box, 0., 12.)
            if distance is not None and distance < best:
                best, label = distance, name
        best = sensors.range_value(best)
        ranges.append(best if best >= .05 else math.nan)
        labels.append(label)
    raw = SimpleNamespace(header=SimpleNamespace(frame_id='laser_frame', stamp=SimpleNamespace(
        sec=first_ns//1_000_000_000, nanosec=first_ns % 1_000_000_000)), ranges=ranges,
        angle_min=-math.pi, angle_increment=math.tau/500, time_increment=.0002,
        range_min=.05, range_max=12.)
    masked = mask_scan(raw, 30.)
    visible = Counter(label for label, value in zip(labels, masked.ranges)
                      if label is not None and math.isfinite(value))
    return masked, dict(visible)


def center_of(rear):
    rx, ry, yaw = rear
    return rx+.0675*math.cos(yaw), ry+.0675*math.sin(yaw), yaw


def evaluation(rear, steering, now, case):
    """평가 전용 기하. 제어기에 전달하지 않는다."""
    cx, cy, yaw = center_of(rear)
    ego = Polygon(Footprint().polygon(cx, cy, yaw))
    out = {'center': [cx, cy, yaw]}
    for name, box in world(now, case):
        if name == 'wall':
            continue
        poly = Polygon(box.footprint.polygon(box.x_m, box.y_m, box.yaw_rad))
        gap = ego.distance(poly)
        contact = ego.intersects(poly)
        key = name if name != 'residual' else 'residual'
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = box.x_m-cx, box.y_m-cy
        out[key] = dict(gap_m=gap, contact=bool(contact), dx=c*dx+s*dy, dy=-s*dx+c*dy)
    patches = wheel_contact_corners(np.array([[cx, cy, yaw]]), steering, LateralIntentConfig())
    out['wheels_outside_road'] = int(np.any(np.abs(patches[0, :, :, 1]) > HALF_ROAD+1e-10, axis=1).sum())
    corners = np.asarray(Footprint().polygon(cx, cy, yaw))
    out['toward_side_extent_m'] = float(np.max(case['sign']*corners[:, 1]))
    return out


def classify(dx, visible, distance):
    if abs(dx) <= .2:
        return 'alongside'
    if dx > .2:
        return 'ahead'
    if visible > 0:
        return 'behind_visible'
    return 'blind' if distance <= 1.5 else 'far_hidden'


def stop_restart_metrics(case, run):
    """C 장면의 기존 stop_restart 관문 값(앞차 4-9 s 정지). 다른 장면은 None."""
    if case['lead'] is not LEAD_STOP:
        return None
    held = [r for r in run if 4. <= r['t'] < 9. and r['actual_speed_mps'] < .01 and r['safe'][0] == 0]
    longest = run_length = 0.
    for r in run:
        if 4. <= r['t'] < 9. and r['actual_speed_mps'] < .01 and r['safe'][0] == 0:
            run_length += .02
            longest = max(longest, run_length)
        else:
            run_length = 0.
    restart = next((r['t'] for r in run if r['t'] >= 9. and r['actual_speed_mps'] >= .1), None)
    at_restart = next((r['evaluation']['center'][0] for r in run if r['t'] >= 9.), None)
    return dict(hold_samples=len(held), longest_hold_s=round(longest, 3),
                restart_delay_s=None if restart is None else round(restart-9., 3),
                progress_after_restart_m=None if at_restart is None else run[-1]['evaluation']['center'][0]-at_restart,
                gates=dict(minimum_hold_s=1., maximum_restart_delay_s=2.5, restart_speed_mps=.1,
                           minimum_progress_after_restart_m=1.))


def compress(states):
    out = []
    for t, state in states:
        if not out or out[-1][1] != state:
            out.append([t, state])
    return out


def realized(name, sequence):
    order = [s for _, s in sequence]

    def ordered(*required):
        index = 0
        for state in order:
            if index < len(required) and state == required[index]:
                index += 1
        return index == len(required)
    kind = name[0]
    if kind in 'ACDEG':
        after_blind = order[order.index('blind'):] if 'blind' in order else []
        return ordered('alongside', 'behind_visible', 'blind') and not any(
            s in ('alongside', 'ahead') for s in after_blind)
    if kind == 'B':
        if not ordered('alongside', 'behind_visible', 'blind'):
            return False
        rest = order[order.index('blind'):]
        return 'ahead' in rest and any(s in ('behind_visible', 'alongside') for s in rest)
    if kind == 'F':
        # 등록 문장대로 사각 없이 뒤 옆 관측이 유지되어야 한다(옆/앞으로 바뀌면 미성립).
        return bool(order) and set(order) == {'behind_visible'}
    return False


def rollout(name, *, sensor_profile='ideal', seed=17, trace_path=None, geometry_only=False, rear_lane_keep=False):
    case = CASES[name]
    config = FollowConfig(**HEADER_FOLLOW, **({'rear_lane_keep_trial': True} if rear_lane_keep else {}))
    motion = MotionConfig(**HEADER_MOTION)
    history = MotionHistory(motion)
    controller = LocalFollow(config)
    guard = ShadowSafety(RecoveryPolicy(True, True, True), config, 30.)
    sensors = ProbeSensors(sensor_profile, seed)
    trace = FollowTrace(str(trace_path), motion, config) if trace_path else None
    ego_y = -case['sign']*EGO_Y
    pose = (-.0675, ego_y, 0.)
    physical, records, scan_states = [], [], []
    actual_speed = previous = travel_m = 0.
    for k in range(25):
        t = k*.02
        physical.append(dict(t=t, rear_pose=list(pose), speed=0., steering=0.))
        mean, gyro, _ = sensors.measure_motion(t, 0., 0., 0.)
        history.add_mean_wheel(t, mean)
        history.add_gyro(t, gyro)
    raw = visible = None
    scan_time = None
    epoch = 0
    gap = case.get('scan_gap')
    duration = case['duration']
    for k in range(round((duration+.6)/.02)+1):
        now = round(GREEN_S+k*.02, 9)
        physical.append(dict(t=now, rear_pose=list(pose), speed=actual_speed, steering=previous))
        if sensors.motion_enabled:
            mean, gyro, _ = sensors.measure_motion(now, travel_m, actual_speed, previous)
            history.add_mean_wheel(now, mean)
            history.add_gyro(now, gyro)
        elapsed = now-GREEN_S
        in_gap = gap is not None and gap[0] <= elapsed < gap[1]
        if k % 5 == 0 and not in_gap:
            raw, visible = render_scan(physical, now, case, sensors)
            scan_time = now
            ev = evaluation(pose, previous, now, case)
            side = ev['side']
            scan_states.append([round(elapsed, 3), classify(side['dx'], visible.get('side', 0),
                                                             math.hypot(side['dx'], side['dy'])),
                                visible.get('side', 0)])
        _, measured_values = history.wheels.available(now)
        measured_speed = float(measured_values[-1])
        scheduled_stop = elapsed >= duration
        status = 'ok'
        if geometry_only:
            command, detail = (min(.35, actual_speed+.02) if not scheduled_stop else 0., 0.), {}
            safe = command
        elif now-scan_time > motion.scan_timeout_s:
            # 실제 노드의 SENSOR_STOP: 출력 0과 제어기 재설정(관측 기억은 unknown).
            status = 'sensor_stop'
            controller.reset()
            if trace:
                trace.emit('reset', {'reason': 'SENSOR_STOP'})
            command, detail = (0., previous), {'reason': 'sensor_stop'}
            safe = (0., previous)
        else:
            try:
                scene = motion_scene(raw, history, now, timing_verified=True, epoch=epoch, rear_half_angle_deg=30.)
                output = controller.command(scene, history, measured_speed, 'left', previous, .02, raw_scan=raw)
            except (MotionUnavailable, ValueError) as error:
                controller.reset()
                # local_pursuit.py와 같은 형식(mode 포함). 판단 재생 비교 대상이다.
                output = 0., 0., {'reason': 'follow_input_stop', 'behavior': 'HOLD', 'input_error': str(error),
                                  'mode': 'g1_follow'}
            if trace:
                trace.record_call(raw, history, now, epoch, measured_speed, 'left', previous, .02, True, 30., output)
            command = (0., previous) if scheduled_stop else (float(output[0]), float(output[1]))
            detail = output[2]
            frozen = snapshot_call(raw, history, now, epoch, measured_speed, 'left', previous, .02, True, 30.)
            safe_out, safety = guard.step(frozen, motion, command)
            safe = safe_out if safe_out is not None else (0., previous)
            if not 0 <= safe[0] <= command[0]+1e-12:
                raise ValueError('independent safety increased requested speed')
            status = (safety or {}).get('reason', 'none')
        next_speed = float(np.clip(safe[0], max(0., actual_speed-2.*.02), actual_speed+1.*.02))
        next_steering = float(safe[1])
        ev = evaluation(pose, previous, now, case)
        sides = detail.get('selector_sides') or {}
        records.append(dict(t=round(elapsed, 3), command=list(command), safe=list(safe), safety=status,
            reason=detail.get('reason'), selector_mode=detail.get('selector_mode'),
            selector_reason=detail.get('selector_reason'), selector_choice=detail.get('selector_choice'),
            selector_sides=sides, rear_lane_keep=detail.get('rear_lane_keep'),
            actual_speed_mps=actual_speed, measured_speed_mps=measured_speed,
            evaluation=ev))
        if any(ev[n]['contact'] for n in ('lead', 'side', 'residual') if n in ev):
            break
        physical[-1].update(speed=next_speed, steering=next_steering)
        if not sensors.motion_enabled:
            history.add_mean_wheel(now, next_speed/.0325)
            history.add_gyro(now, next_speed*math.tan(next_steering)/.135)
        travel_m += next_speed*.02
        pose = bicycle_step(*pose, next_speed, next_steering, .02, .135)
        actual_speed, previous = next_speed, next_steering
    if trace:
        trace.close()
    return summarize(name, case, records, scan_states, sensors, geometry_only, ego_y)


def summarize(name, case, records, scan_states, sensors, geometry_only, ego_y):
    sequence = compress([(t, s) for t, s, _ in scan_states])
    duration = case['duration']
    run = [r for r in records if r['t'] < duration]
    start_x = run[0]['evaluation']['center'][0]
    stationary, longest, longest_at = 0., 0., None
    for r in run:
        lead_moving = actor_speed(case['lead'], r['t']) > 0
        if lead_moving and r['actual_speed_mps'] < .01 and r['safe'][0] == 0:
            stationary += .02
            if stationary > longest:
                longest, longest_at = stationary, r['t']
        else:
            stationary = 0.
    start_extent = run[0]['evaluation']['toward_side_extent_m']
    not_ahead = [r for r in run if r['evaluation']['side']['dx'] <= .2]
    modes = compress([(r['t'], r['selector_mode']) for r in run])
    first_blind = next((t for t, s in sequence if s == 'blind'), None)
    # 첫 GUARD 뒤 RELEASED 시각과 그때 평가 상태(사각 진입 전 해제도 그대로 기록).
    guard_at = next((t for t, m in modes if m == 'GUARD'), None)
    release = next((t for t, m in modes if m == 'RELEASED' and guard_at is not None and t > guard_at), None)
    release_state = next((s for t, s, _ in reversed(scan_states) if release is not None and t <= release), None)
    contact = next((dict(t=r['t'], actor=n) for r in records for n in ('lead', 'side', 'residual')
                    if n in r['evaluation'] and r['evaluation'][n]['contact']), None)
    lateral_gaps = [r['evaluation']['side']['gap_m'] for r in run
                    if abs(r['evaluation']['side']['dx']) <= .2]
    summary = dict(case=name, scope='synthetic sensor closed loop; not ROS/Gazebo/hardware',
        geometry_only=geometry_only, sensor_profile=sensors.description(),
        scenario_sequence=sequence, scenario_realized=realized(name, sequence),
        first_blind_s=first_blind, first_release_after_guard_s=release, scan_state_at_release=release_state,
        selector_modes=modes,
        forward_progress_m=run[-1]['evaluation']['center'][0]-start_x,
        final_lateral_m=run[-1]['evaluation']['center'][1], start_lateral_m=ego_y,
        longest_stationary_while_lead_moves_s=round(longest, 3), longest_stationary_end_s=longest_at,
        contact=contact,
        minimum_side_gap_m=min(r['evaluation']['side']['gap_m'] for r in run),
        minimum_alongside_side_gap_m=min(lateral_gaps) if lateral_gaps else None,
        minimum_lead_gap_m=min(r['evaluation']['lead']['gap_m'] for r in run),
        minimum_residual_gap_m=(min(r['evaluation']['residual']['gap_m'] for r in run)
                                if 'residual' in run[0]['evaluation'] else None),
        toward_side_encroachment_m=max(r['evaluation']['toward_side_extent_m'] for r in not_ahead)-start_extent
            if not_ahead else None,
        maximum_wheels_outside_road=max(r['evaluation']['wheels_outside_road'] for r in run),
        stop_command_zero=all(r['command'][0] == 0 and r['safe'][0] == 0 for r in records if r['t'] >= duration),
        reasons=dict(Counter(r['reason'] for r in run)),
        selector_reasons=dict(Counter(r['selector_reason'] for r in run)),
        safety_reasons=dict(Counter(r['safety'] for r in run)),
        safety_reduced=sum(r['safe'][0] < r['command'][0]-1e-9 for r in run),
        calls=len(run), rear_lane_keep_applied=sum(bool((r.get('rear_lane_keep') or {}).get('applied')) for r in run),
        rear_lane_keep_reasons=dict(Counter((r.get('rear_lane_keep') or {}).get('reason') for r in run if r.get('rear_lane_keep'))),
        stop_restart=stop_restart_metrics(case, run))
    return summary, records, scan_states


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cases', nargs='+', default=list(CASES))
    parser.add_argument('--sensor-profile', default='ideal', choices=('ideal', 'range', 'motion', 'combined'))
    parser.add_argument('--geometry-only', action='store_true', help='제어기 없이 장면 성립(부호/배치/레이어)만 확인')
    parser.add_argument('--trace', action='store_true', help='판단 기록 후 자체 재생')
    parser.add_argument('--side-y', type=float, help='진단 sweep 전용: 옆차 횡위치 크기(기본 0.20 m)')
    parser.add_argument('--rear-lane-keep', action='store_true', help='선택형 후방 소실 직후 차선 유지 후보 ON')
    args = parser.parse_args()
    global SIDE_Y
    if args.side_y is not None:
        if not 0 < args.side_y <= .35:
            parser.error('side y outside road/grass sweep range')
        SIDE_Y = args.side_y
    output = args.output.resolve()
    output.mkdir(parents=False, exist_ok=False)
    sources = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(
        [Path(__file__).resolve(), *(ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'),
         ROOT/'scripts/probe_bypass_entry_sequence.py', ROOT/'scripts/bypass_probe_sensors.py',
         ROOT/'scripts/g0_geometry.py', ROOT/'scripts/g2_tracking_probe.py',
         ROOT/'artifacts/validation/2026-10-28/traffic_residual_v1/safety_shadow.py'])}
    report = dict(cases={}, sources=sources, follow_config=HEADER_FOLLOW, motion_config=HEADER_MOTION,
                  side_y_m=SIDE_Y, ego_start_y_magnitude_m=EGO_Y, rear_lane_keep_trial=args.rear_lane_keep)
    for name in args.cases:
        began = time.perf_counter()
        if name not in CASES:
            parser.error('unregistered case: '+name)
        trace_path = output/f'{name}_trace.jsonl' if args.trace and not args.geometry_only else None
        summary, records, states = rollout(name, sensor_profile=args.sensor_profile, trace_path=trace_path,
                                           geometry_only=args.geometry_only, rear_lane_keep=args.rear_lane_keep)
        summary['wall_s'] = round(time.perf_counter()-began, 2)
        if trace_path:
            with trace_path.open(encoding='utf-8') as stream:
                replay = replay_events(json.loads(line) for line in stream)
            summary['self_replay'] = {k: v for k, v in replay.items() if k not in ('outcomes', 'mismatches')}
            summary['self_replay']['first_mismatches'] = (replay.get('mismatches') or [])[:3]
        (output/f'{name}_records.json').write_text(json.dumps(dict(records=records, scan_states=states),
            ensure_ascii=False), encoding='utf-8')
        report['cases'][name] = summary
        print(json.dumps({k: summary[k] for k in ('case', 'scenario_realized', 'forward_progress_m',
            'longest_stationary_while_lead_moves_s', 'contact', 'toward_side_encroachment_m',
            'minimum_side_gap_m', 'first_release_after_guard_s', 'wall_s')}, ensure_ascii=False), flush=True)
    (output/'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding='utf-8')


if __name__ == '__main__':
    main()
