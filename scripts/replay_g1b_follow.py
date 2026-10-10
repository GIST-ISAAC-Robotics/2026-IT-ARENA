#!/usr/bin/env python3
"""G1-b 보존 scan·MCU 피드백을 LocalFollow에 다시 넣는 open-loop 사후 분석.

실행 당시 기록한 원시 scan(`scans_received.jsonl`)과 가상 MCU가 보고한 평균 바퀴 속도
(`actuation_trace.json`)를 50 Hz 제어 시각에 다시 공급한다. IMU 자이로는 기록하지
않았으므로 0으로 두거나, 평가기 자세의 yaw 변화율을 **분석용 근사**로만 사용한다.
제어 출력은 기록된 센서 흐름을 바꾸지 않으므로 실제 폐루프 재현이 아니다. 실행이
정체한 뒤의 구간은 그 정지 기하에서 제어기가 무엇을 판단하는지만 보여 준다.
제어기·보호층 동작을 바꾸지 않으며 정답 자세를 제어 입력으로 쓰지 않는다.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def load_modules(source_root):
    sys.path[:0] = [str(source_root/'src/arena_autonomy'), str(source_root/'src/arena_vehicle_interface')]
    from arena_autonomy.lidar_motion import MotionConfig, MotionHistory, MotionUnavailable
    from arena_autonomy.local_follow import FollowConfig, LocalFollow
    from arena_autonomy.local_path import local_path, mask_scan
    from arena_autonomy.scene_motion import motion_scene
    return SimpleNamespace(MotionConfig=MotionConfig, MotionHistory=MotionHistory, MotionUnavailable=MotionUnavailable,
                           FollowConfig=FollowConfig, LocalFollow=LocalFollow, local_path=local_path,
                           mask_scan=mask_scan, motion_scene=motion_scene)


def scan_message(row):
    nanos = round(row['stamp_s']*1e9)
    return SimpleNamespace(header=SimpleNamespace(frame_id='laser_frame', stamp=SimpleNamespace(sec=nanos//10**9, nanosec=nanos % 10**9)),
                           ranges=[math.inf if r is None else r for r in row['ranges']], angle_min=row['angle_min'],
                           angle_increment=row['angle_increment'], range_min=.05, range_max=12.,
                           time_increment=row['time_increment'])


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def replay(run, modules, first, last, gyro='zero', side='left', cold_start=(), wheel_delay_s=.005, gyro_delay_s=.004):
    m = modules
    scans = [json.loads(line) for line in (run/'scans_received.jsonl').read_text().splitlines()]
    scans.sort(key=lambda row: row['stamp_s'])
    feedback = sorted({row['feedback_capture_us']: row['mean_wheel_mps'] for row in json.loads((run/'actuation_trace.json').read_text())
                       if row.get('feedback_valid') and row.get('feedback_capture_us') is not None}.items())
    if gyro == 'evaluation_pose_rate':
        ego = np.array([json.loads(line) for line in (run/'ego_poses.jsonl').read_text().splitlines()])
        rate = np.gradient(np.unwrap(ego[:, 3]), ego[:, 0])
        gyro_t = np.arange(max(0., first-1.), last+.5, .005)
        gyro_v = np.interp(gyro_t, ego[:, 0], rate)
    else:
        gyro_t = np.arange(max(0., first-1.), last+.5, .005)
        gyro_v = np.zeros_like(gyro_t)
    cfg = m.FollowConfig(wheelbase_m=.135, max_steering_rad=.37, max_speed_mps=1., wall_offset_m=.425)
    controller = m.LocalFollow(cfg)
    history = m.MotionHistory(m.MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675, lidar_x_m=.06))
    fi = gi = si = 0
    steering = 0.
    rows = []
    cold = sorted(cold_start)
    for k in range(int(round(first/.02)), int(round(last/.02))+1):
        now = k*.02
        while fi < len(feedback) and feedback[fi][0]*1e-6 <= now-wheel_delay_s:
            history.wheels.add(feedback[fi][0]*1e-6, feedback[fi][1])
            fi += 1
        while gi < len(gyro_t) and gyro_t[gi] <= now-gyro_delay_s:
            history.add_gyro(float(gyro_t[gi]), float(gyro_v[gi]))
            gi += 1
        while si+1 < len(scans) and scans[si+1]['stamp_s']+len(scans[si+1]['ranges'])*scans[si+1]['time_increment'] <= now+1e-9:
            si += 1
        row = scans[si]
        if row['stamp_s']+len(row['ranges'])*row['time_increment'] > now+1e-9:
            continue
        while cold and cold[0] <= now+1e-9:
            controller.reset()
            cold.pop(0)
        speed = history.wheels.values[-1] if history.wheels.values else 0.
        try:
            scene = m.motion_scene(m.mask_scan(scan_message(row)), history, now, timing_verified=True)
        except m.MotionUnavailable as error:
            rows.append({'t': round(now, 3), 'scan_s': row['stamp_s'], 'reason': 'input:'+str(error), 'command_mps': 0.})
            controller.reset()
            continue
        command, steering, detail = controller.command(scene, history, speed, side, steering, .02)
        points = np.asarray(scene.occupied_endpoints_xy_m)
        record = {'t': round(now, 3), 'scan_s': row['stamp_s'], 'measured_speed_mps': speed, 'command_mps': command,
                  **{key: detail.get(key) for key in (
                      'reason', 'behavior', 'boundary_selection', 'boundary_candidate_ahead_m', 'fresh_boundary_accepted',
                      'boundary_from_memory', 'boundary_age_s', 'path_wall', 'boundary_support_points',
                      'boundary_protrusions_removed', 'bilateral_reference', 'swept_clearance_m', 'occupied_points')}}
        if detail.get('boundary_selection') is not None:
            for wall in ('left', 'right'):
                path, info = m.local_path(points, wall, cfg.wall_offset_m)
                record[wall+'_unfiltered_ahead_m'] = info.get('boundary_observed_ahead_m') if path is not None else None
        rows.append(record)
    return rows


def summarize(rows):
    reasons = {}
    for row in rows:
        reasons[row['reason']] = reasons.get(row['reason'], 0)+1
    active = [row for row in rows if row['reason'] in ('local_follow', 'follow_release_wait', 'follow_gap_hold')]
    started = active[0]['t'] if active else None
    lost = next((row['t'] for row in rows if started is not None and row['t'] > started
                 and row['reason'] == 'follow_boundary_unconfirmed'), None)
    return {'ticks': len(rows), 'reasons': reasons, 'first_boundary_after_s': started,
            'first_unconfirmed_after_boundary_s': lost,
            'positive_command_ticks': sum(row['command_mps'] > 0 for row in rows),
            'last_reason': rows[-1]['reason'] if rows else None,
            'selections': sorted({row['boundary_selection'] for row in rows if row.get('boundary_selection')})}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--first', type=float, required=True)
    parser.add_argument('--last', type=float, required=True)
    parser.add_argument('--gyro', choices=('zero', 'evaluation_pose_rate'), default='zero')
    parser.add_argument('--side', default='left')
    parser.add_argument('--cold-start', type=float, action='append', default=[],
                        help='이 sim 시각에 LocalFollow를 reset해 무기억 재획득을 본다(분석 조건)')
    parser.add_argument('--source-root', type=Path, default=ROOT,
                        help='비교할 소스 루트(예: 보존 ZIP을 푼 이전 소스). 기본은 현재 작업 트리')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    modules = load_modules(args.source_root.resolve())
    rows = replay(args.run, modules, args.first, args.last, args.gyro, args.side, args.cold_start)
    follow = args.source_root.resolve()/'src/arena_autonomy/arena_autonomy/local_follow.py'
    report = {
        'scope': ('open-loop replay of recorded scans and recorded MCU mean-wheel feedback; gyro not recorded '
                  '(zero or evaluation-pose yaw-rate approximation). Controller output does not change the recorded '
                  'sensor stream, so this is not a closed-loop reproduction.'),
        'run': str(args.run), 'window_s': [args.first, args.last], 'gyro': args.gyro, 'side': args.side,
        'cold_start_s': args.cold_start, 'source_root': str(args.source_root),
        'local_follow_sha256': sha256(follow),
        'inputs_sha256': {name: sha256(args.run/name) for name in ('scans_received.jsonl', 'actuation_trace.json')},
        'summary': summarize(rows), 'rows': rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=1, default=float), encoding='utf-8')
    print(json.dumps(report['summary'], default=float))


if __name__ == '__main__':
    main()
