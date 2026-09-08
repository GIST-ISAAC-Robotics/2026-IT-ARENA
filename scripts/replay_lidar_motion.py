#!/usr/bin/env python3
"""저장된 실시간 센서 입력에 네 보정을 재적용한다. 새 폐루프 주행 판정 아님.

동일 ROS 시각의 뒤늦은 callback을 먼저 쓴 것처럼 만들지 않기 위해 수신 시각이
제어 시각보다 엄격히 앞선 입력만 사용한다. 원 실행과 경계 표본이 달라질 수 있다.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO/'src/arena_autonomy'))
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory, MotionUnavailable, MODES


def read_rows(path):
    return [json.loads(x) for x in path.read_text().splitlines() if x.strip()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    if not out.is_relative_to(REPO/'artifacts'):
        raise ValueError('artifacts output required')
    out.mkdir(parents=True, exist_ok=False)
    trial = args.trial.resolve()
    report = json.loads((trial/'report.json').read_text())
    h = MotionHistory(MotionConfig(**report['motion_config']))
    events = sorted(read_rows(trial/'motion_inputs.jsonl'), key=lambda r:r['received_s'])
    scans = sorted(read_rows(trial/'scans_raw.jsonl'), key=lambda r:r['received_s'])
    commands = [r for r in read_rows(trial/'commands.jsonl') if r['phase']=='running']
    ei = si = 0
    scan = None
    counts = {m:{'ok':0, 'rejected':{}, 'max_displacement_from_raw_m':0.} for m in MODES}
    rows = []
    for command in commands:
        now = command['time_s']
        while ei < len(events) and events[ei]['received_s'] < now - 1e-9:
            e = events[ei]
            ei += 1
            if not e['accepted']:
                continue
            if e['kind']=='wheels':
                h.add_wheels(e['stamp_s'], *e['values'])
            else:
                h.add_gyro(e['stamp_s'], e['values'][2])  # 독립 시험장 IMU 장착 roll/pitch=0
        while si < len(scans) and scans[si]['received_s'] < now - 1e-9:
            scan = scans[si]
            si += 1
        if scan is None:
            continue
        ranges = [float('nan') if r is None else r for r in scan['ranges']]
        raw = None
        for mode in MODES:
            try:
                points, indices, meta = h.project(ranges, scan['angle_min'], scan['angle_increment'],
                    scan['range_min_m'], scan['range_max_m'], scan['stamp_s'], scan['time_increment_s'], now, mode, True)
                if mode=='none':
                    raw = points
                displacement = float(np.max(np.linalg.norm(points-raw, axis=1))) if len(indices) else 0.
                counts[mode]['ok'] += 1
                counts[mode]['max_displacement_from_raw_m'] = max(counts[mode]['max_displacement_from_raw_m'], displacement)
                rows.append({'time_s':now, 'mode':mode, 'scan_stamp_s':scan['stamp_s'],
                             'matches_recorded_scan':scan['stamp_s']==command['scan_stamp_s'], **meta})
            except MotionUnavailable as error:
                name = str(error)
                counts[mode]['rejected'][name] = counts[mode]['rejected'].get(name, 0)+1
    result = {'passed':all(x['ok'] and not x['rejected'] for x in counts.values()),
              'source_trial':str(trial.relative_to(REPO)), 'counts':counts,
              'contract':'received_s < control_time_s; equal-clock callbacks excluded conservatively',
              'closed_loop_tracking_verified':False}
    (out/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    (out/'transforms.json').write_text(json.dumps(rows, ensure_ascii=False), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
