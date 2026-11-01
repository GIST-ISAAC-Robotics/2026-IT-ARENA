"""실행 기록에서 후보 작동 여부와 주행 요약을 뽑는다. 판정선을 새로 만들지 않는다."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
DAY = HERE.parent
sys.path.insert(0, str(DAY/'race_acceleration_v1'))
from compare import analyze  # noqa: E402


def longest_stall(rows, span=.05):
    """녹색 뒤 진행이 span 미만인 최장 시간. 고밀도 표본이 아니라 궤적 표본 기준이다."""
    t = np.array([r['time'] for r in rows])
    p = np.array([r['progress_m'] for r in rows])
    best, where, left = 0., None, 0
    for right in range(len(t)):
        while p[right]-p[left] >= span:
            left += 1
        if t[right]-t[left] > best:
            best, where = float(t[right]-t[left]), float(p[left])
    return dict(duration_s=best, progress_m=where, span_m=span)


def activity(run):
    header, reacquire, steer, release = None, [], [], []
    zero_after_steer, last_steer = None, None
    with (run/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            if row['kind'] == 'header':
                header = row['data']['follow_config']
            if row['kind'] != 'call':
                continue
            input_, (v, s, detail) = row['data']['input'], row['data']['output']
            now = input_['reference_s']
            if detail.get('bilateral_reference') == 'low_speed_arc':
                reacquire.append(dict(t=now, measured_mps=input_['speed_mps'], accepted=detail.get('fresh_boundary_accepted')))
            if detail.get('reason') == 'follow_steer_in_place':
                steer.append(now)
                last_steer = now
            if last_steer is not None and v > 0:
                release.append(dict(t=now, target_mps=v, steering_rad=s, reason=detail.get('reason')))
                last_steer = None
    return dict(option_in_header=header.get('low_speed_arc_reacquire'), reacquire_calls=len(reacquire), first_reacquire=reacquire[:3],
                steer_in_place_calls=len(steer), steer_window_s=[steer[0], steer[-1]] if steer else None,
                positive_after_steer=release[:5], activated=bool(reacquire or steer))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', type=Path, default=HERE/'run1')
    parser.add_argument('--output', type=Path, default=HERE/'run1_candidate.json')
    options = parser.parse_args()
    run = options.run
    report = json.loads((run/'report.json').read_bytes())
    with (run/'trajectory.jsonl').open(encoding='utf-8') as stream:
        rows = [row for row in map(json.loads, stream) if row['light'] == 'green']
    data = dict(candidate=activity(run), lap=analyze(run), stall=longest_stall(rows),
                imu_rows=sum(1 for row in rows if row.get('imu')), trajectory_rows=len(rows),
                report={k: report.get(k) for k in ('passed', 'error', 'completed_laps', 'progress_m', 'collision_samples', 'stop_stable',
                                                   'shutdown_clean', 'source_unchanged', 'speed_profile', 'peak_ground_speed_mps')},
                video=report.get('video'), signal=(report.get('team_signal_evidence') or {}).get('passed'),
                scope='후보 작동이 없으면 후보 검증으로 세지 않는다. 1회 실행이며 재현성이 아니다.')
    with options.output.open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
    print(json.dumps({k: data[k] for k in ('candidate', 'stall', 'imu_rows', 'report', 'signal')}, ensure_ascii=False)[:1500])


if __name__ == '__main__':
    main()
