"""저장 C1 반환을 평가 정답 자세로 actor 좌표에 옮겨 오목 시각 외형을 확인한다.

평가 전용: 제어기에 정답을 보내지 않는다. 회전 보정 성능 평가가 아니라 모델이
기존 box와 다른 반환을 실제 만들었는지 확인한다. 원래 단위 스캔/광선은 중복 세지 않는다.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from arena_autonomy.follow_trace import thaw


def read_poses(path):
    rows = np.array([json.loads(line)[:4] for line in path.read_bytes().splitlines()], dtype=float)
    if rows.ndim != 2 or rows.shape[1] != 4 or len(rows) < 2 or np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError('평가 자세의 순서/형식 오류')
    rows[:, 3] = np.unwrap(rows[:, 3])
    return rows


def interpolate(rows, times):
    return np.column_stack([np.interp(times, rows[:, 0], rows[:, k]) for k in (1, 2, 3)])


def evaluate(run):
    report = json.loads((run/'report.json').read_bytes())
    green = report['evaluation']['t_green']
    ego, actor = read_poses(run/'ego_poses.jsonl'), read_poses(run/'actor_poses.jsonl')
    first, last = max(ego[0, 0], actor[0, 0], green+3.5), min(ego[-1, 0], actor[-1, 0])
    seen, near, center, ends, old_sides = set(), [], [], [], []
    accepted_scan_stamps = set()
    with (run/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        header = json.loads(next(stream))['data']
        mount = header['motion_config']
        for line in stream:
            event = json.loads(line)
            if event['kind'] != 'call':
                continue
            scan = thaw(event['data']['input']['scan'])
            stamp = scan['sec']+scan['nanosec']*1e-9
            if stamp in seen:
                continue
            seen.add(stamp)
            ranges = np.array(scan['ranges'])
            indices = np.arange(len(ranges))
            times = stamp+indices*scan['time_increment']
            keep = ((times >= first) & (times <= last) & np.isfinite(ranges)
                    & (ranges >= scan['range_min']) & (ranges < scan['range_max']))
            times, indices, ranges = times[keep], indices[keep], ranges[keep]
            if not len(times):
                continue
            ep, ap = interpolate(ego, times), interpolate(actor, times)
            angle = scan['angle_min']+indices*scan['angle_increment']+mount['lidar_yaw_rad']
            lx = mount['lidar_x_m']+ranges*np.cos(angle)
            ly = mount['lidar_y_m']+ranges*np.sin(angle)
            wx = ep[:, 0]+np.cos(ep[:, 2])*lx-np.sin(ep[:, 2])*ly-ap[:, 0]
            wy = ep[:, 1]+np.sin(ep[:, 2])*lx+np.cos(ep[:, 2])*ly-ap[:, 1]
            x = np.cos(ap[:, 2])*wx+np.sin(ap[:, 2])*wy
            y = -np.sin(ap[:, 2])*wx+np.cos(ap[:, 2])*wy
            for t, index, px, py in zip(times, indices, x, y):
                if abs(px) > .115 or abs(py) > .09:
                    continue
                row = dict(t=float(t), scan_stamp_s=stamp, ray=int(index), x=float(px), y=float(py))
                near.append(row)
                # 8 mm는 렌더 취득/자세 표본 근사의 구분 여유. 25 mm 형상 차이보다 작다.
                if abs(px) < .06 and abs(abs(py)-.05) <= .008:
                    center.append(row)
                    accepted_scan_stamps.add(stamp)
                if abs(px) < .06 and abs(abs(py)-.075) <= .008:
                    old_sides.append(row)
                if .085 < abs(px) < .098 and abs(abs(py)-.075) <= .008:
                    ends.append(row)
    return dict(scope='평가 정답으로 저장 반환을 대조. 제어 입력/실차 외형/정밀 접촉 증명 아님.',
                method='광선별 취득시각으로 자차/actor 평가 자세를 선형 보간. 상대 정지 후3.5초부터; 같은 스캔 중복 제외.',
                geometry_epsilon_m=.008, unique_scans=len(seen), near_actor_returns=len(near),
                recessed_center_returns=len(center), recessed_center_unique_scans=len(accepted_scan_stamps),
                outer_end_returns=len(ends), old_box_center_side_returns=len(old_sides),
                recessed_visual_observed=bool(center) and len(accepted_scan_stamps) >= 2,
                center_examples=center[:12], end_examples=ends[:12],
                sensor_points_actor_frame=near)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    options = parser.parse_args()
    result = evaluate(options.run)
    options.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8', newline='\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('sensor_points_actor_frame', 'center_examples', 'end_examples')}, ensure_ascii=False))
