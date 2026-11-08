"""움직이는 상대용 오목 시각 반환 대조 (이 artifact 전용, 평가 전용).

`scripts/audit_actor_outline_returns.py`의 `read_poses`/`interpolate`와 같은 actor 좌표 기하·8 mm 구분 여유를
그대로 쓴다. 바꾼 점은 원 함수의 '상대 정지 후 3.5초부터' 창 대신 (기본) 초록~사례 종료 창을 쓰고, 상대가
움직이므로 각 광선 취득시각을 감싸는 자차/상대 자세 표본 간격을 검사해 공백 광선을 버린다는 것뿐이다.
제어기에 정답을 보내지 않으며, 회전 보정 성능·실물 형상·정밀 접촉의 증명이 아니다.

판정(실행 전 고정): 오목 중앙 반환 >= 2, 그 고유 스캔 >= 2, 오목 중앙 반환 > 옛 외접상자 중앙 옆면 반환,
그리고 자세 시각을 ±10 ms 옮겨도 앞 두 조건이 유지될 것.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from arena_autonomy.follow_trace import thaw
from audit_actor_outline_returns import interpolate, read_poses

EPS = .008          # 원 분석기와 같은 구분 여유
MAX_POSE_GAP_S = .05  # 광선 시각을 감싸는 자세 표본 간격 상한(자차 약1–2 ms, 상대 10 ms 표본)


def bracket_gap(rows, times):
    i = np.clip(np.searchsorted(rows[:, 0], times), 1, len(rows)-1)
    inside = (rows[i-1, 0] <= times) & (times <= rows[i, 0])
    return np.where(inside, rows[i, 0]-rows[i-1, 0], np.inf)


def classify(px, py):
    ax, ay = np.abs(px), np.abs(py)
    near = (ax <= .115) & (ay <= .09)
    center = near & (ax < .06) & (np.abs(ay-.05) <= EPS)
    old_side = near & (ax < .06) & (np.abs(ay-.075) <= EPS)
    ends = near & (ax > .085) & (ax < .098) & (np.abs(ay-.075) <= EPS)
    return near, center, old_side, ends


def rates(rows, first, last):
    keep = (rows[:, 0] >= first) & (rows[:, 0] <= last)
    r = rows[keep]
    dt = np.diff(r[:, 0])
    speed = np.hypot(np.diff(r[:, 1]), np.diff(r[:, 2]))/dt
    yaw_rate = np.diff(r[:, 3])/dt
    return dict(samples=int(len(r)), dt_median_s=float(np.median(dt)), dt_max_s=float(dt.max()),
                speed_median_mps=float(np.median(speed)), speed_max_mps=float(speed.max()),
                yaw_rate_abs_max_rps=float(np.abs(yaw_rate).max()),
                yaw_rate_abs_p99_rps=float(np.percentile(np.abs(yaw_rate), 99)))


def evaluate(run, start_offset_s=0., shifts=(-.01, .01)):
    report = json.loads((run/'report.json').read_bytes())
    green = report['evaluation']['t_green']
    end = report.get('case_window', {}).get('end_sim_s', np.inf)
    ego, actor = read_poses(run/'ego_poses.jsonl'), read_poses(run/'actor_poses.jsonl')
    first = max(ego[0, 0], actor[0, 0], green+start_offset_s)
    last = min(ego[-1, 0], actor[-1, 0], end)
    rays = []
    seen, dropped_gap = set(), 0
    with (run/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        mount = json.loads(next(stream))['data']['motion_config']
        for line in stream:
            event = json.loads(line)
            if event['kind'] != 'call':
                continue
            scan = thaw(event['data']['input']['scan'])
            stamp = scan['sec']+scan['nanosec']*1e-9
            if stamp in seen:
                continue
            seen.add(stamp)
            ranges = np.array(scan['ranges'], dtype=float)
            indices = np.arange(len(ranges))
            times = stamp+indices*scan['time_increment']
            keep = ((times >= first) & (times <= last) & np.isfinite(ranges)
                    & (ranges >= scan['range_min']) & (ranges < scan['range_max']))
            if not keep.any():
                continue
            times, indices, ranges = times[keep], indices[keep], ranges[keep]
            ok = (bracket_gap(ego, times) <= MAX_POSE_GAP_S) & (bracket_gap(actor, times) <= MAX_POSE_GAP_S)
            dropped_gap += int((~ok).sum())
            for t, i, r in zip(times[ok], indices[ok], ranges[ok]):
                rays.append((t, stamp, i, r, scan['angle_min'], scan['angle_increment']))
    if not rays:
        raise ValueError('창 안 광선 없음')
    t, stamps, idx, rng, amin, ainc = (np.array(c) for c in zip(*rays))

    def project(shift):
        return interpolate(ego, t+shift), interpolate(actor, t+shift)

    beam = amin+idx*ainc+mount['lidar_yaw_rad']
    lx = mount['lidar_x_m']+rng*np.cos(beam)
    ly = mount['lidar_y_m']+rng*np.sin(beam)

    def actor_frame(shift):
        ep, ap = project(shift)
        wx = ep[:, 0]+np.cos(ep[:, 2])*lx-np.sin(ep[:, 2])*ly-ap[:, 0]
        wy = ep[:, 1]+np.sin(ep[:, 2])*lx+np.cos(ep[:, 2])*ly-ap[:, 1]
        return np.cos(ap[:, 2])*wx+np.sin(ap[:, 2])*wy, -np.sin(ap[:, 2])*wx+np.cos(ap[:, 2])*wy, ap

    def summary(shift):
        px, py, ap = actor_frame(shift)
        near, center, old_side, ends = classify(px, py)
        cs = np.unique(stamps[center])
        return dict(shift_s=shift, near_actor_returns=int(near.sum()), recessed_center_returns=int(center.sum()),
                    recessed_center_unique_scans=int(len(cs)), old_box_center_side_returns=int(old_side.sum()),
                    old_box_center_side_unique_scans=int(len(np.unique(stamps[old_side]))),
                    outer_end_returns=int(ends.sum())), (px, py, ap, near, center, old_side, ends)

    base, (px, py, ap, near, center, old_side, ends) = summary(0.)
    shifted = [summary(s)[0] for s in shifts]
    cs = np.unique(stamps[center])
    actor_at_center = ap[center]
    span = dict(first_t=float(t[center].min()), last_t=float(t[center].max()),
                actor_displacement_m=float(np.hypot(*(actor_at_center[-1, :2]-actor_at_center[0, :2]))),
                relative_x_range_m=[float(px[center].min()), float(px[center].max())]) if center.any() else None
    observed = (base['recessed_center_returns'] >= 2 and base['recessed_center_unique_scans'] >= 2
                and base['recessed_center_returns'] > base['old_box_center_side_returns'])
    stable = all(s['recessed_center_returns'] >= 2 and s['recessed_center_unique_scans'] >= 2 for s in shifted)
    rows = lambda mask: [dict(t=float(a), scan_stamp_s=float(b), ray=int(c), x=float(d), y=float(e))
                         for a, b, c, d, e in zip(t[mask], stamps[mask], idx[mask], px[mask], py[mask])]
    return dict(
        scope='평가 정답으로 저장 C1 반환을 움직이는 상대 좌표로 옮겨 대조. 제어 입력/실차 외형/정밀 접촉 증명 아님.',
        method='광선별 취득시각으로 자차/actor 평가 자세 선형 보간(원 함수 read_poses/interpolate 재사용), '
               f'창=초록+{start_offset_s}s~사례 종료, 감싸는 자세 표본 간격>{MAX_POSE_GAP_S}s 광선 제외, 같은 스캔 중복 제외.',
        geometry_epsilon_m=EPS, window_s=[float(first), float(last)], unique_scans=len(seen),
        rays_in_window=int(len(t)), rays_dropped_pose_gap=dropped_gap,
        pose_rates=dict(ego=rates(ego, first, last), actor=rates(actor, first, last)),
        **base, shift_checks=shifted, center_span=span,
        recessed_visual_observed=bool(observed), stable_under_time_shift=bool(stable),
        verdict=bool(observed and stable),
        center_scan_stamps=[float(s) for s in cs],
        center_examples=rows(center)[:12], old_side_examples=rows(old_side)[:12], end_examples=rows(ends)[:12],
        sensor_points_actor_frame=rows(near))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--start-offset', type=float, default=0.)
    options = parser.parse_args()
    result = evaluate(options.run, options.start_offset)
    options.output.write_text(json.dumps(result, ensure_ascii=False, indent=1)+'\n', encoding='utf-8', newline='\n')
    print(json.dumps({k: v for k, v in result.items() if k not in (
        'sensor_points_actor_frame', 'center_examples', 'old_side_examples', 'end_examples', 'center_scan_stamps')},
        ensure_ascii=False))
