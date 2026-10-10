#!/usr/bin/env python3
"""보존 기록의 진단 요약. 실험 판정은 변경하지 않는다."""
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path

BASE = Path(__file__).resolve().parent


def load_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def summarize(run):
    report = json.loads((run/'report.json').read_text())
    dynamics = load_rows(run/'braking_diagnostics.jsonl')
    first = report['green_observed_sim_s']
    stop = report['stop_request_sim_s']
    active = [r for r in dynamics if first <= r['sim_time_s'] <= stop]
    peak = max(dynamics, key=lambda r: abs(r['wheel_surface_speed_mps']-r['truth_longitudinal_mps']))
    result = {
        'run': run.name, 'preserved_passed': report['passed'],
        'maximum_ground_speed_mps': max(math.hypot(r['truth_longitudinal_mps'], r['truth_lateral_mps']) for r in active),
        'max_wheel_body_mismatch_mps': abs(peak['wheel_surface_speed_mps']-peak['truth_longitudinal_mps']),
        'peak_mismatch_dynamics': peak,
        'diagnostics_max_gap_s': max(b['sim_time_s']-a['sim_time_s'] for a, b in zip(dynamics, dynamics[1:])),
        'input_sha256': {name: hashlib.sha256((run/name).read_bytes()).hexdigest() for name in
            ('report.json', 'braking_diagnostics.jsonl', 'wheel_contacts.jsonl', 'scans_received.jsonl',
             'ego_poses.jsonl', 'actor_poses.jsonl')},
        'limits': '진단 표본의 모형 값이며 실물 슬립이나 완전 가림의 증명이 아니다. 원래 행동/종료 판정을 변경하지 않는다.',
    }
    if report['case'] == 'initial_occlusion':
        scans = load_rows(run/'scans_received.jsonl')
        ego, actor = load_rows(run/'ego_poses.jsonl'), load_rows(run/'actor_poses.jsonl')
        et, at = [r[0] for r in ego], [r[0] for r in actor]
        scan = next(r for r in scans if first+.5 <= r['stamp_s'] <= first+1.)
        matched = []
        for index, distance in enumerate(scan['ranges']):
            if distance is None:
                continue
            t = scan['stamp_s']+index*scan['time_increment']
            ei, ai = bisect_right(et, t)-1, bisect_right(at, t)-1
            if min(ei, ai) < 0 or t-et[ei] > .02 or t-at[ai] > .05:
                raise ValueError('pose coverage missing for diagnostic scan')
            _, ex, ey, yaw = ego[ei]
            _, ax, ay, ayaw = actor[ai]
            angle = scan['angle_min']+index*scan['angle_increment']
            x = ex+.06*math.cos(yaw)+distance*math.cos(yaw+angle)-ax
            y = ey+.06*math.sin(yaw)+distance*math.sin(yaw+angle)-ay
            ox, oy = x*math.cos(ayaw)+y*math.sin(ayaw), -x*math.sin(ayaw)+y*math.cos(ayaw)
            # 2D 근사에서 20x15 cm 앞차 외곽과 일치하는 반환. C1 해상도·자세 오차 여유 3 cm.
            if abs(ox) <= .13 and abs(oy) <= .105 and abs(abs(ox)-.10) <= .03:
                matched.append(math.degrees(angle))
        result['initial_actor_return_diagnostic'] = {
            'scan_first_s': scan['stamp_s'], 'matched_rays': len(matched),
            'matched_bearing_range_deg': [min(matched), max(matched)] if matched else None,
            'scope': '초기 정지 스캔을 평가 자세로 평면 변환한 앞차 외곽 일치 반환. 부분 가림의 정성 확인이며 완전 벽 가림/안전 판정으로 쓰지 않음.',
        }
    return result


if __name__ == '__main__':
    result = {'runs': [summarize(run) for run in sorted(BASE.glob('*_s0')) if (run/'report.json').exists()]}
    result['analysis_source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    with (BASE/'diagnostics_summary.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps(result, ensure_ascii=False, indent=2))
