#!/usr/bin/env python3
"""보존 기록의 진단 요약. 실험 판정은 변경하지 않는다."""
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
from build_experimental_track import obstacle_polygons
from shapely.geometry import LineString, Point
from shapely.strtree import STRtree

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
    if report['case'] in ('side_left', 'side_right'):
        scans = load_rows(run/'scans_received.jsonl')
        ego, actor = load_rows(run/'ego_poses.jsonl'), load_rows(run/'actor_poses.jsonl')
        et, at = [r[0] for r in ego], [r[0] for r in actor]
        def pair_distance(row):
            t, ex, ey, _ = row
            index = bisect_right(at, t)-1
            if index < 0 or t-at[index] > .05:
                return math.inf
            return math.hypot(ex-actor[index][1], ey-actor[index][2])
        closest = min((row for row in ego if first <= row[0] <= stop), key=pair_distance)
        scan = min(scans, key=lambda r: abs(r['stamp_s']+.05-closest[0]))
        walls = obstacle_polygons(run/'runtime_world.sdf')
        tree = STRtree(walls)
        matched, shadow_deltas = [], []
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
            if abs(angle) > math.radians(150.):
                continue  # 제어 입력의 후방 60도 사각을 분석에서 관측했다고 세지 않음.
            x = ex+.06*math.cos(yaw)+distance*math.cos(yaw+angle)-ax
            y = ey+.06*math.sin(yaw)+distance*math.sin(yaw+angle)-ay
            ox, oy = x*math.cos(ayaw)+y*math.sin(ayaw), -x*math.sin(ayaw)+y*math.cos(ayaw)
            # 실제 광선 취득 시각별 평가 자세를 사용한 평면 근사. 20x15 cm 외곽과 3 cm 이내 일치.
            outside = math.hypot(max(abs(ox)-.10, 0.), max(abs(oy)-.075, 0.))
            inside = min(.10-abs(ox), .075-abs(oy))
            distance_to_surface = outside if outside > 0 else inside
            if distance_to_surface <= .03:
                matched.append(math.degrees(angle))
                origin = (ex+.06*math.cos(yaw), ey+.06*math.sin(yaw))
                ray = LineString([origin, (origin[0]+12*math.cos(yaw+angle), origin[1]+12*math.sin(yaw+angle))])
                wall_ranges = []
                for wi in tree.query(ray):
                    hit = ray.intersection(walls[int(wi)].exterior)
                    if not hit.is_empty:
                        wall_ranges.append(Point(origin).distance(hit))
                if wall_ranges and min(wall_ranges) > distance+.03:
                    shadow_deltas.append(min(wall_ranges)-distance)
        result['closest_side_actor_return_diagnostic'] = {
            'closest_center_distance_m': pair_distance(closest), 'closest_sim_s': closest[0], 'scan_first_s': scan['stamp_s'], 'matched_rays': len(matched),
            'matched_bearing_range_deg': [min(matched), max(matched)] if matched else None,
            'actor_return_before_counterfactual_wall_rays': len(shadow_deltas),
            'counterfactual_wall_extra_range_min_max_m': [min(shadow_deltas), max(shadow_deltas)] if shadow_deltas else None,
            'scope': '가장 가까운 옆차 구간의 스캔을 취득 시각별 평가 자세로 변환한 외곽 일치 반환. 높이/roll/pitch는 평면 근사이며 실물 센서 보증이나 자유공간 판정이 아님.',
        }
    return result


if __name__ == '__main__':
    result = {'runs': [summarize(run) for run in sorted(BASE.glob('*_s0')) if (run/'report.json').exists()]}
    result['analysis_source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    with (BASE/'diagnostics_summary.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps(result, ensure_ascii=False, indent=2))
