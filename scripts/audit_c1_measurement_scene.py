#!/usr/bin/env python3
"""저장된 C1 SDK probe CSV의 순서/각도/개수를 지역 장면까지 보존하는지 검사.

하드웨어·ROS·주행을 실행하지 않는다. 시간은 투영 검사용 합성 상대 시각이며
실물 SDK 시각을 검증하거나 운영 시간 관문을 승인하지 않는다. 공개 출력에는
원시 실내 좌표·각도·장치 식별값을 포함하지 않는다.
"""
import argparse
from collections import Counter
import csv
import hashlib
import itertools
import json
import math
from pathlib import Path
import statistics

from arena_autonomy.local_scene import scene_from_measurements


def audit(scans_path, points_path):
    with scans_path.open(newline='') as stream:
        measured = {int(r['scan']): int(r['count']) for r in csv.DictReader(stream)
                    if r['measured'] == '1' and int(r['result']) == 0 and int(r['count']) > 0}
    if not measured:
        raise ValueError('no measured successful scans')
    seen = set()
    counts, statuses = [], Counter()
    maximum_angle_error = 0.
    with points_path.open(newline='') as stream:
        for scan_id, group in itertools.groupby(csv.DictReader(stream), key=lambda row: int(row['scan'])):
            if scan_id not in measured:
                continue
            if scan_id in seen:
                raise ValueError('split or repeated scan group')
            rows = list(group)
            if len(rows) != measured[scan_id] or [int(r['index']) for r in rows] != list(range(len(rows))):
                raise ValueError('missing/duplicated/reordered source points')
            angles, ranges = [], []
            for row in rows:
                angle, distance, quality = (int(row[k]) for k in ('angle_q14', 'dist_mm_q2', 'quality'))
                if not (0 <= angle <= 65535 and 0 <= distance <= 4294967295 and 0 <= quality <= 255):
                    raise ValueError('SDK field outside storage range')
                # vendor ROS 비반전 표시와 같은 π-angle 좌표 대응. 실장 yaw 교정은 아니다.
                angles.append(math.pi-angle*math.pi/32768.)
                ranges.append(distance/4000. if quality else math.nan)
            stamps = [1.+i*.0002 for i in range(len(rows))]
            scene = scene_from_measurements(ranges, angles_rad=angles, acquired_at_s=stamps,
                t_ref_s=stamps[-1]+.01, range_min=.05, range_max=12., rear_half_angle_deg=0.)
            if len(scene.rays) != len(rows):
                raise AssertionError('measurement count changed')
            for i, ray in enumerate(scene.rays):
                if ray.source_index != i or ray.acquired_at_s != stamps[i]:
                    raise AssertionError('source index/time changed')
                actual = math.atan2(ray.direction_xy[1], ray.direction_xy[0])
                error = abs(math.remainder(actual-angles[i], math.tau))
                maximum_angle_error = max(maximum_angle_error, error)
                if error > 1e-12:
                    raise AssertionError('source direction changed')
                expected_valid = math.isfinite(ranges[i]) and .05 <= ranges[i] <= 12.
                if (ray.status == 'return') != expected_valid:
                    raise AssertionError('invalid measurement became a return')
                if expected_valid and not math.isclose(math.dist(ray.ray_origin_xy_m, ray.hit_xy_m),
                                                      ranges[i], rel_tol=1e-12, abs_tol=1e-12):
                    raise AssertionError('range changed')
                statuses[ray.status] += 1
            seen.add(scan_id)
            counts.append(len(rows))
    if seen != measured.keys():
        raise ValueError('measured scans missing from point file')
    return dict(scan_count=len(counts), measurement_count=sum(counts),
                count_min=min(counts), count_median=statistics.median(counts), count_max=max(counts),
                maximum_angle_error_rad=maximum_angle_error, statuses=dict(statuses),
                counts_indices_angles_ranges_preserved=True,
                timestamps='synthetic relative 200us/point for projection audit only',
                timing_verified=False, driver_or_controller_connected=False,
                inputs_sha256={name: hashlib.sha256(path.read_bytes()).hexdigest()
                               for name, path in [('scans', scans_path), ('points', points_path)]})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scans', type=Path, required=True)
    parser.add_argument('--points', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output exists; preserve previous evidence')
    result = audit(args.scans, args.points)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: result[k] for k in ('scan_count', 'measurement_count', 'count_min',
          'count_median', 'count_max', 'counts_indices_angles_ranges_preserved', 'timing_verified')}))


if __name__ == '__main__':
    main()
