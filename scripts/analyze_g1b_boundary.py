#!/usr/bin/env python3
"""G1-b 기록 스캔을 정지 상태 가정으로 G1 경계 획득 함수에 다시 넣는 사후 분석.

차량이 실제로 정지한 구간(평가기 정답 속도 확인)만 사용한다. 운동 보정은 영속도
이력으로 두므로 주행 중 구간의 재현이 아니다. 제어기 동작을 바꾸지 않는다.
"""
import argparse
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory  # noqa: E402
from arena_autonomy.local_follow import FollowConfig, LocalFollow, path_agrees  # noqa: E402
from arena_autonomy.local_path import local_path, mask_scan  # noqa: E402
from arena_autonomy.scene_motion import motion_scene  # noqa: E402


def scan_message(row):
    nanos = round(row['stamp_s']*1e9)
    return SimpleNamespace(header=SimpleNamespace(frame_id='laser_frame', stamp=SimpleNamespace(sec=nanos//10**9, nanosec=nanos % 10**9)),
                           ranges=[math.inf if r is None else r for r in row['ranges']], angle_min=row['angle_min'],
                           angle_increment=row['angle_increment'], range_min=.05, range_max=12.,
                           time_increment=row['time_increment'])


def summarize(path, info):
    keys = ('reason', 'boundary_observed_ahead_m', 'wall_distance_m', 'boundary_samples')
    return {k: info.get(k) for k in keys} | {'path_found': path is not None}


def synthetic_rows(ego, actor, count=3, samples=500):
    """공식 SDF 충돌체 평면 근사 + actor 직사각형에 대한 이상적 2D 광선. 렌더링 재현이 아님."""
    from shapely.geometry import LineString, Point
    from shapely.strtree import STRtree
    sys.path.insert(0, str(ROOT/'scripts'))
    from build_experimental_track import obstacle_polygons, rectangle
    polygons = obstacle_polygons(ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf')
    polygons.append(rectangle(actor[0], actor[1], actor[2], .20, .15))
    tree = STRtree(polygons)
    x, y, yaw = ego
    origin = (x+.06*math.cos(yaw), y+.06*math.sin(yaw))
    ranges = []
    for i in range(samples):
        bearing = yaw-math.pi+i*math.tau/samples
        ray = LineString([origin, (origin[0]+12*math.cos(bearing), origin[1]+12*math.sin(bearing))])
        best = math.inf
        for index in tree.query(ray):
            hit = ray.intersection(polygons[int(index)].exterior)
            if not hit.is_empty:
                best = min(best, Point(origin).distance(hit))
        ranges.append(best if math.isfinite(best) and best >= .05 else None)
    return [{'stamp_s': 10.+k*.1, 'time_increment': .1/samples, 'angle_min': -math.pi,
             'angle_increment': math.tau/samples, 'ranges': ranges} for k in range(count)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path, nargs='?')
    parser.add_argument('--first', type=float, default=-math.inf, help='정지 구간 시작 sim 초(평가기 확인)')
    parser.add_argument('--last', type=float, default=math.inf)
    parser.add_argument('--synthetic', type=float, nargs=6, metavar=('EGO_X', 'EGO_Y', 'EGO_YAW', 'ACTOR_X', 'ACTOR_Y', 'ACTOR_YAW'),
                        help='기록 대신 공식 충돌체 평면 근사로 정지 스캔을 만든다')
    parser.add_argument('--side', default='left')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.synthetic:
        rows = synthetic_rows(args.synthetic[:3], args.synthetic[3:])
    else:
        rows = [json.loads(line) for line in (args.run/'scans_received.jsonl').read_text().splitlines()]
        rows = [r for r in rows if args.first <= r['stamp_s'] <= args.last]
    cfg = FollowConfig()
    history = MotionHistory(MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-cfg.wheelbase_m/2, lidar_x_m=.06))
    controller = LocalFollow(cfg)
    results = []
    t = rows[0]['stamp_s']-.5 if rows else 0.
    for row in rows:
        scan = mask_scan(scan_message(row))
        now = row['stamp_s']+len(row['ranges'])*row['time_increment']+.01
        while t <= now:
            history.add_mean_wheel(t, 0.)
            history.add_gyro(t, 0.)
            t += .01
        scene = motion_scene(scan, history, now, timing_verified=True)
        points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float)
        left, left_info = local_path(points, 'left', cfg.wall_offset_m)
        right, right_info = local_path(points, 'right', cfg.wall_offset_m)
        speed, steering, detail = controller.command(scene, history, 0., args.side, 0., .02)
        front = points[(points[:, 0] > .05) & (np.abs(points[:, 1]) < .2)]
        results.append({'scan_stamp_s': row['stamp_s'], 'left': summarize(left, left_info),
                        'right': summarize(right, right_info),
                        'left_right_agree': bool(left is not None and right is not None
                                             and path_agrees(left, right, cfg.boundary_agreement_m)),
                        'nearest_front_return_x_m': float(front[:, 0].min()) if len(front) else None,
                        'controller': {k: detail.get(k) for k in ('behavior', 'reason', 'boundary_candidate_reason',
                                       'boundary_candidate_ahead_m', 'boundary_selection', 'swept_clearance_m')}})
    report = {'scope': 'stationary-period re-evaluation of recorded scans with zero-motion history; not a moving replay',
              'run': str(args.run), 'window_s': [args.first, args.last], 'scans': len(results), 'results': results}
    args.output.write_text(json.dumps(report, indent=1, default=float), encoding='utf-8')
    for r in results[:6]+results[-3:]:
        print(json.dumps(r, default=float))


if __name__ == '__main__':
    main()
