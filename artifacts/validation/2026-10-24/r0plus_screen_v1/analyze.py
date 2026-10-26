"""R0+ 정의 전 제한 선별: 원래 R0 경로가 상대차 면을 따라 바뀌는가.

정지 합성 광선 6조건뿐이다. 제어기 연결·Gazebo·상대차 동역학 시험이 아니다.
"""
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy')]
from g0_geometry import TruthBox, Footprint, cast_rays
from arena_autonomy.local_path import local_path, pursuit_command


def main():
    destination = OUT/'report.json'
    if destination.exists():
        raise FileExistsError('existing bounded screen')
    walls = [TruthBox(0., y, .4, Footprint(8., .05), label=label)
             for y, label in ((.45, 'left_wall'), (-.45, 'right_wall'))]
    walls.append(TruthBox(4., 0., .4, Footprint(.05, .9), label='end_wall'))
    cases = [('clear', []),
             ('front_stopped', [TruthBox(.45, 0., .12)]),
             ('side_parallel', [TruthBox(0., .25, .12)]),
             ('side_front', [TruthBox(.2, .25, .12)]),
             ('side_two_near', [TruthBox(0., .25, .12), TruthBox(.25, .25, .12)]),
             ('front_corner', [TruthBox(.35, .08, .12, yaw_rad=.6)])]
    report = {'scope': 'six static synthetic ray scenes, default R0 path; no state/latency/physics/closed-loop or production change',
              'cases': [], 'source_sha256': {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in (Path(__file__), ROOT/'scripts/g0_geometry.py', ROOT/'src/arena_autonomy/arena_autonomy/local_path.py')}}
    for name, actors in cases:
        raw = cast_rays(walls+actors)
        angles = -math.pi+np.arange(500)*math.tau/500
        ranges = np.asarray(raw['ranges'])
        valid = np.isfinite(ranges) & ~np.asarray(raw['masked'])
        points = np.column_stack((.06+ranges[valid]*np.cos(angles[valid]), ranges[valid]*np.sin(angles[valid])))
        speed, steering, details = pursuit_command(points, 'left', .3, wheelbase=.135, max_speed=1.)
        paths = {}
        for side in ('left', 'right'):
            path, info = local_path(points, side)
            paths[side] = {'reason': info['reason'], 'anchor_distance_m': info.get('wall_distance_m'),
                'observed_ahead_m': info.get('boundary_observed_ahead_m'),
                'nearest_path_y_m': float(path[np.argmin(np.linalg.norm(path, axis=1)), 1]) if path is not None else None}
        report['cases'].append({'case': name, 'r0_speed_mps': speed, 'r0_steering_rad': steering,
            'path_wall': details.get('path_wall'), 'path_reason': details['reason'], 'paths': paths,
            'visible_actor_rays': sum(label == 'target' and not masked for label, masked in zip(raw['truth_labels'], raw['masked']))})
    destination.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps(report['cases'], indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
