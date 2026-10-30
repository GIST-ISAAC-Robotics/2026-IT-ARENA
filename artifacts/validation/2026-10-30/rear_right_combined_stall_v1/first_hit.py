"""후보 합성 기록에서 평행 후보 스윕이 처음 부딪힌 점을 재구성한다(분석 전용).

`probe_beside.py --policy beside`의 판단 기록을 후보 제어기로 다시 계산하고, candidate_speed의 swept_limit 입력을
기록해 여유 ≤ 정지 여유(0.20 m)인 호출에서 첫 간섭 거리의 점(위치·팽창)을 남긴다.
"""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(BASE), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import replay_call, freeze, equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow
import candidate_speed
from candidate_state import MeasuredParallelTrial

EVENTS = []
original = candidate_speed.swept_limit


def recorded(points, steering, wheelbase=.145, horizon=3., margin=.025, length=.20, width=.15):
    value = original(points, steering, wheelbase, horizon, margin, length, width)
    EVENTS.append((np.asarray(points, dtype=float), np.broadcast_to(np.asarray(margin, dtype=float), (len(points),)).copy(),
                   steering, wheelbase, length, width, value))
    return value


candidate_speed.swept_limit = recorded


def hits(points, margin, steering, wheelbase, length, width, distance):
    curvature = math.tan(steering)/wheelbase
    yaw = curvature*distance
    if abs(curvature) < 1e-5:
        rx, ry = distance-wheelbase/2, 0.
    else:
        rx, ry = math.sin(yaw)/curvature-wheelbase/2, (1-math.cos(yaw))/curvature
    dx, dy = points[:, 0]-rx, points[:, 1]-ry
    lx = math.cos(yaw)*dx+math.sin(yaw)*dy-wheelbase/2
    ly = -math.sin(yaw)*dx+math.cos(yaw)*dy
    return np.flatnonzero((np.abs(lx) <= length/2+margin) & (np.abs(ly) <= width/2+margin))


def main(trace, output):
    rows, kinds = [], Counter()
    with trace.open(encoding='utf-8') as stream:
        header = json.loads(next(stream))['data']
        motion = MotionConfig(**header['motion_config'])
        config = replace(FollowConfig(**header['follow_config']), rear_lane_keep_trial=True,
                         rear_lane_keep_policy='observed_measured_v1')
        follow = LocalFollow(config)
        follow.parallel_trial = MeasuredParallelTrial(follow.config)
        follow.reset()
        same = calls = 0
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'reset':
                follow.reset()
                continue
            if event['kind'] != 'call':
                continue
            EVENTS.clear()
            raw = event['data']
            out = freeze(replay_call(follow, raw['input'], motion))
            calls += 1
            same += equal_output(out[:2], raw['output'][:2])
            t = raw['input']['reference_s']
            if t > 8.:
                continue
            for points, margin, steering, wheelbase, length, width, value in EVENTS:
                if value > .20 or len(points) == 0:
                    continue
                if margin.max() <= .0551:  # 옆면 분리 점의 고정 여유 스윕은 따로 표시
                    source = 'beside_fixed'
                else:
                    source = 'age_padded'
                index = hits(points, margin, steering, wheelbase, length, width, value+.04 if value > 0 else 0.)
                if not len(index):
                    continue
                p, m = points[index], margin[index]
                k = int(np.argmin(np.abs(p[:, 1])))
                kind = 'ahead_in_lane' if abs(p[k, 1]) <= width/2 else 'adjacent_lateral_inflation'
                kinds[(source, kind)] += 1
                rows.append(dict(t=t, clearance=value, source=source, kind=kind, x=float(p[k, 0]), y=float(p[k, 1]),
                                 margin=float(m[k]), hits=len(index), speed=raw['input']['speed_mps']))
    report = dict(scope='analysis-only reconstruction of candidate parallel sweep first hits (t<=8 s)',
                  trace=trace.relative_to(ROOT).as_posix(), trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
                  calls=calls, same_command=same, hold_sweeps=len(rows),
                  kinds={f'{a}/{b}': n for (a, b), n in kinds.items()},
                  y_range_m=[min(r['y'] for r in rows), max(r['y'] for r in rows)] if rows else None,
                  margin_range_m=[min(r['margin'] for r in rows), max(r['margin'] for r in rows)] if rows else None,
                  examples=rows[:15], script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    with output.open('x', encoding='utf-8') as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print(json.dumps({k: v for k, v in report.items() if k != 'examples'}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    raise SystemExit(main(args.trace.resolve(), args.output.resolve()))
