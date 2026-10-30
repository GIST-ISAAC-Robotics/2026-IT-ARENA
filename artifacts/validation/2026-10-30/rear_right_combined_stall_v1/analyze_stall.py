"""10/29 v4 합성 B 오른쪽 복합 오차의 초기 정체를 저장 호출로 분해한다(분석 전용, 운영 코드 변경 없음).

저장된 v4 판단 기록의 입력을 현재 `observed_measured_v1` 런타임으로 다시 계산하고(10/30 v4=런타임 652/652 확인),
평행 후보와 기존 G1 경로의 스윕 검사 함수를 감싸 거리 0에서 차체+광선별 팽창과 겹친 점을 기록한다.
감싸기는 이 분석 프로세스 안에서만 하며 결과 명령은 저장 출력과 대조한다.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface'), str(ROOT/'scripts')]
from arena_autonomy.follow_trace import replay_call, freeze, equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow
import arena_autonomy.local_path as local_path
import arena_autonomy.measured_parallel_speed_trial as mps
import arena_autonomy.occupancy_timing as timing

V4 = ROOT/'artifacts/validation/2026-10-29/rear_lane_preference_shadow_v4'
CAPTURE = {'events': [], 'scene': None, 'padding': None}


def install():
    original_padding = timing.per_ray_padding
    original_mps_padding = mps.per_ray_padding

    def padding(scene, fixed, bound):
        result = original_padding(scene, fixed, bound)
        CAPTURE['scene'], CAPTURE['padding'] = scene, np.asarray(result, dtype=float)
        return result

    def padding_mps(scene, fixed, bound):
        result = original_mps_padding(scene, fixed, bound)
        CAPTURE['scene'], CAPTURE['padding'] = scene, np.asarray(result, dtype=float)
        return result
    timing.per_ray_padding = padding
    mps.per_ray_padding = padding_mps
    original_swept = local_path.swept_limit

    def wrap(source):
        def swept(points, steering, wheelbase=.145, horizon=3., margin=.025, length=.20, width=.15):
            value = original_swept(points, steering, wheelbase, horizon, margin, length, width)
            CAPTURE['events'].append(dict(source=source, points=np.asarray(points, dtype=float),
                                          margin=np.broadcast_to(np.asarray(margin, dtype=float),
                                                                 (len(points),)).copy(),
                                          steering=steering, length=length, width=width, clearance=value))
            return value
        return swept
    mps.swept_limit = wrap('parallel_measured')
    for name in list(sys.modules):
        module = sys.modules[name]
        if name.startswith('arena_autonomy.') and getattr(module, 'swept_limit', None) is original_swept and module is not mps:
            module.swept_limit = wrap(name.split('.')[-1])


def blockers(event):
    """거리 0 자세에서 차체+팽창과 겹친 점. swept_limit의 첫 표본(distance=0)과 같은 식이다."""
    p, m = event['points'], event['margin']
    inside = (np.abs(p[:, 0]) <= event['length']/2+m) & (np.abs(p[:, 1]) <= event['width']/2+m)
    return np.flatnonzero(inside)


def side_slack(event, scene):
    """차체 옆 구간(|x|<=길이/2+팽창) 점의 좌/우 최소 여유 |y|-(반폭+팽창). 음수면 거리 0에서 차단."""
    p, m = event['points'], event['margin']
    beside = np.abs(p[:, 0]) <= event['length']/2+m
    result = {}
    for name, mask in (('left', beside & (p[:, 1] > 0)), ('right', beside & (p[:, 1] < 0))):
        index = np.flatnonzero(mask)
        if not len(index):
            result[name] = None
            continue
        slack = np.abs(p[index, 1])-event['width']/2-m[index]
        k = int(index[int(np.argmin(slack))])
        row = dict(slack_m=float(slack.min()), y=float(p[k, 1]), margin=float(m[k]))
        if scene is not None and CAPTURE['padding'] is not None and len(p) == len(CAPTURE['padding']):
            info = ray_info(scene, k)
            row.update(age_s=info['age_s'], acquired_after_first_s=info['acquired_after_first_s'])
        result[name] = row
    return result


def ray_info(scene, index):
    rays = [ray for ray in scene.rays if ray.status == 'return']
    ray = rays[index]
    dx, dy = ray.direction_xy
    return dict(age_s=scene.t_ref_s-ray.acquired_at_s, source_index=ray.source_index,
                bearing_deg=math.degrees(math.atan2(dy, dx)), scan_first_s=scene.source_first_s,
                acquired_after_first_s=ray.acquired_at_s-scene.source_first_s,
                hit_xy=list(ray.hit_xy_m), origin_xy=list(ray.ray_origin_xy_m))


def main(trace, output, first, last):
    stats, sources = Counter(), Counter()
    rows = []
    with trace.open(encoding='utf-8') as stream:
        header = json.loads(next(stream))['data']
        motion = MotionConfig(**header['motion_config'])
        # v4 합성은 timed 헤더 위에 후보 선택기/평행 후보를 직접 교체했다. 런타임에서는 같은 조합을 명시 정책으로 고른다.
        config = FollowConfig(**{**header['follow_config'], 'rear_lane_keep_trial': True,
                                 'rear_lane_keep_policy': 'observed_measured_v1'})
        controller = LocalFollow(config)
        install()  # 지연 import된 모듈까지 감싸도록 제어기 생성 뒤에 설치한다.
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'reset':
                controller.reset()
                continue
            if event['kind'] != 'call':
                continue
            raw = event['data']
            CAPTURE['events'].clear()
            CAPTURE['scene'] = None
            actual = freeze(replay_call(controller, raw['input'], motion))
            stats['calls'] += 1
            same = equal_output(actual[:2], raw['output'][:2])
            stats['same_command'] += same
            t = raw['input']['reference_s']
            if not first <= t <= last:
                continue
            detail = actual[2]
            row = dict(t=t, speed_mps=raw['input']['speed_mps'], command=list(actual[:2]),
                       reason=detail.get('reason'), mode=detail.get('selector_mode'), same_as_recorded=same,
                       sweeps=[])
            scene = CAPTURE['scene']
            for ev in CAPTURE['events']:
                hit = blockers(ev)
                item = dict(source=ev['source'], clearance=ev['clearance'], steering=ev['steering'],
                            zero_distance_blockers=len(hit), side_slack=side_slack(ev, scene))
                if len(hit):
                    sources[ev['source']] += 1
                    p = ev['points'][hit]
                    # 가장 깊이 들어온 점: 옆면 기준 침투량(차체 반폭+팽창-|y|).
                    depth = ev['width']/2+ev['margin'][hit]-np.abs(p[:, 1])
                    k = int(np.argmax(depth))
                    item.update(deepest=dict(x=float(p[k, 0]), y=float(p[k, 1]), margin=float(ev['margin'][hit][k]),
                                             lateral_penetration_m=float(depth[k])),
                                blocker_y_range=[float(p[:, 1].min()), float(p[:, 1].max())],
                                blocker_x_range=[float(p[:, 0].min()), float(p[:, 0].max())],
                                blocker_side=sorted({'left' if y > 0 else 'right' for y in p[:, 1]}))
                    if (scene is not None and CAPTURE['padding'] is not None
                            and len(ev['points']) == len(CAPTURE['padding'])):
                        item['deepest'].update(ray_info(scene, int(hit[k])))
                row['sweeps'].append(item)
            rows.append(row)
    blocked = [r for r in rows if any(s['zero_distance_blockers'] for s in r['sweeps'])]
    sides = Counter(side for r in blocked for s in r['sweeps'] if s['zero_distance_blockers'] for side in s['blocker_side'])
    slack = {}
    for name in ('left', 'right'):
        values = [s['side_slack'][name] for r in rows for s in r['sweeps']
                  if s['source'] == 'parallel_measured' and s['side_slack'].get(name)]
        if values:
            worst = min(values, key=lambda v: v['slack_m'])
            slack[name] = dict(samples=len(values), negative=sum(v['slack_m'] < 0 for v in values),
                               minimum=worst, mean_acquired_after_first_s=float(np.mean(
                                   [v['acquired_after_first_s'] for v in values if 'acquired_after_first_s' in v] or [math.nan])))
    report = dict(scope='saved v4 synthetic inputs recomputed with runtime observed_measured_v1; analysis-only instrumentation',
                  trace=trace.relative_to(ROOT).as_posix(), trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
                  window_s=[first, last], stats=dict(stats), calls_in_window=len(rows),
                  calls_with_zero_distance_blockers=len(blocked), blocker_sources=dict(sources), blocker_sides=dict(sides),
                  parallel_side_slack=slack,
                  stationary_calls=sum(r['speed_mps'] == 0 for r in rows),
                  zero_command_calls=sum(r['command'][0] == 0 for r in rows),
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    with output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    with output.with_suffix('.rows.jsonl').open('x', encoding='utf-8') as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False)+'\n')
    print(json.dumps(report, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--first', type=float, default=0.)
    parser.add_argument('--last', type=float, default=3.5)
    args = parser.parse_args()
    raise SystemExit(main(args.trace.resolve(), args.output.resolve(), args.first, args.last))
