"""옆차 쪽으로의 자차 횡이동을 부호로 나눠 본다(평가 전용 정답 자세, 제어 입력 아님).

기존 `maximum_ego_road_lateral_change_m`는 절댓값이라 좌우 실행의 방향을 구별하지 못한다. 여기서는 시험 구간
첫 표본 대비 도로 횡위치 변화를 옆차가 있는 쪽(+)과 반대쪽(−)으로 나누고, 최소 간격 시각을 함께 남긴다.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
import g1_traffic_eval as ev
import numpy as np

FIXTURE = ROOT/'artifacts/validation/2026-10-27/official_departure_v2/fixture'


def analyze(run):
    report = json.loads((run/'report.json').read_text(encoding='utf-8'))
    ego = [json.loads(line) for line in (run/'ego_poses.jsonl').read_text().splitlines()]
    actor = json.loads((run/'additional_actors.json').read_text(encoding='utf-8'))['poses']['eval_e3_side']
    route = np.genfromtxt(FIXTURE/'centerline.csv', delimiter=',', names=True)
    scene = json.loads((FIXTURE/'scene.json').read_text(encoding='utf-8'))
    centerline = ev.Centerline(route['x_m'], route['y_m'], route['s_m'], scene['track']['lap_length_m'])
    first = report['case_window']['green_sim_s']
    last = first+report['case_window']['duration_s']

    def lateral(x, y):
        along, _ = centerline.nearest_s(x, y)
        cx, cy, heading = centerline.pose(along)
        return -(x-cx)*math.sin(heading)+(y-cy)*math.cos(heading)
    rows = [r for r in ev.window_tuples(ego, first, last)]
    actor_rows = ev.window_tuples(actor, first, last)
    side = math.copysign(1., lateral(*actor_rows[0][1:3]))
    initial = lateral(*rows[0][1:3])
    times = [a[0] for a in actor_rows]
    series, gaps = [], []
    for t, x, y, yaw in rows:
        series.append((t, side*(lateral(x, y)-initial)))
        i = int(np.searchsorted(times, t, side='right'))-1
        if i >= 0 and t-times[i] <= ev.POSE_MAX_GAP_S:
            _, ax, ay, ayaw = actor_rows[i]
            gaps.append((t, ev.rectangle(x, y, yaw, .20, .15).distance(ev.rectangle(ax, ay, ayaw, .20, .15))))
    toward = max(series, key=lambda r: r[1])
    away = min(series, key=lambda r: r[1])
    closest = min(gaps, key=lambda r: r[1])
    return dict(run=run.relative_to(ROOT).as_posix(), actor_side='left' if side > 0 else 'right',
                initial_ego_road_lateral_m=initial, window_s=[first, last], samples=len(series),
                max_toward_actor_m=toward[1], max_toward_actor_s=toward[0],
                max_away_from_actor_m=-away[1], max_away_from_actor_s=away[0],
                final_toward_actor_m=series[-1][1], minimum_gap_m=closest[1], minimum_gap_s=closest[0],
                input_sha256={name: hashlib.sha256((run/name).read_bytes()).hexdigest()
                              for name in ('report.json', 'ego_poses.jsonl', 'additional_actors.json')})


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = dict(scope='evaluation-only signed lateral motion; +: toward side actor, centerline-relative, sampled poses',
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  runs=[analyze(p.resolve()) for p in args.runs])
    with args.output.open('x', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False))
