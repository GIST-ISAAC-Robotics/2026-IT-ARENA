"""저장 E3 자세의 간격 변화 분해. 평가 전용이며 제어·원인 책임 판정이 아니다.

두 자세 흐름을 기존 평가기와 같은 과거 최근 표본 방식으로 결합한다. 각 구간에서
자차/상대차를 어느 순서로 바꾸는지에 따른 두 분해를 평균한다. 합은 실제 표본
간격 변화와 일치하지만, 실제로 실행 가능한 반사실 주행이나 사고 책임은 아니다.
"""
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys

import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'src/arena_autonomy').is_dir())
sys.path.insert(0, str(ROOT / 'scripts'))
import g1_traffic_eval as ev

ACTOR = 'eval_e3_side'
LENGTH, WIDTH = .20, .15
GAP = .15


def split_change(d00, d10, d01, d11):
    """dij: ego i, actor j의 다각형 거리. 두 갱신 순서를 대칭적으로 평균한다."""
    values = (d00, d10, d01, d11)
    if not all(math.isfinite(v) and v >= 0 for v in values):
        raise ValueError('nonfinite or negative polygon distance')
    ego = ((d10-d00)+(d11-d01))/2
    actor = ((d01-d00)+(d11-d10))/2
    return ego, actor


def half_span(yaw):
    return (LENGTH*abs(math.sin(yaw))+WIDTH*abs(math.cos(yaw)))/2


def polygon(row):
    return ev.rectangle(*row[1:], LENGTH, WIDTH)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def join_poses(ego, actor, first, last):
    if not ev._pose_rows_valid(ego) or not ev._pose_rows_valid(actor):
        raise ValueError('invalid pose stream')
    times = [row[0] for row in actor]
    result = []
    for e in ego:
        if not first <= e[0] <= last:
            continue
        i = bisect_right(times, e[0])-1
        if i < 0 or e[0]-times[i] > ev.POSE_MAX_GAP_S:
            raise ValueError('missing as-of actor pose')
        result.append((e, actor[i]))
    if len(result) < 2:
        raise ValueError('insufficient samples')
    if any(b[0][0]-a[0][0] > ev.POSE_MAX_GAP_S for a, b in zip(result, result[1:])):
        raise ValueError('ego pose gap')
    return result


def evaluate(mode):
    base = ROOT / 'artifacts/validation/2026-10-24/e3_onroad_v1' / mode
    paths = [base / n for n in ('report.json', 'ego_poses.jsonl', 'additional_actors.json')]
    saved = json.loads(paths[0].read_text(encoding='utf-8'))
    ego = [json.loads(line) for line in paths[1].read_text().splitlines()]
    actors = json.loads(paths[2].read_text())
    actor = actors['poses'][ACTOR]
    first = saved['case_window']['green_sim_s']
    last = first+saved['case_window']['duration_s']
    joined = join_poses(ego, actor, first, last)
    # 고정 도로축은 등록 상대차의 첫 자세로만 잡으며, 이 직선 장면에 한정한다.
    ax0, ay0, heading = actor[0][1:]
    nx, ny = -math.sin(heading), math.cos(heading)
    config_path = ROOT / 'config/tests/e3_onroad_encounter_v1.json'
    config = json.loads(config_path.read_text(encoding='utf-8'))
    spec = next(a for a in config['additional_actors'] if a['model_name'] == ACTOR)
    assert spec['route_kind'] == 'side_road_straight' and spec['minimum_body_gap_m'] == GAP
    offset = spec['lateral_offset_m']

    def lateral(p):
        return (p[1]-ax0)*nx+(p[2]-ay0)*ny+offset

    def yaw(p):
        return math.remainder(p[3]-heading, math.tau)

    e0 = joined[0][0]
    rows = []
    cumulative_e = cumulative_a = 0.
    max_identity_error = 0.
    old_ep = old_ap = None
    for e, a in joined:
        ep, ap = polygon(e), polygon(a)
        gap = ep.distance(ap)
        de = da = 0.
        if old_ep is not None:
            de, da = split_change(old_ep.distance(old_ap), ep.distance(old_ap), old_ep.distance(ap), gap)
            max_identity_error = max(max_identity_error, abs(de+da-(gap-rows[-1]['gap_m'])))
        cumulative_e += de
        cumulative_a += da
        e_lat, a_lat = lateral(e), lateral(a)
        # signed projection: 종방향으로 떨어져 있어도 음수가 될 수 있다. 접촉 판정이 아니다.
        signed = a_lat-e_lat-half_span(yaw(e))-half_span(yaw(a))
        row = dict(t=e[0], actor_pose_age_s=e[0]-a[0], gap_m=gap,
                   ego_distance_delta_m=de, actor_distance_delta_m=da,
                   ego_distance_cumulative_m=cumulative_e, actor_distance_cumulative_m=cumulative_a,
                   ego_lateral_m=e_lat, actor_lateral_m=a_lat, ego_yaw_from_road_rad=yaw(e),
                   signed_lateral_projection_m=signed,
                   ego_lateral_loss_from_start_m=e_lat-lateral(e0),
                   ego_rotation_loss_from_start_m=half_span(yaw(e))-half_span(yaw(e0)),
                   ego_rotation_loss_from_parallel_m=half_span(yaw(e))-WIDTH/2)
        rows.append(row)
        old_ep, old_ap = ep, ap
    total_error = abs(cumulative_e+cumulative_a-(rows[-1]['gap_m']-rows[0]['gap_m']))
    assert max_identity_error <= 1e-9 and total_error <= 1e-9
    below = next((r for r in rows if r['gap_m'] < GAP), None)
    original_encounter = saved['additional_actors']['actors'][ACTOR]['encounter']
    assert below and abs(below['t']-original_encounter['first_below_gap_s']) <= 1e-9
    assert len(rows) == original_encounter['samples']
    minimum = min(rows, key=lambda r: r['gap_m'])
    assert abs(minimum['gap_m']-original_encounter['minimum_gap_m']) <= 1e-9
    contacts = saved['additional_actors']['contacts']['events']
    contact_times = [r['t'] for r in contacts if any('arena_car::' in p for p in r['pair'])
                     and any(ACTOR+'::' in p for p in r['pair'])]
    # 사전 지정 구간 경계. 관측·후보의 사건 시각은 기존 보고서에서 가져온 평가 참고값이다.
    breaks = ([first, 3.92, 4.22, 5.44, 6.52, 7.0, last] if mode == 'g1' else
              [first, 3.86, 4.321, 5.22, 10.4, 13.6, 14.8, 14.997, last])
    times = [r['t'] for r in rows]
    indices = sorted(set(max(0, bisect_right(times, t)-1) for t in breaks))
    segments = []
    for i, j in zip(indices, indices[1:]):
        r0, r1 = rows[i], rows[j]
        de = r1['ego_distance_cumulative_m']-r0['ego_distance_cumulative_m']
        da = r1['actor_distance_cumulative_m']-r0['actor_distance_cumulative_m']
        assert abs(de+da-r1['gap_m']+r0['gap_m']) <= 1e-9
        segments.append(dict(first_s=r0['t'], last_s=r1['t'], samples=j-i,
                             gap_before_m=r0['gap_m'], gap_after_m=r1['gap_m'],
                             ego_distance_change_m=de, actor_distance_change_m=da,
                             ego_lateral_loss_m=r1['ego_lateral_m']-r0['ego_lateral_m'],
                             ego_rotation_loss_m=half_span(r1['ego_yaw_from_road_rad'])-
                                                 half_span(r0['ego_yaw_from_road_rad'])))
    sanity = {}
    if mode == 'g1':
        prior_path = ROOT / 'artifacts/validation/2026-10-24/e3_gap_budget_v1/report.json'
        prior = json.loads(prior_path.read_text())
        at = rows[max(0, bisect_right(times, prior['time_s'])-1)]
        # 이전 계산은 그 자세에서의 중심선 접선을 사용했다. 위 고정 actor 축과
        # 0.001 rad가량 다르므로 서로 다른 좌표 정의를 억지로 일치시키지 않는다.
        world = ROOT / 'src/arena_gazebo/worlds/it_arena_official'
        route_path, scene_path = world/'centerline.csv', world/'scene.json'
        route = np.genfromtxt(route_path, delimiter=',', names=True)
        scene = json.loads(scene_path.read_text(encoding='utf-8'))
        line = ev.Centerline(route['x_m'], route['y_m'], route['s_m'], scene['track']['lap_length_m'])
        e = joined[max(0, bisect_right(times, prior['time_s'])-1)][0]
        along, _ = line.nearest_s(e[1], e[2])
        cx, cy, local_heading = line.pose(along)
        local_lat = -(e[1]-cx)*math.sin(local_heading)+(e[2]-cy)*math.cos(local_heading)
        local_yaw = math.remainder(e[3]-local_heading, math.tau)
        initial_lat = prior['rows'][0]['ego_y_m']
        translation = local_lat-initial_lat
        rotation = half_span(local_yaw)-WIDTH/2
        errors = [abs(translation-prior['translation_loss_m']), abs(rotation-prior['rotation_loss_m'])]
        assert max(errors) <= .0001
        sanity = dict(at_s=at['t'], translation_m=translation, yaw_half_span_m=rotation,
                      max_difference_from_previous_m=max(errors),
                      fixed_axis_translation_m=at['ego_lateral_m']-initial_lat,
                      fixed_axis_yaw_half_span_m=at['ego_rotation_loss_from_parallel_m'],
                      local_vs_fixed_axis_rad=local_heading-heading,
                      note='prior local-centerline frame reproduced separately; fixed-axis timeline is a different definition')
        paths += [prior_path, route_path, scene_path]
    paths.append(config_path)
    return dict(mode=mode, original_passed=saved['passed'], gap_threshold_unchanged_m=GAP,
                samples=len(rows), first_below_gap_s=below['t'], minimum=minimum,
                first_contact_sensor_s=min(contact_times) if contact_times else None,
                max_actor_pose_age_s=max(r['actor_pose_age_s'] for r in rows),
                max_identity_error_m=max_identity_error, cumulative_identity_error_m=total_error,
                segments=segments, selected_samples=[rows[i] for i in indices], prior_sanity=sanity,
                source_hashes={str(p.relative_to(ROOT)): sha(p) for p in paths})


def main():
    if len(sys.argv) != 2:
        raise SystemExit('usage: analyze.py NEW_OUTPUT_DIRECTORY')
    out = Path(sys.argv[1]).resolve()
    out.mkdir(parents=True, exist_ok=False)
    for name in ('analyze.py', 'test_attribution.py', 'run_once.sh'):
        source = Path(__file__).with_name(name)
        shutil.copy2(source, out / name)
    results = [evaluate(mode) for mode in ('g1', 'r0plus')]
    hashes = {str(p.relative_to(ROOT)): sha(p) for p in
              (Path(__file__).resolve(), ROOT/'scripts/g1_traffic_eval.py')}
    for r in results:
        for rel, digest in r['source_hashes'].items():
            assert sha(ROOT/rel) == digest
    report = dict(scope='evaluation only: symmetric distance identity; not causal blame, feasible counterfactual, '
                        'closed-loop validation or a changed pass verdict',
                  sign='negative contribution closes sampled polygon distance; positive opens it',
                  join='last observed actor pose at or before each ego timestamp, max age 0.05 s; no interpolation',
                  limitations=['set_pose batching and as-of join prevent instantaneous velocity/causal attribution',
                               'polygon distance saturates at zero in overlap; contact uses original sensor separately',
                               'signed lateral projection is not polygon gap or collision',
                               'straight fixed road axis and nominal 20x15 cm boxes only'],
                  traces=results, script_hashes=hashes)
    (out/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+'\n',
                                  encoding='utf-8')
    print(json.dumps({r['mode']: {k: r[k] for k in ('samples', 'first_below_gap_s', 'first_contact_sensor_s',
                                                  'max_identity_error_m', 'segments', 'prior_sanity')}
                      for r in results}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
