"""바퀴별 노면 평가기의 저장 궤적 대조와 실제 공식 월드 위 합성 궤적 반례. 평가 전용(정답 사용).

- 저장 궤적: 기존 차체 지표(1 cm² 초과 표본 수)를 같은 식으로 재계산해 원래 감사값과 일치하는지 확인하고,
  바퀴 지표를 별도 열로 붙인다. 기존 판정을 새 기준으로 다시 매기지 않는다.
- actor 자세: 상자 actor의 기록 자세를 팀 차량 기하의 가상 차량으로 평가한다. 실제 바퀴가 아니다.
- 합성 궤적: 실제 SDF 지면 위의 운동학 자세열이며 제어기·물리 주행이 아니다.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in HERE.parents if (p/'src/arena_autonomy').is_dir())
sys.path.insert(0, str(ROOT/'scripts'))
import surface_contact_eval as sc  # noqa: E402

RUNS = {
    'e1_r0_20261024': 'artifacts/validation/2026-10-24/r0_baseline_e1_v1/run',
    'g1_solo_20261016': 'artifacts/validation/2026-10-16/g1_corridor_solo_v1/run',
    'g1_recent_solo_20261017': 'artifacts/validation/2026-10-17/g1_recent_solo_v1/run',
    'e3_harness_g1': 'artifacts/validation/2026-10-24/e3_harness_v1/run',
    'e3_stop_restart_g1': 'artifacts/validation/2026-10-24/e3_stop_restart_v1/run',
    'e3_onroad_g1': 'artifacts/validation/2026-10-24/e3_onroad_v1/g1',
    'e3_onroad_r0plus': 'artifacts/validation/2026-10-24/e3_onroad_v1/r0plus',
    'r0plus_stopped_center': 'artifacts/validation/2026-10-24/r0plus_core_v1/stopped_center',
    'r0plus_stop_restart': 'artifacts/validation/2026-10-24/r0plus_core_v1/stop_restart',
}
WORLD = ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf'


def brief(result):
    keep = ('samples', 'pose_source', 'steering_source', 'first_s', 'last_s', 'max_sample_gap_s', 'sample_gaps_over_limit',
            'body_outside_samples_over_1e-6_m2', 'body_outside_samples_over_1cm2', 'max_body_outside_road_area_m2',
            'steering_footprint_outside_samples_over_1e-6_m2', 'max_steering_footprint_outside_road_area_m2',
            'wall_wheel_samples')
    out = {k: result.get(k) for k in keep}
    out['interpretations'] = {k: {x: v[x] for x in ('max_off_wheels', 'penalty_state_raw_rising_edges',
                                                     'penalty_events_merged', 'penalty_state_time_s', 'any_off_time_s',
                                                     'grass_contact_time_s', 'other_ground_contact_time_s',
                                                     'longest_continuous_grass_contact_s', 'samples_by_off_wheel_count',
                                                     'two_wheel_pair_kinds_samples')}
                              for k, v in result['interpretations'].items()}
    return out


def original_audit(run):
    run = ROOT/run
    audit = run/'road_audit.json'
    if audit.exists():
        data = json.loads(audit.read_text(encoding='utf-8'))
        return {'source': 'road_audit.json', 'outside_samples_over_1cm2': data.get('outside_samples_over_1cm2'),
                'max_outside_road_area_m2': data.get('max_outside_road_area_m2'), 'driving_samples': data.get('driving_samples')}
    report = json.loads((run/'report.json').read_text(encoding='utf-8'))
    road = report.get('road') or (report.get('evaluation') or {}).get('road')
    return {'source': 'report.json road', 'outside_samples': (road or {}).get('outside_samples'),
            'scope': (road or {}).get('scope')}


def stored(surface):
    out = {}
    for name, run in RUNS.items():
        result = sc.evaluate_run(ROOT/run, surface)
        out[name] = {'original_body_audit': original_audit(run), 'surface': brief(result)}
        print(name, json.dumps({k: out[name]['surface'][k] for k in ('samples', 'body_outside_samples_over_1cm2',
                                                                      'steering_footprint_outside_samples_over_1e-6_m2')}),
              json.dumps({k: v['max_off_wheels'] for k, v in out[name]['surface']['interpretations'].items()}), flush=True)
    return out


def actors(surface):
    out = {}
    for name, run in (('e3_harness_side_actor_on_grass', RUNS['e3_harness_g1']),
                      ('e3_onroad_side_actor_on_road', RUNS['e3_onroad_g1'])):
        poses = json.loads((ROOT/run/'additional_actors.json').read_text(encoding='utf-8'))['poses']['eval_e3_side']
        poses = sc.deduplicate([tuple(map(float, p[:4])) for p in poses])
        evaluations = [sc.evaluate_pose(surface, x, y, yaw, 0.) for _, x, y, yaw in poses]
        result = sc.analyze_series([p[0] for p in poses], evaluations)
        out[name] = {'scope': 'box actor recorded poses treated as a virtual vehicle with team wheel geometry; not real wheels',
                     'surface': brief({**result, 'pose_source': 'additional_actors.json', 'steering_source': 'zero'})}
    return out


def centerline():
    cl = np.genfromtxt(WORLD.parent/'centerline.csv', delimiter=',', names=True)
    xy = np.column_stack((cl['x_m'], cl['y_m']))
    t = np.gradient(xy, axis=0)
    heading = np.unwrap(np.arctan2(t[:, 1], t[:, 0]))
    curvature = np.gradient(heading)/np.maximum(1e-9, np.gradient(cl['s_m']))
    return cl['s_m'], xy, heading, curvature


def synthetic_pass(surface, s0, *, side=1., peak=.34, duration=4., speed=.3, dt=.002, noise=None, hold=None):
    """중심선 s0에서 출발해 전진하며 횡위치를 0→peak→0으로 바꾸는 운동학 자세열. 방향은 진행+횡이동 합.

    noise는 판정 표본의 위치 잡음(자세 추정·판정 떨림 모사)에만 더하고 방향 계산에는 넣지 않는다.
    """
    s_arr, xy, heading, _ = centerline()
    times = np.arange(0., duration, dt)
    rng = np.random.default_rng(0)
    if hold is not None:
        smooth = np.full(len(times), hold)
    else:
        smooth = peak*np.sin(np.pi*times/duration)**2
    jitter = rng.uniform(-noise, noise, len(times)) if noise else 0.
    lateral = side*(smooth+jitter)
    s = s0+speed*times
    x = np.interp(s, s_arr, xy[:, 0]); y = np.interp(s, s_arr, xy[:, 1]); h = np.interp(s, s_arr, heading)
    dl = np.gradient(side*smooth, times)
    yaw = h+np.arctan2(dl, speed)
    px = x-np.sin(h)*lateral
    py = y+np.cos(h)*lateral
    evaluations = [sc.evaluate_pose(surface, a, b, c, 0.) for a, b, c in zip(px, py, yaw)]
    return sc.analyze_series(times.tolist(), evaluations)


def synthetic(surface):
    s_arr, xy, heading, curvature = centerline()
    # 직선: |곡률| 최소 구간, 곡선: 최대 곡률 근처, 기타 지면: 갈림길 접합부 공백(7.601, 5.885)에 가장 가까운 중심선
    straight_s = float(s_arr[np.argmin(np.convolve(np.abs(curvature), np.ones(80)/80, mode='same')[100:-100])+100])
    corner_s = float(s_arr[np.argmax(np.abs(curvature))])-.6
    gap_i = int(np.argmin(np.hypot(xy[:, 0]-7.601, xy[:, 1]-5.885)))
    gap_point = np.array([7.601, 5.885])
    normal = np.array([-math.sin(heading[gap_i]), math.cos(heading[gap_i])])
    gap_side = float(np.sign((gap_point-xy[gap_i])@normal))
    cases = {
        'straight_left_ramp_0_to_0.34m': dict(s0=straight_s-.6, side=1.),
        'straight_right_ramp_0_to_0.34m': dict(s0=straight_s-.6, side=-1.),
        'corner_outer_ramp': dict(s0=corner_s, side=-float(np.sign(curvature[np.argmax(np.abs(curvature))]))),
        'corner_inner_ramp': dict(s0=corner_s, side=float(np.sign(curvature[np.argmax(np.abs(curvature))]))),
        'junction_other_ground_ramp': dict(s0=float(s_arr[gap_i])-.6, side=gap_side),
        'flicker_near_three_wheel_threshold': dict(s0=straight_s-.6, side=1., hold=.2865, noise=.003),
        'steady_two_wheel_band_center': dict(s0=straight_s-.6, side=1., hold=.225, noise=.003),
    }
    out = {}
    for name, kwargs in cases.items():
        result = synthetic_pass(surface, **kwargs)
        out[name] = {'parameters': {k: v for k, v in kwargs.items()},
                     'surface': brief({**result, 'pose_source': 'synthetic kinematic', 'steering_source': 'zero'})}
        print(name, json.dumps({k: v['max_off_wheels'] for k, v in result['interpretations'].items()}),
              json.dumps({k: (v['penalty_state_raw_rising_edges'], v['penalty_events_merged']) for k, v in result['interpretations'].items()}),
              flush=True)
    out['_stations'] = {'straight_s': straight_s, 'corner_s': corner_s, 'junction_gap_station_s': float(s_arr[gap_i])}
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    surface = sc.SurfaceMap.from_world(WORLD)
    result = {'scope': 'evaluation-only wheel surface metrics; stored body verdicts preserved; synthetic poses are not driving',
              'world': surface.source, 'geometry': sc.asdict(sc.VehicleGeometry()), 'geometry_status': sc.VALUE_STATUS,
              'stored_runs': stored(surface), 'actor_virtual_vehicles': actors(surface), 'synthetic_official_world': synthetic(surface)}
    consistency = {}
    for name, entry in result['stored_runs'].items():
        original = entry['original_body_audit']
        recomputed = entry['surface']
        if original.get('outside_samples_over_1cm2') is not None:
            consistency[name] = {'original_over_1cm2': original['outside_samples_over_1cm2'],
                                 'recomputed_over_1cm2': recomputed['body_outside_samples_over_1cm2'],
                                 'note': 'original counts started trajectory samples; recomputed counts every pose sample'}
        else:
            consistency[name] = {'original_over_1e-6': original.get('outside_samples'),
                                 'recomputed_over_1e-6': recomputed['body_outside_samples_over_1e-6_m2']}
    result['body_metric_consistency'] = consistency
    result['sources'] = {'surface_contact_eval.py': hashlib.sha256((ROOT/'scripts/surface_contact_eval.py').read_bytes()).hexdigest(),
                         'run_surface.py': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=1, ensure_ascii=False, default=float), encoding='utf-8', newline='\n')
    print(json.dumps(consistency))


if __name__ == '__main__':
    main()
