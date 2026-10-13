#!/usr/bin/env python3
"""G1 분리 경계 후보의 고정 합성 관측 시험. 차량 이동/제동 검증은 아니다."""
import argparse
from collections import Counter
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time
import zipfile

import numpy as np

from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.local_scene import scene_from_scan
from g0_geometry import Footprint, TruthBox, cast_rays

ROOT = Path(__file__).resolve().parents[1]


def fixed_ranges(kind='side'):
    """정답 형상은 여기에서 거리 생성에만 사용하고 제어기로 보내지 않는다."""
    boxes = [TruthBox(2., sign*.45, .4, Footprint(10., .05), label='wall')
             for sign in (-1, 1)]
    boxes += [TruthBox(x, 0., .4, Footprint(.05, .95), label='end') for x in (-3., 6.)]
    if kind != 'empty':
        boxes.append(TruthBox(.45, .32, .12, label='side_actor'))
    if kind == 'blocked':
        boxes.append(TruthBox(.26, 0., .12, label='central_blocker'))
    if kind not in ('empty', 'side', 'blocked'):
        raise ValueError('unknown fixed geometry')
    # cast_rays의 truth_labels는 반환하지 않는다. 최근접 가림을 유지한다.
    return np.asarray(cast_rays(boxes)['ranges'], dtype=float)


def perturb_ranges(ranges, rng, noise_m=0., bias_m=0., missing_front=False):
    if not all(math.isfinite(v) for v in (noise_m, bias_m)) or noise_m < 0:
        raise ValueError('finite noise/bias required')
    result = np.asarray(ranges, dtype=float).copy()
    valid = np.isfinite(result)
    result[valid] += rng.uniform(-noise_m, noise_m, np.count_nonzero(valid))+bias_m
    if missing_front:
        angles = -math.pi+np.arange(len(result))*math.tau/len(result)
        result[np.abs(angles) <= math.radians(8.)] = np.nan
    # min/max 밖 값을 뒤벽/최대거리로 바꾸지 않는다. scene adapter가 unknown으로 보존한다.
    return result


def fixed_history():
    history = MotionHistory(MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675,
                                         lidar_x_m=.06, history_s=5.))
    for stamp in np.arange(0., 1.51, .01):
        history.add_mean_wheel(float(stamp), 0.)
        history.add_gyro(float(stamp), 0.)
    return history


def run_case(case, mode='fragmented_candidate'):
    controller = LocalFollow(FollowConfig(boundary_evidence_mode=mode))
    history = fixed_history()
    rng = np.random.default_rng(case.get('seed', 0))
    baseline = fixed_ranges(case.get('geometry', 'side'))
    blocked = fixed_ranges('blocked')
    rows, timings = [], []
    frozen = None
    for index, now in enumerate(np.arange(.4, 1.11, .1)):
        fault = case.get('fault')
        active = index >= case.get('fault_after', 0)
        values = blocked if fault == 'new_blocker' and active else baseline
        values = perturb_ranges(values, rng, case.get('noise_m', 0.), case.get('bias_m', 0.),
                                missing_front=fault == 'front_unknown' and active)
        first = now-.1
        if fault == 'stale' and active:
            first = now-.31
        scene = scene_from_scan(values.tolist(), angle_min=-math.pi, angle_increment=math.tau/500,
                                range_min=.05, range_max=12.,
                                acquired_at_s=(first+np.arange(500)*.0002).tolist(), t_ref_s=float(now))
        if fault == 'frozen' and active:
            if frozen is None:
                frozen = scene
            scene = replace(frozen, t_ref_s=float(now))
        feedback = -.01 if fault == 'negative_feedback' and active else 0.
        began = time.perf_counter()
        value, angle, detail = controller.command(scene, history, feedback, 'left')
        timings.append((time.perf_counter()-began)*1000)
        points = len(scene.occupied_endpoints_xy_m)
        rows.append(dict(reference_s=float(now), command_mps=value, steering_rad=angle,
                         reason=detail['reason'], selection=detail.get('boundary_selection'),
                         fault_active=bool(fault and active), input_occupied_points=points,
                         occupancy_checked=detail.get('occupied_points') is not None,
                         output_occupied_points=detail.get('occupied_points'),
                         unfiltered_occupancy=detail.get('unfiltered_occupancy'),
                         source_first_s=scene.source_first_s))
    finite = all(math.isfinite(row['command_mps']) and math.isfinite(row['steering_rad']) for row in rows)
    occupancy = all(row['output_occupied_points'] == row['input_occupied_points'] and
                    row['unfiltered_occupancy'] for row in rows if row['occupancy_checked'])
    expected = case.get('expected', 'observe')
    stop_rows = [r for r in rows if r['fault_active']] if case.get('fault') else rows
    behavior = (all(r['command_mps'] == 0 for r in stop_rows) if expected == 'hold' else
                any(r['command_mps'] > 0 for r in rows) if expected == 'start' else None)
    checks = {'finite_output': finite, 'all_occupancy_retained_when_checked': occupancy}
    if behavior is not None:
        checks['expected_behavior'] = behavior
    return dict(case=case, mode=mode, checks=checks, passed=all(checks.values()),
                behavior_asserted=expected != 'observe',
                positive_commands=sum(r['command_mps'] > 0 for r in rows),
                reasons=dict(Counter(r['reason'] for r in rows)), rows=rows,
                processing_ms=dict(p50=float(np.percentile(timings, 50)),
                                   p95=float(np.percentile(timings, 95)), maximum=max(timings)))


def cases():
    result = [dict(name='empty_clean', geometry='empty', expected='start'),
              dict(name='side_clean', expected='start')]
    for seed in (0, 1, 2):
        for noise, bias, tag in ((.03, 0., 'random'), (0., -.03, 'negative_bias'),
                                 (0., .03, 'positive_bias'), (.03, .03, 'combined')):
            common = dict(seed=seed, noise_m=noise, bias_m=bias)
            result.append(dict(name=f'side_{tag}_{seed}', expected='observe', **common))
            result.append(dict(name=f'blocked_{tag}_{seed}', geometry='blocked', expected='hold', **common))
    for fault in ('front_unknown', 'new_blocker', 'stale', 'negative_feedback'):
        for after in (0, 4):
            result.append(dict(name=f'{fault}_after_{after}', fault=fault, fault_after=after, expected='hold'))
    result.append(dict(name='frozen_before_confirmation', fault='frozen', expected='hold'))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts') or output == ROOT/'artifacts':
        parser.error('fresh artifact subdirectory required')
    output.mkdir(parents=True, exist_ok=False)
    sources = [Path(__file__), ROOT/'scripts/g0_geometry.py', ROOT/'tests/test_fragmented_observation.py']
    sources += [ROOT/'src/arena_autonomy/arena_autonomy'/f'{name}.py' for name in
                ('local_follow', 'boundary_evidence', 'local_path', 'lidar_motion', 'scene_motion',
                 'local_scene', 'lidar_observation')]
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    with zipfile.ZipFile(output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
        for path in sources:
            bundle.write(path, path.relative_to(ROOT))
    results = [run_case(case, mode) for case in cases() for mode in ('connected', 'fragmented_candidate')]
    unchanged = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == digest for p, digest in hashes.items())
    candidate = [r for r in results if r['mode'] == 'fragmented_candidate']
    report = dict(created_at=datetime.now(timezone.utc).isoformat(),
                  scope='fixed_static_sequential_rays_policy_response_not_closed_loop',
                  geometry='straight walls inner y=+-0.425 m; side box x=.45,y=.32; central box x=.26',
                  sensor='500 sequential rays / 10 Hz, lidar (.06,0,.095), 60 degree rear mask',
                  feedback='zero mean wheel and gyro history; negative value is a guard injection only',
                  noise_scope='uniform/bias design probes, not a measured C1 distribution or accuracy guarantee',
                  no_actuator_or_safety_layer_execution=True, ground_truth_controller_input=False,
                  config=asdict(FollowConfig(boundary_evidence_mode='fragmented_candidate')),
                  source_sha256=hashes, source_unchanged=unchanged, results=results,
                  passed=unchanged and all(r['passed'] for r in candidate))
    (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps(dict(passed=report['passed'], cases=len(candidate),
                         failed=[r['case']['name'] for r in candidate if not r['passed']],
                         exploratory_progress={r['case']['name']: r['positive_commands'] for r in candidate
                                               if not r['behavior_asserted']}), ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
