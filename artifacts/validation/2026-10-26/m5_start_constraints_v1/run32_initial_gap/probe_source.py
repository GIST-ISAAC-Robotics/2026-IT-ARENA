"""저장 M5 입력의 초기 간격/후속 외곽 거절을 분리한다. 출력 변경·주행 없음."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import zipfile

import numpy as np
import yaml

from arena_autonomy.follow_trace import freeze, replay_call, restore_call, motion_scene
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy import m3_execution_trial as m3
from arena_autonomy.occupancy_timing import per_ray_padding
from probe_grid_departure_observation import motion_history
from probe_grid_follow import ROOT, CAMERA, VEHICLE, json_value


def profile(candidate, context, cfg, distance):
    points, ages = context['grid'].returns, context['grid'].ages
    rows = []
    for i in range(0, len(candidate['poses']), 2):
        if candidate['progress'][i] > distance:
            break
        single = dict(candidate, poses=candidate['poses'][i:i+1],
                      progress=candidate['progress'][i:i+1])
        checked = m3.lateral_clearance_check(single, points, ages, context['frame'],
                                            context['corridor'], cfg, distance)
        worst = checked.get('worst')
        if worst:
            worst = dict(worst, range_error_m=cfg.budget.range_bias_m,
                dynamic_m=cfg.budget.dynamic_m(candidate['curved']),
                other_motion_m=worst['required_m']-cfg.budget.range_bias_m-
                               cfg.budget.dynamic_m(candidate['curved']))
        rows.append(dict(progress_m=float(candidate['progress'][i]),
                         margin_m=checked['min_margin_m'], worst=worst))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(); out = args.output.resolve()
    if not out.is_relative_to(ROOT/'artifacts/validation'):
        parser.error('new validation directory required')
    out.mkdir(parents=True, exist_ok=False)
    base = ROOT/'artifacts/validation/2026-10-26/m5_start_constraints_v1/run31_grid_coverage/data'
    paths = [base/name/'report.json' for name in ('grid2_ego2', 'grid5_ego1', 'grid3_ego1')]
    sensor = yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['lidar']
    vehicle = yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']
    wb = vehicle['drivetrain']['wheelbase_m']
    config = FollowConfig(wheelbase_m=wb, occupancy_timing_mode='per_ray_candidate',
        boundary_evidence_mode='corner_distance_candidate', corridor_fallback_enabled=True)
    execution = m3.ExecutionConfig(command_fallback=True, release_to_baseline=True,
        small_center_straight=True, straight_surface_commitment=True, rounded_occupancy=True,
        straight_with_threat=True)
    motion = motion_history(wb, sensor).config
    sources = {Path(__file__), CAMERA, VEHICLE}
    sources.update((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    for module in tuple(sys.modules.values()):
        file = getattr(module, '__file__', None)
        if file:
            p = Path(file).resolve()
            if p.is_relative_to(ROOT) and p.suffix == '.py': sources.add(p)
    frozen = {p.relative_to(ROOT).as_posix(): p.read_bytes() for p in sources}
    with zipfile.ZipFile(out/'sources.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name, raw in frozen.items(): archive.writestr(name, raw)
    records, inputs = [], []
    for path in paths:
        raw = path.read_bytes(); report = json.loads(raw)
        controller = m3.M3ExecutionFollow(config, execution, allow_simulation_trial=True)
        checked_calls, differences = 0, []
        for i, call in enumerate(report['calls'][:151]):
            captured = []
            original = m3.plan_check
            def observed_plan(scene, frames, candidate, context, cfg, ex, speed):
                result = original(scene, frames, candidate, context, cfg, ex, speed)
                if i in (30, 150):
                    captured.append((candidate, context, cfg, result, frames))
                return result
            m3.plan_check = observed_plan
            try:
                actual = replay_call(controller, call['captured_input'], motion)
            finally:
                m3.plan_check = original
            checked_calls += 1
            if freeze(actual) != [call['request_mps'], call['request_steering'], call['metadata']]:
                differences.append(i)
            if i not in (30, 150): continue
            data, scan, history = restore_call(call['captured_input'], motion)
            scene = motion_scene(scan, history, data['reference_s'],
                timing_verified=data['timing_verified'], epoch=data['epoch'],
                rear_half_angle_deg=data['rear_half_angle_deg'])
            # base.command를 다시 호출하지 않는다. 현재 재생에서 얻은 원래 기준 경로를 사용한다.
            # 거절된 후보의 후속 검사도 상태/출력을 바꾸지 않는 순수 명령 평가만 호출한다.
            diagnostic = []
            for candidate, context, cfg, result, frames in captured:
                distance = min(result['committed_progress_m'],
                    result['in_path_free_progress_m'] if result['in_path_free_progress_m'] is not None else np.inf)
                rows = profile(candidate, context, cfg, distance)
                baseline = (0., 0., dict(path_points=context['frame'].p.tolist(),
                                        boundary_age_s=call['metadata'].get('boundary_age_s')))
                padding = per_ray_padding(scene, .025+config.measurement_padding_m,
                                          config.object_motion_bound_mps)
                next_check = controller._evaluate_command(candidate['name'], candidate['path'], scene,
                    frames, history, data['speed_mps'], data['previous_steering_rad'],
                    data['dt_s'], padding, baseline)
                diagnostic.append(dict(name=candidate['name'], plan=result, lateral_profile=rows,
                    downstream_review=next_check,
                    scope='rejected plan remains rejected; downstream pure review never applied'))
            records.append(dict(case=path.parent.name, call=i, t_s=call['t_s'], reason=call['reason'],
                                candidates=diagnostic))
        inputs.append(dict(path=path.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(raw).hexdigest(),
                           unchanged=path.read_bytes()==raw, checked_calls=checked_calls,
                           output_differences=differences))
    unchanged = all((ROOT/n).read_bytes()==raw for n,raw in frozen.items())
    result = dict(records=records, inputs=inputs, sources_unchanged=unchanged,
        source_sha256={n:hashlib.sha256(raw).hexdigest() for n,raw in frozen.items()},
        integrity_pass=unchanged and all(r['unchanged'] and not r['output_differences'] for r in inputs),
        driving_authorized=False, scope='saved-input diagnostic, no commands or risk-budget changes')
    (out/'summary.json').write_text(json.dumps(json_value(result), ensure_ascii=False, indent=2,
                                            allow_nan=False),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('records','source_sha256')}))
    return 0 if result['integrity_pass'] and len(records)==6 else 1


if __name__ == '__main__': raise SystemExit(main())
