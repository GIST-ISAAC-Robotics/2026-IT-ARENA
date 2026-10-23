#!/usr/bin/env python3
"""평가 전용: 전역 ±60도 사전 거절과 실제 이동 외곽 지지 실패를 구분한다.

운영 함수를 덮어쓰지 않는다. 고정된 단일 사전 조건만 제거한 독립 함수 객체를
만들며 나머지 관측/점유/TTL/후방 검사는 그대로 둔다. 주행 허가가 아니다.
"""
import argparse
import ast
from collections import Counter
import inspect
import json
from pathlib import Path
import sys
import zipfile

from arena_autonomy import observation_support as obs
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from diagnose_front_unknown import digest, wall_geometry
from probe_corner_visibility_grid import horizontal_segments, steady_scenes

ROOT = Path(__file__).resolve().parents[1]


def diagnostic_without_front_prefilter():
    """소스가 예상 구조일 때만 1개 if-return을 제외. 공유 모듈 monkeypatch 없음."""
    tree = ast.parse(inspect.getsource(obs.sweep_support))
    function = tree.body[0]
    matches = [node for node in function.body if isinstance(node, ast.If) and
               any(isinstance(child, ast.Constant) and child.value == 'current_forward_unobserved'
                   for child in ast.walk(node))]
    if len(matches) != 1 or len(matches[0].body) != 1 or not isinstance(matches[0].body[0], ast.Return) or matches[0].orelse:
        raise ValueError('one known standalone front prefilter required')
    function.body.remove(matches[0])
    isolated = dict(vars(obs))
    exec(compile(ast.fix_missing_locations(tree), 'diagnostic_front_prefilter_only', 'exec'), isolated)
    return isolated['sweep_support']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts') or output == ROOT/'artifacts':
        parser.error('fresh artifact subdirectory required')
    output.mkdir(parents=True, exist_ok=False)
    baseline = json.loads(args.baseline.read_text())
    if baseline['history_mode'] != 'steady-arc' or not baseline['input_unchanged'] or not baseline['source_unchanged']:
        raise ValueError('verified steady-arc baseline required')
    input_hashes = {**baseline['input_sha256'], str(args.baseline): digest(args.baseline)}
    if not all(digest(p) == h for p, h in input_hashes.items()):
        raise ValueError('baseline inputs changed')
    run = ROOT/'artifacts/validation/2026-10-17/g1_recent_solo_v1/run'
    for line in (run/'follow_decisions.jsonl').open():
        e = json.loads(line)
        if e['kind'] == 'header':
            cfg = FollowConfig(**e['data']['follow_config'])
            motion = MotionConfig(**e['data']['motion_config'])
        if e['kind'] == 'call' and e['seq'] == 3501:
            data, raw, _ = restore_call(e['data']['input'], motion)
            break
    _, triangles, _ = wall_geometry(run/'runtime_world.sdf')
    segments = horizontal_segments(triangles, baseline['lidar_plane_z_m'])
    age = baseline['measurement_age_s']
    shadow = diagnostic_without_front_prefilter()
    kwargs = dict(protected_sensor_pose=(motion.lidar_x_m, motion.lidar_y_m, motion.lidar_yaw_rad))
    paths = {Path(__file__).resolve(), ROOT/'tests/test_corner_front_gate.py'}
    for module in tuple(sys.modules.values()):
        name = getattr(module, '__file__', None)
        if name:
            p = Path(name).resolve()
            if p.is_relative_to(ROOT) and p.suffix == '.py':
                paths.add(p)
    source_hashes = {p.relative_to(ROOT).as_posix(): digest(p) for p in sorted(paths)}
    with zipfile.ZipFile(output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in source_hashes:
            archive.write(ROOT/name, name)
    rows, mismatches = [], []
    for row in baseline['rows']:
        for case in row['cases']:
            # 모든 전역 정면 거절과 첫 위치의 대조 6개만 재생한다.
            if case['observation']['reason'] != 'current_forward_unobserved' and row is not baseline['rows'][0]:
                continue
            angle = case['steering_rad']
            sources = steady_scenes(row['world_pose'], segments, raw, motion, cfg, age,
                                    data['rear_half_angle_deg'], angle)
            args_call = (sources, .2, .2, angle, angle, cfg)
            original = obs.sweep_support(*args_call, **kwargs)
            candidate = shadow(*args_call, **kwargs)
            if original != case['observation']:
                mismatches.append(dict(s_m=row['s_m'], lateral_m=row['lateral_m'], steering_rad=angle))
            rows.append(dict(s_m=row['s_m'], lateral_m=row['lateral_m'], steering_rad=angle,
                original=original, without_front_prefilter=candidate,
                sampled_geometry_pass=case['geometry']['sampled_geometry_pass'] and case['history_geometry']['sampled_geometry_pass']))
    targeted = [r for r in rows if r['original']['reason'] == 'current_forward_unobserved']
    report = dict(scope='front_prefilter_diagnostic_not_runtime_policy', rows=rows, baseline_mismatches=mismatches,
        targeted_cases=len(targeted), other_control_cases=len(rows)-len(targeted),
        shadow_reasons=dict(Counter(r['without_front_prefilter']['reason'] for r in targeted)),
        shadow_supported=sum(r['without_front_prefilter']['supported'] for r in targeted),
        runtime_modified=False, new_driving_performed=False, unknown_filled=False,
        source_sha256=source_hashes, source_unchanged=all(digest(ROOT/p) == h for p, h in source_hashes.items()),
        input_sha256=input_hashes, input_unchanged=all(digest(p) == h for p, h in input_hashes.items()))
    (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('rows', 'source_sha256', 'input_sha256')}))
    return 0 if not mismatches and report['source_unchanged'] and report['input_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
