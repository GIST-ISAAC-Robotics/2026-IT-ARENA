#!/usr/bin/env python3
"""깊이 각도 창의 실제 저장 질의 AB/BA와 원점 거리 진단. 운영 미연결."""
import argparse
import importlib.util
import json
from pathlib import Path
import platform
import sys
import time
import zipfile

import numpy as np

from arena_autonomy import observation_support as current
from arena_autonomy.follow_trace import equal_output, freeze, restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_runtime import digest

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, default=ROOT/'artifacts/validation/2026-10-19/g1_depth_window_v1/candidate.py')
    parser.add_argument('--capture', type=Path, default=ROOT/'artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl')
    parser.add_argument('--repeats', type=int, default=20)
    args = parser.parse_args()
    if args.repeats < 2:
        parser.error('at least two alternating repeats required')
    args.output.mkdir(parents=True, exist_ok=False)
    spec = importlib.util.spec_from_file_location('depth_window_benchmark_candidate', args.candidate)
    candidate = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(candidate)
    before = digest(args.capture)
    sources = {str(args.candidate.resolve().relative_to(ROOT)): digest(args.candidate)}
    for module in tuple(sys.modules.values()):
        filename = getattr(module, '__file__', None)
        if filename:
            p = Path(filename).resolve()
            if p.is_relative_to(ROOT) and p.suffix == '.py':
                sources[p.relative_to(ROOT).as_posix()] = digest(p)
    for name in ('tests/test_observation_depth_window.py', 'scripts/benchmark_depth_window.py'):
        sources[name] = digest(ROOT/name)
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in sources:
            archive.write(ROOT/name, name)
    old = current.depth_support
    cases, matched = [], 0
    with args.capture.open() as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'header':
                motion = MotionConfig(**event['data']['motion_config'])
                follow = LocalFollow(FollowConfig(**event['data']['follow_config']))
            elif event['kind'] == 'reset':
                follow.reset()
            elif event['kind'] == 'call':
                if event['seq'] > 3156:
                    break
                data, scan, history = restore_call(event['data']['input'], motion)
                scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                     epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
                def capture(source, queries, padding):
                    cases.append((event['seq'], source, np.array(queries, copy=True), np.array(padding, copy=True)))
                    return old(source, queries, padding)
                if event['seq'] >= 3151:
                    current.depth_support = capture
                try:
                    answer = follow.command(scene, history, data['speed_mps'], data['side'],
                                            data['previous_steering_rad'], data['dt_s'])
                finally:
                    current.depth_support = old
                assert equal_output(freeze(answer), event['data']['output']), event['seq']
                matched += 1
    if not cases:
        raise RuntimeError('target calls produced no depth queries')
    rows, geometry = [], []
    functions = [('reference', old), ('window', candidate.depth_support)]
    for index, (seq, scene, queries, pad) in enumerate(cases):
        origins = np.array([r.ray_origin_xy_m for r in scene.rays])
        closest = min((float(np.linalg.norm(queries[start:start+128, None]-origins[None], axis=2).min())
                       for start in range(0, len(queries), 128)), default=None)
        geometry.append(dict(case=index, seq=seq, queries=len(queries), nearest_origin_m=closest,
                             singular_origin_possible=closest is not None and closest <= 1e-9))
        expected = old(scene, queries, pad)
        np.testing.assert_array_equal(candidate.depth_support(scene, queries, pad), expected)
        for repeat in range(args.repeats):
            for label, function in (functions if repeat % 2 == 0 else list(reversed(functions))):
                begin = time.perf_counter_ns()
                answer = function(scene, queries, pad)
                elapsed = (time.perf_counter_ns()-begin)/1e6
                np.testing.assert_array_equal(answer, expected)
                rows.append(dict(case=index, seq=seq, queries=len(queries), repeat=repeat, version=label, elapsed_ms=elapsed))
    def summary(data):
        return dict(count=len(data), p50=float(np.percentile(data, 50)), p95=float(np.percentile(data, 95)), maximum=max(data))
    aggregate = {label: summary([r['elapsed_ms'] for r in rows if r['version'] == label and r['queries']])
                 for label, _ in functions}
    per_case = [{**g, 'summary_ms': {label: summary([r['elapsed_ms'] for r in rows if r['version'] == label and r['case'] == g['case']])
                                   for label, _ in functions}} for g in geometry]
    report = dict(scope='same_stored_depth_queries_AB_BA_not_online_latency', input_sha256=before,
        input_unchanged=digest(args.capture) == before, matched=matched, cases=per_case,
        queries=sum(len(q) for _, _, q, _ in cases), exact_booleans_equal=True, summary_ms=aggregate,
        samples=rows, runtime_files_changed=False, online_deadline_verified=False,
        function_restored=current.depth_support is old, source_sha256=sources,
        source_unchanged=all(digest(ROOT/n) == h for n, h in sources.items()),
        platform=platform.platform(), python=sys.version, numpy=np.__version__)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('samples', 'source_sha256', 'cases')}))
    return 0 if report['input_unchanged'] and report['source_unchanged'] and report['function_restored'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
