"""실제 6호출의 깊이 질의를 회수하고 전체 광선/상계 창의 AB/BA 비용을 대조."""
import json
from pathlib import Path
import sys
import time
import zipfile

import numpy as np
import candidate
from arena_autonomy import observation_support as current
from arena_autonomy.follow_trace import equal_output, freeze, restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_runtime import digest

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
CAPTURE = ROOT/'artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl'


def main():
    assert not (OUT/'report.json').exists()
    before = digest(CAPTURE)
    sources = {}
    for module in tuple(sys.modules.values()):
        filename = getattr(module, '__file__', None)
        if filename:
            p = Path(filename).resolve()
            if p.is_relative_to(ROOT) and p.suffix == '.py':
                sources[p.relative_to(ROOT).as_posix()] = digest(p)
    name = 'tests/test_observation_depth_window.py'
    sources[name] = digest(ROOT/name)
    with zipfile.ZipFile(OUT/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in sources:
            archive.write(ROOT/name, name)
    old = current.depth_support
    cases, matched = [], 0
    with CAPTURE.open() as stream:
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
    rows = []
    functions = [('reference', old), ('window', candidate.depth_support)]
    for index, (seq, scene, queries, pad) in enumerate(cases):
        expected = old(scene, queries, pad)
        np.testing.assert_array_equal(candidate.depth_support(scene, queries, pad), expected)
        for repeat in range(20):
            for label, function in (functions if repeat % 2 == 0 else list(reversed(functions))):
                begin = time.perf_counter_ns()
                answer = function(scene, queries, pad)
                elapsed = (time.perf_counter_ns()-begin)/1e6
                np.testing.assert_array_equal(answer, expected)
                rows.append(dict(case=index, seq=seq, queries=len(queries), repeat=repeat, version=label, elapsed_ms=elapsed))
    summary = {}
    for label, _ in functions:
        costs = [r['elapsed_ms'] for r in rows if r['version'] == label and r['queries']]
        summary[label] = dict(count=len(costs), p50=float(np.percentile(costs, 50)),
                              p95=float(np.percentile(costs, 95)), maximum=max(costs))
    report = dict(scope='angle_window_same_real_queries_AB_BA_not_latency',
        input_sha256=before, input_unchanged=digest(CAPTURE) == before, matched=matched,
        cases=len(cases), queries=sum(len(q) for _, _, q, _ in cases), exact_booleans_equal=True,
        summary_ms=summary, samples=rows, runtime_files_changed=False, online_deadline_verified=False,
        function_restored=current.depth_support is old, source_sha256=sources,
        source_unchanged=all(digest(ROOT/n) == h for n, h in sources.items()))
    (OUT/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('samples', 'source_sha256')}))
    return 0 if report['input_unchanged'] and report['source_unchanged'] and report['function_restored'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
