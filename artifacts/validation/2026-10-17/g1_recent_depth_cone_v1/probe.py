"""실제 보완 6판단 내부 깊이 질의의 동등성과 함수 AB/BA 비용. 운영 미연결."""
import json
from pathlib import Path
import sys
import time
import zipfile

import numpy as np
import candidate
from arena_autonomy import observation_support as current
from arena_autonomy import observation_lateral, corridor_observer
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
    names = {}
    for module in tuple(sys.modules.values()):
        name = getattr(module, '__file__', None)
        if name:
            p = Path(name).resolve()
            if p.is_relative_to(ROOT) and p.suffix == '.py':
                names[p.relative_to(ROOT).as_posix()] = digest(p)
    with zipfile.ZipFile(OUT/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in names:
            archive.write(ROOT/name, name)
    old = current.depth_support
    cases, matched = [], 0
    for line in CAPTURE.open():
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
            def capture(scene, queries, padding):
                cases.append((event['seq'], scene, np.array(queries, copy=True), np.array(padding, copy=True)))
                return old(scene, queries, padding)
            if event['seq'] >= 3151:
                current.depth_support = capture
            try:
                output = follow.command(scene, history, data['speed_mps'], data['side'],
                    data['previous_steering_rad'], data['dt_s'])
            finally:
                current.depth_support = old
            assert equal_output(freeze(output), event['data']['output']), event['seq']
            matched += 1
    rows = []
    for index, (seq, scene, queries, pad) in enumerate(cases):
        expected = old(scene, queries, pad)
        new = candidate.depth_support(scene, queries, pad)
        assert np.array_equal(expected, new), (index, seq)
        for repeat in range(5):
            order = [('old', old), ('candidate', candidate.depth_support)]
            if repeat % 2:
                order.reverse()
            for label, fn in order:
                start = time.perf_counter_ns()
                answer = fn(scene, queries, pad)
                cost = (time.perf_counter_ns()-start)/1e6
                assert np.array_equal(answer, expected)
                rows.append(dict(case=index, seq=seq, queries=len(queries), repeat=repeat, version=label, elapsed_ms=cost))
    summary = {}
    for label in ('old', 'candidate'):
        vals = [r['elapsed_ms'] for r in rows if r['version'] == label and r['queries']]
        summary[label] = dict(count=len(vals), p50=float(np.percentile(vals, 50)), p95=float(np.percentile(vals, 95)), maximum=max(vals))
    report = dict(scope='same_real_queries_depth_only_AB_BA_not_runtime', matched=matched,
        cases=len(cases), empty_cases=sum(len(q) == 0 for _, _, q, _ in cases),
        queries=sum(len(q) for _, _, q, _ in cases), exact_booleans_equal=True, samples=rows, summary_ms=summary,
        input_sha256=before, input_unchanged=before == digest(CAPTURE), source_sha256=names,
        source_unchanged=all(digest(ROOT/n) == h for n, h in names.items()),
        function_restored=current.depth_support is old, runtime_files_changed=False, online_deadline_verified=False)
    (OUT/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k not in ('samples', 'source_sha256')}))
    return 0 if report['input_unchanged'] and report['source_unchanged'] and report['function_restored'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
