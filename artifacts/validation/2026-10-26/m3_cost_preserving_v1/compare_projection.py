"""미연결 투영 산술 후보의 저장 입력 출력·순차 비용 대조. ROS/폐루프 마감 증명 아님."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import zipfile

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path[:0] = [str(ROOT/p) for p in ('src/arena_autonomy', 'src/arena_vehicle_interface')]
from arena_autonomy.lateral_intent import PathFrame
from arena_autonomy.follow_trace import freeze, replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.m3_execution_trial import ExecutionConfig, M3ExecutionFollow
from projection_candidate import project_xy


def distribution(values):
    return dict(count=len(values), p50_ms=float(np.percentile(values, 50)),
                p95_ms=float(np.percentile(values, 95)), max_ms=max(values),
                total_s=sum(values)/1000, over20_ms=sum(v > 20 for v in values))


def run(events, expected, variant, end_s):
    header = events[0]['data']
    ctl = M3ExecutionFollow(FollowConfig(**header['follow_config']),
                           ExecutionConfig(**header['execution_config']), allow_simulation_trial=True)
    motion = MotionConfig(**header['motion_config'])
    original = PathFrame.project
    if variant == 'candidate': PathFrame.project = project_xy
    times, differences = [], []
    try:
        for i, event in enumerate(events[1:-1]):
            if event['kind'] == 'reset':
                ctl.reset()
                if expected[i] != {'reset': True}: differences.append(i)
                continue
            if event['kind'] != 'call': raise ValueError('unexpected event')
            if event['data']['input']['reference_s'] > end_s: break
            before = time.perf_counter_ns()
            result = replay_call(ctl, event['data']['input'], motion)
            times.append((time.perf_counter_ns()-before)/1e6)
            digest = hashlib.sha256(json.dumps(freeze(result), sort_keys=True).encode()).hexdigest()
            if digest != expected[i]['sha']: differences.append(i)
    finally:
        PathFrame.project = original
    return {'variant': variant, 'output_differences': differences, 'timing': distribution(times)}


def main(out, full):
    out.mkdir(parents=True, exist_ok=False)
    sources = list((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')) + [Path(__file__), HERE/'projection_candidate.py']
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    with zipfile.ZipFile(out/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as z:
        for p in sources: z.write(p, p.relative_to(ROOT))
    names = ('run17_observed_release_closed_loop', 'run21_right_encounter')
    if full: names += ('run19_stop_restart', 'run32_corner_after_threat', 'run35_committed_surface_corner')
    results = {}
    for name in names:
        path = ROOT/'artifacts/validation/2026-10-25/m3_execution_review_v1'/name/'run/follow_decisions.jsonl'
        raw = path.read_bytes(); events = [json.loads(line) for line in raw.decode('utf-8').splitlines()]
        expected = json.loads((HERE.parent/'m3_rear_residual_v1/cachefix_runs/run7_premises_cachefix'/
                              (name+'__plain.json')).read_text())
        end_s = float('inf') if full else 6.
        if not full:
            # 워밍업도 출력 일치를 요구하며 측정 요약에서는 분리한다.
            warm = [run(events, expected, v, end_s) for v in ('baseline', 'candidate')]
        else: warm = []
        rows = [run(events, expected, v, end_s) for v in
                (('candidate',) if full else ('baseline', 'candidate', 'candidate', 'baseline', 'baseline', 'candidate'))]
        results[name] = dict(warmups=warm, rounds=rows, input_sha256=hashlib.sha256(raw).hexdigest(),
                             input_unchanged=path.read_bytes()==raw)
        print(json.dumps({name: results[name]}), flush=True)
    unchanged = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
    passed = unchanged and all(r['input_unchanged'] and all(not x['output_differences'] for x in r['rounds']+r['warmups']) for r in results.values())
    (out/'summary.json').write_text(json.dumps(dict(passed=passed, source_unchanged=unchanged,
        source_sha256=hashes, results=results, scope='offline exact-output comparison; no online deadline guarantee'), indent=2)+'\n')
    return 0 if passed else 1


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--output', type=Path, required=True); p.add_argument('--full', action='store_true')
    args = p.parse_args(); sys.exit(main(args.output, args.full))
