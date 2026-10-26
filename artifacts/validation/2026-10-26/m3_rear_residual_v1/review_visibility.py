"""새 합성 광선으로만 후방 가림의 영향을 분리한다. 실물 장착 변경/운영 연결 아님.

0도는 완전 시야라는 비교용 상한이며 실제 차체로 구현 가능함을 뜻하지 않는다.
저장 scan의 NaN을 복원하거나 무응답을 자유 공간으로 간주하지 않는다.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path[:0] = [str(HERE)]
import counterexamples as cx
from review_geometry import scene_with_mask


def job(args):
    case, trial_name, half_angle = args
    original = cx.world_for
    def with_mask(name):
        world = original(name)
        def scene(t_ref, bodies=None):
            ranges, stamps = world.ranges(t_ref, bodies)
            return scene_with_mask(world, t_ref, ranges, stamps, half_angle)
        world.scene = scene
        return world
    cx.world_for = with_mask
    try:
        _, _, rows = cx.run((case, trial_name))
    finally:
        cx.world_for = original
    return f'{case}__{trial_name}__{half_angle:g}', cx.summarize(rows), rows


def main(output, workers):
    if output.exists():
        raise FileExistsError(output)
    paths = [Path(__file__), HERE/'review_geometry.py', HERE/'counterexamples.py',
             HERE/'synthetic_world.py', HERE/'release_premises.py',
             ROOT/'src/arena_autonomy/arena_autonomy/m3_execution_trial.py',
             ROOT/'src/arena_autonomy/arena_autonomy/local_scene.py']
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    cases = ['left_pass', 'right_pass', 'same_side_run37', 'reapproach']
    jobs = [(c, trial, a) for c in cases for trial in ('ledger_current', 'union_strict') for a in (30., 25., 0.)]
    results, all_rows = {}, {}
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for key, result, rows in pool.map(job, jobs):
            results[key], all_rows[key] = result, rows
            print(json.dumps({key: result}, ensure_ascii=False), flush=True)
    source_unchanged = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p, h in hashes.items())
    # Characterization, not a pass criterion demanding that any configuration drive.
    report = {'scope': 'fresh synthetic input / open-loop release comparison, no physics or hardware claim',
              'premises': 'strict only; no P-follow, P-size, or masked-free exemption',
              'source_sha256': hashes, 'source_unchanged': source_unchanged, 'results': results,
              'limits': ['Different masking is a diagnostic hypothesis, not measured hardware.',
                         'The synthetic ego keeps moving independently of controller output.',
                         'Release is not a contact/clearance/complete-lap success verdict.']}
    output.mkdir(parents=True)
    (output/'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    (output/'rows.json').write_text(json.dumps(all_rows, ensure_ascii=False)+'\n', encoding='utf-8')
    return 0 if source_unchanged else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=6)
    args = parser.parse_args()
    sys.exit(main(args.output, args.workers))
