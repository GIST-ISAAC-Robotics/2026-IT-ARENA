"""완전 합성 시야에서도 남은 엄격 관문의 거절을 관찰만 한다."""
from collections import Counter
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import numpy as np
import counterexamples as cx
import release_premises as rp
import review_visibility as rv


def classify(scene, points, pad):
    origins = np.array([r.ray_origin_xy_m for r in scene.rays])
    directions = np.array([r.direction_xy for r in scene.rays])
    indices = np.array([r.source_index for r in scene.rays])
    status = np.array([r.status for r in scene.rays])
    depth = np.array([math.dist(r.ray_origin_xy_m, r.hit_xy_m) if r.status == 'return' else -1.
                      for r in scene.rays])
    counts, samples = Counter(), []
    for offset in range(0, len(points), 128):
        p = points[offset:offset+128]
        delta = p[:, None, :]-origins[None, :, :]
        angles = np.arctan2(directions[None, :, 0]*delta[:, :, 1]-directions[None, :, 1]*delta[:, :, 0],
                            np.sum(directions[None, :, :]*delta, axis=2))
        low = np.where(angles >= -1e-10, np.maximum(angles, 0.), np.inf)
        high = np.where(angles <= 1e-10, np.maximum(-angles, 0.), np.inf)
        lo, hi = np.argmin(low, axis=1), np.argmin(high, axis=1)
        for i in range(len(p)):
            a, b = lo[i], hi[i]
            failures = []
            if not math.isfinite(low[i, a]+high[i, b]): failures.append('no_bracket')
            elif low[i, a]+high[i, b] > .025: failures.append('angular_gap')
            if abs(indices[a]-indices[b]) > 1: failures.append('index_gap')
            if status[a] != 'return' or status[b] != 'return': failures.append('nonreturn')
            if depth[a]-np.linalg.norm(delta[i, a])-pad[a] <= 0 or depth[b]-np.linalg.norm(delta[i, b])-pad[b] <= 0:
                failures.append('depth_or_padding')
            key = '+'.join(failures) or 'brackets_and_depth_free'
            counts[key] += 1
            if len(samples) < 6:
                samples.append({'xy': p[i].tolist(), 'source_indices': [int(indices[a]), int(indices[b])],
                                'angular_gap_rad': float(low[i, a]+high[i, b]), 'reason': key})
    return dict(counts), samples


def main(output, baseline):
    if output.exists(): raise FileExistsError(output)
    base = json.loads(baseline.read_text())
    root = HERE.parents[3]
    paths = [Path(__file__), HERE/'release_premises.py', HERE/'review_visibility.py',
             HERE/'counterexamples.py', HERE/'synthetic_world.py',
             root/'src/arena_autonomy/arena_autonomy/m3_execution_trial.py']
    hashes = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    results = {}
    for case in ['left_pass', 'right_pass']:
        events = []
        def observe(**data):
            scene = data['scene']
            if scene.t_ref_s < 5.5: return
            blocked = data['rest'][data['effective']]
            counts, samples = classify(scene, blocked, data['pad'])
            events.append({'t': scene.t_ref_s, 'side': data['record'].side,
                           'blocked': len(blocked), 'causes': counts, 'samples': samples})
        rp.DEBUG_HOOK = observe
        try:
            key, summary, rows = rv.job((case, 'union_strict', 0.))
        finally:
            rp.DEBUG_HOOK = None
        results[case] = {'summary': summary, 'output_equals_run11': rows == base[key], 'events': events}
    report = {'scope': 'observation-only full-view synthetic replay; no controller change',
              'results': results, 'source_sha256': hashes,
              'source_unchanged': all(hashlib.sha256((root/p).read_bytes()).hexdigest() == h for p,h in hashes.items())}
    output.mkdir(parents=True)
    (output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'source_unchanged': report['source_unchanged'],
                      'results': {k: {'released_at': v['summary']['released_at'],
                                        'output_equals_run11': v['output_equals_run11'],
                                        'cause_totals': dict(sum((Counter(x['causes']) for x in v['events']), Counter()))}
                                  for k,v in results.items()}},ensure_ascii=False))
    return 0 if report['source_unchanged'] and all(v['output_equals_run11'] for v in results.values()) else 1


if __name__ == '__main__':
    p=argparse.ArgumentParser(); p.add_argument('--output',type=Path,required=True); p.add_argument('--baseline',type=Path,required=True)
    a=p.parse_args(); sys.exit(main(a.output,a.baseline))
