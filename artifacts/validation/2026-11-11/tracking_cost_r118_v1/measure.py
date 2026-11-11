"""동결 r117 후보의 단계 비용. 장치 입력/명령 발행 없이 저장 센서만 사용한다."""
import collections
import hashlib
import json
from pathlib import Path
import sys
import time

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
PREVIOUS = A.parent / 'current_tracking_r117_v1'
sys.path.insert(0, str(PREVIOUS))
import probe
import candidate
from arena_autonomy.follow_trace import equal_output


def main():
    manifest = json.loads((A/'bundle_manifest.json').read_bytes())
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    changed = [n for n, v in manifest['files'].items() if digest(ROOT/n) != v['sha256']]
    if changed:
        raise RuntimeError('frozen source mismatch: '+str(changed))
    costs = collections.defaultdict(list)

    def measure(module, name):
        original = getattr(module, name)
        def timed(*args, **kwargs):
            start = time.perf_counter()
            try:
                return original(*args, **kwargs)
            finally:
                costs[name].append(1000*(time.perf_counter()-start))
        setattr(module, name, timed)
    for name in ('transport_points', 'pursue_path', 'swept_limit', 'motion_region'):
        measure(candidate, name)
    for name in ('restore_call', 'motion_scene'):
        measure(probe, name)
    probe.A = A
    probe.main()
    read = lambda p: [json.loads(s) for s in p.open(encoding='utf8')]
    expected, actual = read(PREVIOUS/'details.jsonl'), read(A/'details.jsonl')
    if len(expected) != len(actual):
        raise RuntimeError('candidate result count mismatch')
    keys = ('reference_s', 'eligible', 'reason', 'speed_mps', 'steering_rad')
    differences = [i for i, (a, b) in enumerate(zip(expected, actual))
        if not equal_output({k:a.get(k) for k in keys}, {k:b.get(k) for k in keys})]
    full = [i for i, (a, b) in enumerate(zip(expected, actual)) if not equal_output(a, b)]
    summary = {name:dict(calls=len(v), sum_ms=sum(v),
        p50_ms=sorted(v)[len(v)//2], p95_ms=sorted(v)[int(.95*len(v))], maximum_ms=max(v))
        for name, v in costs.items()}
    result = dict(stages=summary, candidate_output_difference_indices=differences,
        full_diagnostic_difference_indices=full,
        source_unchanged=all(digest(ROOT/n) == v['sha256'] for n,v in manifest['files'].items()),
        scope='Saved r117 candidate, stage wrappers add cost; not ROS real-time or actual driving.')
    with (A/'stages.json').open('x') as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result))


if __name__ == '__main__':
    main()
