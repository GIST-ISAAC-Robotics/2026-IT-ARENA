"""동결된 한 후보의 원 508 plan을 native ARM에서 순서대로 한 번 계산한다."""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import platform
import resource
import statistics
import sys
import time

A = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('variant', choices=('baseline', 'candidate'))
    args = parser.parse_args()
    assert sys.platform == 'linux' and platform.machine() == 'aarch64'
    root = A/args.variant
    sys.path[:0] = [str(root/'src/arena_autonomy'), str(root/'src/arena_vehicle_interface')]
    import numpy as np
    from arena_autonomy.race_runtime_policy.evaluator import Evaluator
    from arena_autonomy import _numeric_kernels
    from state_codec import state
    result = A/'results'/args.variant
    result.mkdir(parents=True, exist_ok=False)
    raw = (A/'inputs.json').read_bytes()
    data = json.loads(raw)
    assert len(data['plans']) == 508 and data['header']['numeric_execution'] == 'native_v1'
    rows, resets = [], []
    generation = None
    started = time.perf_counter_ns()
    evaluator = Evaluator('plan', data['header'])
    constructor_ms = (time.perf_counter_ns()-started)/1e6
    wall_start = time.perf_counter()
    try:
        with gzip.open(result/'outputs.jsonl.gz', 'xt', encoding='utf8', compresslevel=1) as stream:
            for index, item in enumerate(data['plans']):
                key = [item['generation'], item['sequence']]
                if key[0] != generation:
                    evaluator.reset(item['reset_reason'])
                    resets.append(dict(index=index, generation=key[0], reason=item['reset_reason']))
                    generation = key[0]
                before = json.dumps(item['input'], sort_keys=True, allow_nan=False)
                started = time.perf_counter_ns()
                output = evaluator(item['input'])
                elapsed = (time.perf_counter_ns()-started)/1e6
                assert before == json.dumps(item['input'], sort_keys=True, allow_nan=False)
                # 호출 타이머 밖에서 전체 상태를 기록한다. 입력/상태 주입과 함수 계측은 없다.
                record = dict(key=key, output=output, state=state(evaluator.follow))
                stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n')
                rows.append(dict(index=index, key=key, reference_s=item['input']['reference_s'], ms=elapsed))
                if (index+1) % 100 == 0 or key[1] in (717, 1304, 1311):
                    print(json.dumps(dict(completed=index+1, key=key, ms=elapsed)), flush=True)
    finally:
        evaluator.close()
    paths = {name: str(Path(module.__file__).resolve()) for name, module in sys.modules.items()
             if name.startswith(('arena_autonomy', 'arena_vehicle_interface')) and getattr(module, '__file__', None)}
    assert all(Path(p).is_relative_to(root/'src') for p in paths.values())
    assert (A/'inputs.json').read_bytes() == raw
    values = sorted(row['ms'] for row in rows)
    at = (len(values)-1)*.95
    lo = int(at)
    summary = dict(variant=args.variant, plan_calls=len(rows), rows=rows, resets=resets,
        constructor_ms=constructor_ms, first_ms=rows[0]['ms'], median_ms=statistics.median(values),
        p95_ms=values[lo]+(values[min(lo+1, len(values)-1)]-values[lo])*(at-lo),
        max_ms=max(values), over_100ms=sum(ms > 100 for ms in values),
        elapsed_wall_s=time.perf_counter()-wall_start, python=sys.version, numpy_version=np.__version__,
        native_path=str(Path(_numeric_kernels.__file__).resolve()), imports=paths,
        input_sha256=hashlib.sha256(raw).hexdigest(), original_header_unchanged=True,
        input_unchanged=True, state_injected=False, repeated_calls=0, warmup_calls=0,
        peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        thread_environment={k: os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS')},
        timing_scope='Evaluator call only; no instrumentation. Input validation and full state gzip serialization are outside the timer, between calls.',
        limitation='One baseline then one candidate sequence: scheduling, cache and thermal order effects remain. No ROS, live sensors, actuation or real-time throughput proof.')
    (result/'summary.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf8')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('rows','imports')}), flush=True)


if __name__ == '__main__':
    main()
