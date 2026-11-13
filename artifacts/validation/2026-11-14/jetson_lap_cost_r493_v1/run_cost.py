"""현재 코드로 r475 전체 plan 제출 순서를 한 번 계산한다. 센서/ROS/출력 없음."""
from pathlib import Path
import hashlib
import json
import resource
import statistics
import sys
import time

A = Path(__file__).resolve().parent
S = A / 'src'
sys.path[:0] = [str(S / 'arena_autonomy'), str(S / 'arena_vehicle_interface')]
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.follow_trace import equal_output


def main():
    raw = (A / 'inputs.json').read_bytes()
    data = json.loads(raw)
    expected = {tuple(r['key']): r['output'] for r in data['recorded_outputs']}
    assert len(data['plans']) == 508
    rows = []
    generation = None
    started = time.perf_counter_ns()
    evaluator = Evaluator('plan', data['header'])
    constructor_ms = (time.perf_counter_ns() - started) / 1e6
    try:
        with (A / 'results/outputs.jsonl').open('x') as out:
            for item in data['plans']:
                key = item['generation'], item['sequence']
                if key[0] != generation:
                    evaluator.reset(item['reset_reason'])
                    generation = key[0]
                before = json.dumps(item['input'], sort_keys=True, allow_nan=False)
                start = time.perf_counter_ns()
                value = evaluator(item['input'])
                elapsed = (time.perf_counter_ns() - start) / 1e6
                assert before == json.dumps(item['input'], sort_keys=True, allow_nan=False)
                old = expected[key]
                normal = value[2].get('normal') or {}
                rows.append(dict(key=key, reference_s=item['input']['reference_s'], ms=elapsed,
                    command=value[:2], recorded_command=old[:2],
                    command_equal=equal_output(value[:2], old[:2]),
                    output_equal=equal_output(value, old), selector_reason=normal.get('selector_reason'),
                    reason=normal.get('reason'), old_positive_new_zero=old[0] > .001 and value[0] <= .001))
                out.write(json.dumps(dict(key=key, output=value), allow_nan=False) + '\n')
                if len(rows) % 100 == 0:
                    print(json.dumps(dict(completed=len(rows), last_ms=elapsed)), flush=True)
    finally:
        evaluator.close()
    values = sorted(r['ms'] for r in rows)
    percentile = (len(values) - 1) * .95
    lo = int(percentile)
    imports = {n: str(Path(m.__file__).resolve()) for n, m in sys.modules.items()
               if n.startswith(('arena_autonomy', 'arena_vehicle_interface')) and getattr(m, '__file__', None)}
    assert all(Path(p).is_relative_to(S) for p in imports.values())
    assert hashlib.sha256((A / 'inputs.json').read_bytes()).digest() == hashlib.sha256(raw).digest()
    result = dict(plan_calls=len(rows), constructor_ms=constructor_ms, median_ms=statistics.median(values),
        p95_ms=values[lo] + (values[min(lo + 1, len(values)-1)]-values[lo])*(percentile-lo),
        max_ms=max(values), first_ms=rows[0]['ms'], over_100ms=sum(v > 100 for v in values),
        old_positive_new_zero=sum(r['old_positive_new_zero'] for r in rows),
        recorded_command_equal=sum(r['command_equal'] for r in rows),
        recorded_output_equal=sum(r['output_equal'] for r in rows), rows=rows, imports=imports,
        input_unchanged=True, warmup_calls=0, peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        scope='One complete saved simulation plan-submission sequence under current code. No ROS, live sensors, parent/current lane, independent guard or actuation. Recorded output is historical comparison, not current-code equality oracle.')
    (A / 'results/result.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('rows', 'imports')}), flush=True)


if __name__ == '__main__':
    main()
