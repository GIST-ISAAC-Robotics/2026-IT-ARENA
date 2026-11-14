"""같은 ARM의 baseline/candidate 원값·판단값·시간 진단을 각각 비교한다."""
import argparse
from collections import Counter
import gzip
import json
from pathlib import Path
import sys

A = Path(__file__).resolve().parent


def compare(left, right, out, *, timing):
    from arena_autonomy.follow_trace import comparable_decision
    from state_codec import comparable_state, differences
    open_read = lambda p: gzip.open(p, 'rt', encoding='utf8') if p.suffix == '.gz' else p.open(encoding='utf8')
    rows = []
    output_paths, state_paths = Counter(), Counter()
    with open_read(left) as astream, open_read(right) as bstream:
        for index, (a, b) in enumerate(zip(astream, bstream, strict=True)):
            a, b = json.loads(a), json.loads(b)
            assert a['key'] == b['key']
            od = differences(a['output'], b['output'])
            sd = differences(a['state'], b['state'])
            output_paths.update(d['path'] for d in od)
            state_paths.update(d['path'] for d in sd)
            rows.append(dict(index=index, key=a['key'], command_exact=a['output'][:2] == b['output'][:2],
                strict_output_exact=not od, decision_exact=comparable_decision(a['output']) == comparable_decision(b['output']),
                strict_state_exact=not sd, diagnostic_normalized_state_exact=comparable_state(a['state']) == comparable_state(b['state']),
                output_differences=od, state_differences=sd))
    keys = ('command_exact', 'strict_output_exact', 'decision_exact', 'strict_state_exact', 'diagnostic_normalized_state_exact')
    result = dict(count=len(rows), **{key:sum(r[key] for r in rows) for key in keys},
        output_difference_paths=dict(output_paths), state_difference_paths=dict(state_paths), rows=rows,
        exclusions='Output: existing comparable_decision paths only. State: [normal.]partial_policy.surface_cache[2].processing_ms only. Strict originals retained.',
        comparison_basis='same-source-policy baseline versus same-source-policy candidate; historical r493 outputs are not the equality oracle')
    assert len(rows) == 508
    if timing:
        before = json.loads((left.parent/'summary.json').read_bytes())
        after = json.loads((right.parent/'summary.json').read_bytes())
        assert before['input_sha256'] == after['input_sha256'] and before['resets'] == after['resets']
        assert before['thread_environment'] == after['thread_environment']
        cost_keys = ('constructor_ms','first_ms','median_ms','p95_ms','max_ms','over_100ms','elapsed_wall_s','peak_rss_kib')
        paired = []
        for x, y in zip(before['rows'], after['rows'], strict=True):
            assert x['key'] == y['key']
            paired.append(dict(key=x['key'], before_ms=x['ms'], after_ms=y['ms'], delta_ms=y['ms']-x['ms']))
        result.update(input_sha256=before['input_sha256'], resets=before['resets'],
            baseline_cost={k:before[k] for k in cost_keys}, candidate_cost={k:after[k] for k in cost_keys},
            paired_cost=paired, slow_targets=[r for r in paired if r['key'][1] in (717,1304,1311)],
            native_same_arm=True, input_unchanged=before['input_unchanged'] and after['input_unchanged'],
            performance_claim='one ordered saved sequence only; no causal speedup or full runtime claim')
    result['decision_and_state_pass'] = all(result[k] == len(rows) for k in ('command_exact','decision_exact','diagnostic_normalized_state_exact'))
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('rows','paired_cost')}, ensure_ascii=False, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--saved-windows', type=Path)
    parser.add_argument('--python-deps', type=Path)
    args = parser.parse_args()
    sys.path[:0] = ([str(args.python_deps)] if args.python_deps else []) + [
        str(A/'candidate/src/arena_autonomy'), str(A/'candidate/src/arena_vehicle_interface')]
    if args.saved_windows:
        source = args.saved_windows
        result = compare(source/'original/outputs.jsonl', source/'stage/outputs.jsonl', A/'windows_existing_reanalysis.json', timing=False)
    else:
        result = compare(A/'results/baseline/outputs.jsonl.gz', A/'results/candidate/outputs.jsonl.gz', A/'results/comparison.json', timing=True)
    return 0 if result['decision_and_state_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
