"""캐시 수정 전후의 저장 출력 비교. 원본 결과는 바꾸지 않는다."""
import argparse
import hashlib
import json
from pathlib import Path


def read(path):
    raw = path.read_bytes()
    return json.loads(raw), hashlib.sha256(raw).hexdigest()


def main(old_dir, new_dir, old_counter, new_counter, output):
    if output.exists():
        raise FileExistsError(output)
    old, old_hash = read(old_dir/'summary.json')
    new, new_hash = read(new_dir/'summary.json')
    results = {}
    for name, entry in new['inputs'].items():
        previous = old['inputs'][name]
        row = {'input_sha_matches_previous': entry['sha256'] == previous['sha256'],
               'input_unchanged': entry['input_unchanged'], 'variants': {}}
        for variant, value in entry['variants'].items():
            a, a_hash = read(old_dir/f'{name}__{variant}.json')
            b, b_hash = read(new_dir/f'{name}__{variant}.json')
            ca, cb = [r for r in a if not r.get('reset')], [r for r in b if not r.get('reset')]
            command_changed = sum(abs(x['speed']-y['speed']) > 1e-12 or abs(x['steer']-y['steer']) > 1e-12
                                  for x, y in zip(ca, cb)) + abs(len(ca)-len(cb))
            row['variants'][variant] = {
                'calls': value['calls'],
                'full_output_differences_before_after': sum(x.get('sha') != y.get('sha') for x, y in zip(a, b))
                                                          + abs(len(a)-len(b)),
                'command_differences_before_after': command_changed,
                'released_before': previous['variants'][variant]['released_at'],
                'released_after': value['released_at'], 'positive_after': value['positive'],
                'full_output_differences_vs_plain_after': value['output_differences_vs_plain'],
                'memory_blocked_first_t_after': value['memory_blocked_first_t'],
                'rows_sha_before': a_hash, 'rows_sha_after': b_hash}
        results[name] = row
    a, a_hash = read(old_counter/'summary.json')
    b, b_hash = read(new_counter/'summary.json')
    comparisons = {}
    for case, trials in b['results'].items():
        for name, value in trials.items():
            prev = a['results'][case][name]
            if prev != value:
                comparisons[f'{case}__{name}'] = {'before': prev, 'after': value}
    checks = {'inputs_equal_and_unchanged': all(r['input_sha_matches_previous'] and r['input_unchanged']
                                               for r in results.values()),
              'strict_equals_plain_all_inputs': all(r['variants']['strict']['full_output_differences_vs_plain_after'] == 0
                                                     for r in results.values()),
              'replay_source_unchanged': new['sources_after_unchanged'],
              'counterexample_checks': all(b['checks'].values()),
              'counterexample_source_unchanged': b['sources_unchanged']}
    report = {'scope': 'corrected prototype vs archived defect output; open-loop only',
              'checks': checks, 'input_results': results,
              'counterexample_changed_results': comparisons,
              'counterexample_checks_before': a['checks'], 'counterexample_checks_after': b['checks'],
              'summary_sha256': {'old_replay': old_hash, 'new_replay': new_hash,
                                'old_counterexamples': a_hash, 'new_counterexamples': b_hash},
              'cost_retest': 'deferred because premise-dependent candidate is not accepted; no corrected timing claim'}
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'checks': checks, 'counterexample_changed_results': list(comparisons),
                     'replay_changed_variants': {n: [v for v, item in r['variants'].items()
                          if item['full_output_differences_before_after']] for n, r in results.items()}}, ensure_ascii=False))
    return 0 if all(checks.values()) else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--old', type=Path, required=True)
    parser.add_argument('--new', type=Path, required=True)
    parser.add_argument('--old-counter', type=Path, required=True)
    parser.add_argument('--new-counter', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    raise SystemExit(main(args.old, args.new, args.old_counter, args.new_counter, args.output))
