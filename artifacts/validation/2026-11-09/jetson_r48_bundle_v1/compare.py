"""저장소/추출본 CLI 실행 4회의 결과를 비교해 parity.json을 쓴다. python3 -I -B compare.py"""
import gzip
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[3]
EXTRACT = REPO/'build/jetson_follow_20261109_r48'
REL = HERE.relative_to(REPO).as_posix()
TRACES = dict(r48_corner=1904, r44_straight=548)
ROW_KEYS = ('seq', 'reason', 'handoff_state', 'selection', 'new_source_stamp', 'failed_stage')


def load(side, name):
    base = (REPO if side == 'repo' else EXTRACT)/REL/f'cli_{side}'/name
    report = json.loads((base/'report.json').read_text(encoding='utf-8'))
    run = json.loads((HERE/f'run_{side}_{name}.json').read_text(encoding='utf-8'))
    with gzip.open(HERE/f'commands_{side}_{name}.json.gz', 'rt', encoding='utf-8') as stream:
        calls = json.load(stream)['calls']
    return report, run, calls


def cost(report):
    total = report.get('groups', {}).get('all', {}).get('total_ms', {})
    return {k: total.get(k) for k in ('count', 'p50', 'p95', 'maximum', 'over_20ms')}


def main():
    hashes = {k: json.loads((HERE/f'input_hashes_{k}.json').read_text(encoding='utf-8')) for k in ('before', 'after')}
    result = dict(input_hashes_unchanged=hashes['before'] == hashes['after'], traces={})
    for name, expected in TRACES.items():
        (r_rep, r_run, r_calls), (b_rep, b_run, b_calls) = load('repo', name), load('bundle', name)
        rows = lambda rep: [[row.get(k) for k in ROW_KEYS] for row in rep.get('rows', [])]
        first_diff = next((i for i, (a, b) in enumerate(zip(r_calls, b_calls)) if a[3] != b[3]), None)
        entry = dict(
            expected_calls=expected,
            repo=dict(exit=r_run['cli_exit'], passed=r_rep.get('passed'), baseline_matched=r_rep.get('baseline_matched'),
                      timed_calls=r_rep.get('timed_calls'), output_mismatches=len(r_rep.get('output_mismatches', [])),
                      error=r_rep.get('error'), source_snapshot_omitted=r_rep.get('source_snapshot_omitted'),
                      source_snapshot_complete=r_rep.get('source_snapshot_complete'), snapshot_files=len(r_rep.get('source_sha256', {})),
                      input_unchanged=r_rep.get('input_unchanged'), source_unchanged=r_rep.get('source_unchanged'),
                      recorded_calls=r_run['recorded_calls'], project_modules=r_run['project_modules'],
                      from_root_only=r_run['from_root_only'], outside=r_run['outside'], isolated=r_run['isolated'],
                      cwd_on_start_path=r_run['cwd_on_start_path'], total_ms=cost(r_rep)),
            bundle=dict(exit=b_run['cli_exit'], passed=b_rep.get('passed'), baseline_matched=b_rep.get('baseline_matched'),
                        timed_calls=b_rep.get('timed_calls'), output_mismatches=len(b_rep.get('output_mismatches', [])),
                        error=b_rep.get('error'), source_snapshot_omitted=b_rep.get('source_snapshot_omitted'),
                        source_snapshot_complete=b_rep.get('source_snapshot_complete'), snapshot_files=len(b_rep.get('source_sha256', {})),
                        input_unchanged=b_rep.get('input_unchanged'), source_unchanged=b_rep.get('source_unchanged'),
                        recorded_calls=b_run['recorded_calls'], project_modules=b_run['project_modules'],
                        from_root_only=b_run['from_root_only'], outside=b_run['outside'], isolated=b_run['isolated'],
                        cwd_on_start_path=b_run['cwd_on_start_path'], total_ms=cost(b_rep)),
            command_sequence_equal=r_run['command_sequence_sha256'] == b_run['command_sequence_sha256'] and len(r_calls) == len(b_calls),
            command_sequence_sha256=r_run['command_sequence_sha256'], first_command_diff_index=first_diff,
            cli_rows_equal=rows(r_rep) == rows(b_rep),
            positive_speed_calls=sum(1 for c in r_calls[expected:] if c[0] > 0),
            reasons={})
        for c in r_calls[expected:]:
            entry['reasons'][c[2]] = entry['reasons'].get(c[2], 0)+1
        side_ok = lambda s, omitted: (s['exit'] == 0 and s['passed'] and s['baseline_matched'] == expected
                                      and s['timed_calls'] == expected and s['output_mismatches'] == 0
                                      and s['recorded_calls'] == 2*expected and s['from_root_only'] and s['isolated'] == 1
                                      and not s['cwd_on_start_path'] and s['input_unchanged'] and s['source_unchanged']
                                      and s['source_snapshot_omitted'] == omitted)
        entry['passed'] = (side_ok(entry['repo'], []) and side_ok(entry['bundle'], ['tests/test_follow_profile.py'])
                           and entry['command_sequence_equal'] and entry['cli_rows_equal'])
        result['traces'][name] = entry
    result['passed'] = result['input_hashes_unchanged'] and all(e['passed'] for e in result['traces'].values())
    result['note'] = ('recorded_calls = replay_events baseline pass + 1 timed pass; costs include recording wrapper '
                      'overhead and are single-run WSL2 values, not Jetson/ROS deadlines')
    with (HERE/'parity.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
