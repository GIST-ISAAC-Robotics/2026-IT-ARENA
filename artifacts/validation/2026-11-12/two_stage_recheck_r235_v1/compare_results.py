"""r235 두 실행의 전체 출력/명령과 단계별 비용을 독립 집계한다."""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
A = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import comparable_decision, equal_output

def load(mode, name):
    return json.loads((A/('jetson_'+mode)/name).read_bytes())

def main():
    baseline, candidate = (load(mode, 'answers.json') for mode in ('parent', 'two_stage'))
    assert len(baseline) == len(candidate) == 632
    full, commands = [], []
    for left, right in zip(baseline, candidate):
        assert left[0] == right[0]
        seq = left[0]
        if not equal_output(left, right):
            full.append(seq)
        a, b = comparable_decision(left[1]), comparable_decision(right[1])
        if not equal_output(a[:2], b[:2]):
            commands.append(dict(seq=seq, stage='plan'))
        assert (left[2] is None) == (right[2] is None)
        if left[2] is not None:
            keys = ('speed_mps', 'steering_rad', 'reason', 'reference_s')
            if not equal_output([left[2].get(k) for k in keys], [right[2].get(k) for k in keys]):
                commands.append(dict(seq=seq, stage='recheck'))
    modes = {}
    for mode in ('parent', 'two_stage'):
        report, durations = load(mode, 'report.json'), load(mode, 'durations.json')
        assert report['execution_complete'] and report['fault'] is None and report['source_unchanged']
        assert report['missing'] == report['duplicates'] == report['unexpected'] == 0
        assert report['cleanup'] == {'forced': False, 'exitcode': 0}
        modes[mode] = {key: report[key] for key in ('sent','received','callback_latency_ms','parent_tick_ms',
            'parent_callback_ms','child_ms','current_ms','parent_recheck_ms','recheck_end_to_end_ms',
            'recorded_schedule_lateness_ms')}
        modes[mode]['rechecks_200ms_or_more'] = sum(row['recheck_end_to_end_ms'] >= 200 for row in durations if row['has_current'])
        modes[mode]['rechecks'] = sum(row['has_current'] for row in durations)
        modes[mode]['answers_sha256'] = hashlib.sha256((A/('jetson_'+mode)/'answers.json').read_bytes()).hexdigest()
    result = dict(full_output_equal=not full, full_difference_sequences=full, command_differences=commands,
                  compared_plans=len(baseline), compared_rechecks=modes['parent']['rechecks'], modes=modes,
                  scope='same fixed snapshots and no actuation; current-age enforcement and full vehicle execution not tested')
    (A/'comparison.json').write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result))
    return 0 if not full and not commands else 1

if __name__ == '__main__':
    raise SystemExit(main())
