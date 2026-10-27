"""동일 코너 등록 조건의 이전/후보 실행을 요약. 제어 입력과 무관한 사후 평가다."""
import json
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def summarize(base):
    run = base/'run1_corner'
    report, window = read(run/'report.json'), read(base/'run1_window_eval.json')
    counts, reasons = Counter(), Counter()
    guarded = []
    for line in (run/'follow_decisions.jsonl').open(encoding='utf-8'):
        entry = json.loads(line)
        if entry['kind'] != 'call':
            continue
        speed, steering, detail = entry['data']['output']
        counts[detail.get('selector_choice', 'missing')] += 1
        reasons[detail.get('selector_reason', 'missing')] += 1
        if detail.get('selector_choice') == 'guard_baseline':
            g = detail['selector_guard_candidate']
            guarded.append({'speed': speed, 'unknown': detail['selector_unknown_filled'],
                            'mode': detail['selector_mode'], 'region_unknown': g['region']['unobserved'],
                            'tail_risk': detail.get('selector_masked_tail_risk_accepted', False)})
    ev = report['evaluation']
    return {'original_passed': report['passed'], 'run_valid': report['run_valid'],
            'harness_valid': report['harness_valid'], 'gates': ev['gates'],
            'progress': ev['progress'], 'case_metrics': ev['case_metrics'],
            'gap': ev['gap'], 'road': ev['road'],
            'contacts': report['contact_evidence'], 'additional_actor_gates': report['additional_actors'],
            'stop': report['stop'], 'actuation_stop_passed': report['actuation_stop_passed'],
            'shutdown_clean': report['shutdown_clean'], 'process_exit_codes': report['process_exit_codes'],
            'remaining_pids': report['remaining_test_pids'],
            'stop_restart_15s': window['stop_restart_15s'],
            'after15_stall_s': window['after_15s']['longest_stall_s'],
            'selector_choices': counts, 'selector_reasons': reasons,
            'guard_candidate': {'calls': len(guarded),
                                'max_command_mps': max((g['speed'] for g in guarded), default=None),
                                'unknown_filled_calls': sum(g['unknown'] for g in guarded),
                                'not_guard_mode_calls': sum(g['mode'] != 'GUARD' for g in guarded),
                                'unobserved_region_calls': sum(g['region_unknown'] > 0 for g in guarded),
                                'small_masked_tail_risk_calls': sum(g['tail_risk'] for g in guarded)}}


if __name__ == '__main__':
    out = HERE/'run1_comparison.json'
    if out.exists():
        raise FileExistsError(out)
    previous = summarize(HERE.parent/'g1_pass_release_operation_v1')
    candidate = summarize(HERE)
    result = {'scope': 'one closed-loop run each, unchanged registered scenario; not repeatability or hardware proof',
              'previous': previous, 'candidate': candidate}
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k: {n: v[n] for n in ('original_passed', 'progress', 'after15_stall_s', 'guard_candidate')}
                      for k, v in result.items() if isinstance(v, dict)}, ensure_ascii=False))
