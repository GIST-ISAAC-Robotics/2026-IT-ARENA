"""G1 선택기 ON 코너 연결 실행의 사후 창 분리 평가(평가 전용, 원래 판정 보존).

정지·재개는 10/25 `corner_window_eval.stop_restart_window`(run27 재현 확인 포함)를 그대로 써서 green+15 s 창으로 다시 읽고,
그 뒤 코너 구간은 평가기 40 s 결과·최장 정지·판단 이유·선택기 상태를 따로 집계한다. 정답 자세는 평가에만 쓴다.
"""
import importlib.util
import json
from collections import Counter
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
for part in ('scripts', 'src/arena_autonomy', 'src/arena_vehicle_interface'):
    sys.path.insert(0, str(ROOT/part))
SPEC = importlib.util.spec_from_file_location(
    'corner_window_eval', ROOT/'artifacts/validation/2026-10-25/m3_execution_review_v1/corner_window_eval.py')
cw = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(cw)
ev = cw.ev


def selector_window(run, start, end):
    trace = json.loads((run/'trace.json').read_text(encoding='utf-8'))
    rows = [r for r in trace if start <= r['t'] <= end]
    stall = ev.longest_true_interval(rows, lambda r: r['ground_speed_mps'] < .05, max_gap_s=ev.TRACE_MAX_GAP_S)
    reasons, modes, choices, selector, threats, windows, rejected = (Counter(), Counter(), Counter(), Counter(), [],
                                                                     Counter(), Counter())
    for line in (run/'follow_decisions.jsonl').open(encoding='utf-8'):
        e = json.loads(line)
        if e['kind'] != 'call' or not start <= e['data']['input']['reference_s'] <= end:
            continue
        d = e['data']['output'][2]
        reasons[d.get('reason')] += 1
        modes[str(d.get('selector_mode', d.get('selector_mode_before')))] += 1
        choices[str(d.get('selector_choice'))] += 1
        selector[str(d.get('selector_reason'))] += 1
        evidence = d.get('selector_evidence') or {}
        for side in ('left', 'right'):
            item = evidence.get(side) or {}
            if item.get('present'):
                threats.append(e['data']['input']['reference_s'])
            if 'side_window' in item:
                windows['free' if item['side_window']['free'] else 'not_free'] += 1
        if d.get('selector_choice') == 'parallel':
            rejected[str((d.get('parallel_hold_trial') or {}).get('reason'))] += 1
    last = rows[-1] if rows else {}
    return {'start_s': start, 'end_s': end, 'longest_stall_s': stall, 'decision_reasons': dict(reasons),
            'selector_modes': dict(modes), 'selector_choices': dict(choices), 'selector_reasons': dict(selector),
            'parallel_reasons': dict(rejected), 'side_window_checks': dict(windows),
            'threat_present_s': [min(threats), max(threats)] if threats else None, 'threat_present_calls': len(threats),
            'last_trace': {k: last.get(k) for k in ('t', 's', 'progress_m', 'yaw', 'ground_speed_mps')}}


def transitions(run):
    out, last = [], None
    for line in (run/'follow_decisions.jsonl').open(encoding='utf-8'):
        e = json.loads(line)
        if e['kind'] != 'call':
            continue
        d = e['data']['output'][2]
        mode = d.get('selector_mode', d.get('selector_mode_before'))
        sides = d.get('selector_sides')
        key = (mode, json.dumps({k: (v['present'], v['vanish'], v['passed']) for k, v in sides.items()}) if sides else None)
        if key != last:
            out.append({'t': e['data']['input']['reference_s'], 'mode': mode, 'sides': sides,
                        'selector_reason': d.get('selector_reason')})
            last = key
    return out


def main():
    run, out = Path(sys.argv[1]), Path(sys.argv[2])
    if out.exists():
        raise FileExistsError(out)
    base = ROOT/'artifacts/validation/2026-10-25/m3_execution_review_v1'
    r27, m27, g27 = cw.stop_restart_window(base/'run27_small_center_stop_restart/run', 'config/tests/g1_traffic_v1.json',
                                           'stop_restart', 15.)
    original = r27['evaluation']
    check = {'restart_delay_s': [m27['restart_delay_s'], original['case_metrics']['restart_delay_s']],
             'progress_after_restart_m': [m27['progress_after_restart_m'], original['case_metrics']['progress_after_restart_m']],
             'sampled_gap_at_window_end_m': [m27['sampled_gap_at_window_end_m'], original['progress']['sampled_gap_at_window_end_m']],
             'longest_hold_in_window_s': [m27['longest_hold_in_window_s'], original['case_metrics']['longest_hold_in_window_s']]}
    reproduced = all(abs(a-b) < 1e-9 for a, b in check.values()) and all(g27[k] == original['gates'][k] for k in g27)
    report, metrics, gates = cw.stop_restart_window(run, 'config/tests/g1_corner_after_threat_v1.json',
                                                    'stop_restart_first_curve', 15.)
    green = report['evaluation']['t_green']
    result = {'scope': 'post-hoc window split with evaluator functions; evaluation only; original 40 s verdict unchanged',
              'run27_reproduction': {'matches_original': reproduced, 'values': check},
              'stop_restart_15s': {'metrics': metrics, 'gates': gates},
              'first_15s_selector': selector_window(run, green, green+15.),
              'after_15s': selector_window(run, green+15., green+40.),
              'evaluator_40s': {'passed': report.get('passed'), 'gates': report['evaluation'].get('gates'),
                                'case_metrics': report['evaluation'].get('case_metrics')},
              'selector_transitions': transitions(run)}
    out.write_text(json.dumps(result, ensure_ascii=False, indent=1)+'\n', encoding='utf-8', newline='\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'selector_transitions'}, ensure_ascii=False)[:6000])
    if not reproduced:
        raise SystemExit('run27 reproduction mismatch')


if __name__ == '__main__':
    main()
