"""사전 등록한 출발 목표만 평가. 과거 재접근/원본 slow_lead 실패를 바꾸지 않는다."""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def main():
    run = HERE/'run1'
    report = json.loads((run/'report.json').read_bytes())
    surface = json.loads((HERE/'run1_surface.json').read_bytes())
    registered = json.loads((HERE/'registered.json').read_bytes())
    ev = report['evaluation']
    green, end = ev['t_green'], report['case_window']['end_sim_s']
    trace = [r for r in json.loads((run/'trace.json').read_bytes()) if green <= r['t'] <= end]
    first = next((r['t']-green for r in trace if r['ground_speed_mps'] >= .1), None)
    applied = []
    for line in (run/'follow_decisions.jsonl').open(encoding='utf-8'):
        event = json.loads(line)
        if event['kind'] != 'call':
            continue
        output = event['data']['output']
        normal = output[2].get('normal') or output[2]
        dep = (normal.get('parallel_hold_trial') or {}).get('side_departure')
        if dep and dep.get('applied'):
            applied.append(dict(t=event['data']['input']['reference_s'], command=output[:2], diagnostic=dep))
    gap, wheels = ev['gap'], surface['interpretations']
    checks = dict(
        run_valid=report.get('run_valid') is True,
        source=report.get('source_unchanged') is True and all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == v for p,v in registered['source_sha256'].items()),
        option_recorded=report.get('handoff_execution',{}).get('checks',{}).get('side_departure') is True,
        candidate_used=any(r['command'][0] > 0 for r in applied),
        stop=bool(report.get('stop_checks')) and all(report['stop_checks'].values()),
        shutdown=report.get('shutdown_clean') is True,
        signal=ev['gates'].get('green_observed') is True,
        early_progress=first is not None and first <= 1.3,
        progress=ev['progress']['at_window_end_m'] >= .3,
        contact=gap['actor_contact_messages_with_ego'] == 0,
        overlap=gap['sampled_overlap_samples'] == 0,
        gap=gap['min_sampled_gap_m'] is not None and gap['min_sampled_gap_m'] >= .08,
        swept_gap=gap.get('min_swept_lower_bound_m') is not None and gap['min_swept_lower_bound_m'] >= .08 and gap.get('swept_missing_actor_intervals') == 0,
        wheel_evidence=surface['samples'] > 0 and surface['sample_gaps_over_limit'] == 0 and set(wheels) == {'any','center','full'},
        no_wall=surface['wall_wheel_samples'] == 0,
        two_wheels=bool(wheels) and all(v['max_off_wheels'] <= 2 for v in wheels.values()))
    result = dict(passed=all(checks.values()), checks=checks, metrics=dict(
        first_0p1_mps_after_green_s=first, progress_m=ev['progress']['at_window_end_m'],
        peak_speed_mps=max(r['ground_speed_mps'] for r in trace),
        minimum_gap_m=gap['min_sampled_gap_m'], minimum_swept_gap_m=gap.get('min_swept_lower_bound_m'),
        candidate_applied_calls=len(applied)), original_passed=report['passed'],
        original_gates=ev['gates'], applied=applied,
        scope='정지 옆차 한 배치에서 제한 출발 1회. 오른쪽 사각 재접근/독립 상대/실차 안전의 증명 아님.')
    (HERE/'run1_goal.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k != 'applied'}))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
