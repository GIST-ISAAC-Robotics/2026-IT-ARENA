"""run32 사후 창 분리 평가. 평가기 함수(progress_series·sampled_gaps·longest_true_interval·window)를 그대로 쓰고
정지·재개 판정만 green+15 s 창으로 다시 계산한다. 코너 판정은 원래 평가기의 40 s 창 결과를 따로 읽는다.
정답 자세는 사후 평가 전용이다. run27(15 s case)에 같은 계산을 적용해 원래 보고서 값과 같은지 먼저 확인한다.
"""
import json
from pathlib import Path
import sys

import validate_g1_traffic as runner
import g1_traffic_eval as ev


def load_rows(path):
    return [tuple(json.loads(line)) for line in path.open(encoding='utf-8')]


def stop_restart_window(run, config_path, case_name, window_s):
    report = json.loads((run/'report.json').read_text(encoding='utf-8'))
    cfg = runner.load_config(Path(config_path))
    rules = cfg['acceptance'][case_name]
    green = report['evaluation']['t_green']
    t_end = green+window_s
    ego, actor = load_rows(run/'ego_poses.jsonl'), load_rows(run/'actor_poses.jsonl')
    trace = json.loads((run/'trace.json').read_text(encoding='utf-8'))
    line, _ = runner.load_centerline()
    # 원래 평가기와 같은 표본을 사용한다. t_end 뒤 표본도 보존해야
    # 종료 시각이 두 표본 사이일 때 _value_at의 보간을 동일하게 재현한다.
    span = ev.window_tuples(ego, green, max(r[0] for r in ego))
    progress, _ = ev.progress_series(line, span)

    def progress_at(t):
        value = ev._value_at(progress, t, 'progress_m')
        return value if value is not None else 0.
    sampled = ev.sampled_gaps(span, actor, cfg['actor']['length_m'], cfg['actor']['width_m'], ev.POSE_MAX_GAP_S)
    gap_end = ev._value_at(sampled, t_end, 'gap_m')
    rows = [r for r in trace if r['t'] >= green]

    def stationary(row):
        return row['ground_speed_mps'] < .01 and row['safe_speed_mps'] == 0 and row['autonomy'].get('behavior') == 'HOLD'
    first, last = (green+v for v in rules['hold_window_s'])
    hold = ev.longest_true_interval(ev.window(rows, first, last), stationary, max_gap_s=ev.TRACE_MAX_GAP_S)
    restart = [r['t'] for r in ev.window(rows, last, t_end) if r['ground_speed_mps'] > rules['restart_speed_mps']]
    delay = restart[0]-last if restart else None
    after = progress_at(t_end)-progress_at(last)
    metrics = {'window_end_s': t_end, 'longest_hold_in_window_s': hold, 'restart_delay_s': delay,
               'progress_after_restart_m': after, 'progress_at_window_end_m': progress_at(t_end),
               'sampled_gap_at_window_end_m': gap_end}
    gates = {'hold': hold >= rules['minimum_hold_s'],
             'restart': delay is not None and delay <= rules['maximum_restart_delay_s'],
             'progress_after_restart': after >= rules['minimum_progress_after_restart_m'],
             'final_gap': gap_end is not None and gap_end <= rules['maximum_final_gap_m']}
    return report, metrics, gates


def after_window(run, start, end, report):
    """green+15 s 이후: 최장 정지(정답 속도 < 0.05 m/s), 판단 이유·모드, 측방 위협 재촉발."""
    trace = json.loads((run/'trace.json').read_text(encoding='utf-8'))
    # 명시 STOP/밀집 정지 절차를 주행 중 교착으로 세지 않는다.
    rows = [r for r in trace if start <= r['t'] <= end]
    stall = ev.longest_true_interval(rows, lambda r: r['ground_speed_mps'] < .05, max_gap_s=ev.TRACE_MAX_GAP_S)
    reasons, modes, threats, rejected = {}, {}, [], 0
    for line in (run/'follow_decisions.jsonl').open(encoding='utf-8'):
        e = json.loads(line)
        if e['kind'] != 'call' or not start <= e['data']['input']['reference_s'] <= end:
            continue
        d = e['data']['output'][2]
        lat = d.get('m3_lateral') or {}
        reasons[d['reason']] = reasons.get(d['reason'], 0)+1
        modes[str(lat.get('mode'))] = modes.get(str(lat.get('mode')), 0)+1
        rejected += bool(d.get('m3_all_commands_rejected'))
        if any(v.get('present') for v in (lat.get('threats') or {}).values()):
            threats.append(e['data']['input']['reference_s'])
    last = rows[-1] if rows else {}
    ev_ = report['evaluation']
    return {'start_s': start, 'end_s': end, 'longest_stall_s': stall, 'decision_reasons': reasons, 'lateral_modes': modes,
            'all_commands_rejected': rejected, 'threat_present_s': [min(threats), max(threats)] if threats else None,
            'threat_present_calls': len(threats), 'last_trace': {k: last.get(k) for k in ('t', 's', 'progress_m', 'yaw', 'ground_speed_mps')},
            'evaluator_40s_case_metrics': ev_.get('case_metrics'), 'evaluator_40s_gates': ev_.get('gates')}


def main():
    out = Path(sys.argv[1])
    if out.exists():
        raise FileExistsError(out)
    base = Path('artifacts/validation/2026-10-25/m3_execution_review_v1')
    r27, m27, g27 = stop_restart_window(base/'run27_small_center_stop_restart/run', 'config/tests/g1_traffic_v1.json',
                                        'stop_restart', 15.)
    original = r27['evaluation']
    check = {'restart_delay_s': [m27['restart_delay_s'], original['case_metrics']['restart_delay_s']],
             'progress_after_restart_m': [m27['progress_after_restart_m'], original['case_metrics']['progress_after_restart_m']],
             'sampled_gap_at_window_end_m': [m27['sampled_gap_at_window_end_m'], original['progress']['sampled_gap_at_window_end_m']],
             'longest_hold_in_window_s': [m27['longest_hold_in_window_s'], original['case_metrics']['longest_hold_in_window_s']]}
    reproduced = all(abs(a-b) < 1e-9 for a, b in check.values()) and all(g27[k] == original['gates'][k] for k in g27)
    result = {'scope': 'post-hoc window split with evaluator functions; evaluation only, no controller input or verdict rewrite',
              'run27_reproduction': {'matches_original': reproduced, 'values': check}}
    run32 = base/'run32_corner_after_threat/run'
    if run32.is_dir():
        report, metrics, gates = stop_restart_window(run32, 'config/tests/g1_corner_after_threat_v1.json',
                                                     'stop_restart_first_curve', 15.)
        result['run32_stop_restart_15s'] = {'metrics': metrics, 'gates': gates}
        result['run32_after_15s'] = after_window(run32, report['evaluation']['t_green']+15.,
                                              report['evaluation']['t_green']+40., report)
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    assert reproduced


if __name__ == '__main__':
    main()
