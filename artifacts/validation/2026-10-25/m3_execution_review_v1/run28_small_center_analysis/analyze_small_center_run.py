"""정지·재개 주행의 저장 판단·자세 읽기 전용 대조(run19 대 run27). 제어 재실행·판정 변경 없음.

위협 소멸 뒤 전 후보 거절 HOLD, 작은 중앙 보정 생략 적용/직진 선택, 예측 편차, 0이 아닌 조향의 활성 양수,
앞차 재개 구간의 자차 정지 시간을 나란히 적는다. 정답 자세는 사후 평가용으로만 읽는다.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path


def decisions(run):
    rows = []
    for line in (run/'follow_decisions.jsonl').open(encoding='utf-8'):
        event = json.loads(line)
        if event['kind'] == 'header':
            header = event['data']
        if event['kind'] != 'call':
            continue
        speed, steering, detail = event['data']['output']
        lateral = detail.get('m3_lateral') or {}
        rows.append({'t': event['data']['input']['reference_s'], 'speed': speed, 'steering': steering,
                     'reason': detail.get('reason'), 'mode': lateral.get('mode'), 'selected': lateral.get('selected'),
                     'all_rejected': bool(detail.get('m3_all_commands_rejected')),
                     'present': any(v.get('present') for v in (lateral.get('threats') or {}).values()),
                     'small': lateral.get('small_center'), 'state': lateral.get('state') or {},
                     'released': bool(lateral.get('released_to_baseline')),
                     'gate': (lateral.get('release_gate') or {}).get('reason')})
    return header, rows


def stopped_spans(run, first, last, threshold=.01):
    """정답 자세 차분 속도 < threshold인 연속 구간(사후 평가). 반환: 최장, 합계."""
    samples = [json.loads(line) for line in (run/'ego_poses.jsonl').open(encoding='utf-8')]
    samples = [s for s in samples if first <= s[0] <= last]
    longest = total = 0.
    start = None
    for a, b in zip(samples, samples[1:]):
        dt = b[0]-a[0]
        if dt <= 0:
            continue
        v = ((b[1]-a[1])**2+(b[2]-a[2])**2)**.5/dt
        if v < threshold:
            start = a[0] if start is None else start
            total += dt
            longest = max(longest, b[0]-start)
        else:
            start = None
    return longest, total


def summarize(run):
    header, rows = decisions(run)
    report = json.loads((run/'report.json').read_text(encoding='utf-8'))
    ev = report['evaluation']
    green = ev['t_green']
    present = [r['t'] for r in rows if r['present']]
    after = [r for r in rows if present and r['t'] > max(present)]
    active = [r for r in after if r['mode'] not in (None, 'BASELINE')]
    small = [r for r in after if r['small']]
    ok = [r for r in small if r['small']['ok']]
    lead_restart = green+9.
    longest, total = stopped_spans(run, lead_restart, green+15.)
    return {'execution_config': header['execution_config'], 'calls': len(rows),
            'threat_first_last_s': [min(present), max(present)] if present else None,
            'after_threat_calls': len(after), 'after_threat_active_calls': len(active),
            'all_rejected_after_threat': len([r for r in after if r['all_rejected']]),
            'all_rejected_after_threat_s': [round(r['t'], 3) for r in after if r['all_rejected']],
            'active_positive': sum(r['speed'] > 0 for r in active),
            'active_positive_nonzero_steering': sum(r['speed'] > 0 and r['steering'] != 0. for r in active),
            'modes_after_threat': dict(Counter(r['mode'] for r in active)),
            'reasons_after_threat': dict(Counter(r['reason'] for r in after)),
            'gate_reasons': dict(Counter(r['gate'] for r in after if r['gate'])),
            'release_events_s': [round(r['t'], 3) for r in rows if r['released']],
            'small_center_calls': len(small), 'small_center_ok': len(ok),
            'small_center_ok_selected_straight': sum(r['selected'] == 'straight' for r in ok),
            'small_center_not_ok_curved': sum(1 for r in small if not r['small']['ok'] and r['small']['curved']),
            'small_center_not_ok_invalid': sum(1 for r in small if not r['small']['ok'] and not r['small']['valid']),
            'small_center_not_ok_offset': sum(1 for r in small if not r['small']['ok'] and r['small']['valid']
                                              and not r['small']['curved']),
            'max_predicted_offset_when_ok_m': max((r['small']['max_abs_offset_m'] for r in ok), default=None),
            'max_abs_d0_when_ok_m': max((abs(r['state']['d0']) for r in ok), default=None),
            'max_abs_psi_when_ok_rad': max((abs(r['state']['psi']) for r in ok), default=None),
            'max_abs_d0_active_after_threat_m': max((abs(r['state']['d0']) for r in active if r['state']), default=None),
            'lead_moving_window_s': [lead_restart, green+15.],
            'ego_longest_stop_while_lead_moving_s': longest, 'ego_total_stop_while_lead_moving_s': total,
            'passed': report['passed'], 'run_valid': report['run_valid'], 'gates': report['gates'],
            'case_metrics': ev['case_metrics'], 'progress': ev['progress'],
            'min_sampled_gap_m': ev['gap']['min_sampled_gap_m'], 'min_swept_gap_m': ev['gap']['min_swept_lower_bound_m'],
            'decisions_sha256': hashlib.sha256((run/'follow_decisions.jsonl').read_bytes()).hexdigest()}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('runs', type=Path, nargs='+')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result = {'scope': 'stored closed-loop outputs and evaluation-only poses; no control rerun or verdict change',
              'runs': {run.parent.name: summarize(run) for run in args.runs}}
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
