"""장면 성립·계획/보호/MCU/모터축/차체 속도를 시간축으로 맞춰 첫 병목을 찾는다. 평가 전용."""
from collections import Counter
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent


def load_rows(run):
    return json.loads((run/'trace.json').read_bytes())


def spans(times, flags, gap=.15):
    out, begin, last = [], None, None
    for t, flag in zip(times, flags):
        if flag:
            if begin is None or t-last > gap:
                if begin is not None:
                    out.append((begin, last))
                begin = t
            last = t
    if begin is not None:
        out.append((begin, last))
    return [(round(a, 2), round(b, 2)) for a, b in out]


def main(name):
    run = HERE/name
    report = json.loads((run/'report.json').read_bytes())
    evaluation = report.get('evaluation', {})
    green = evaluation.get('t_green')
    trace = load_rows(run)
    decisions = []
    for line in (run/'follow_decisions.jsonl').open(encoding='utf-8'):
        e = json.loads(line)
        if e['kind'] == 'call':
            i, o = e['data']['input'], e['data']['output']
            d = o[2]
            decisions.append((i['reference_s']-green, o[0], d.get('reason'), d.get('behavior'), d.get('swept_clearance_m'), d.get('observed_path_ahead_m'), d.get('target_speed_before_hold_mps')))
    t = np.array([r['t'] for r in trace])-green
    ground = np.array([r['ground_speed_mps'] for r in trace])
    safe = np.array([r['safe_speed_mps'] for r in trace])
    drive = np.array([r.get('drive_speed_mps', np.nan) for r in trace])
    gap = np.array([r.get('sampled_gap_m', np.nan) for r in trace], dtype=float)
    behavior = [r['autonomy'].get('behavior') for r in trace]
    rows = []
    for edge in np.arange(0., t.max(), 1.):
        mask = (t >= edge) & (t < edge+1)
        if not mask.any():
            continue
        calls = [c for c in decisions if edge <= c[0] < edge+1]
        reasons = Counter(c[2] for c in calls)
        rows.append(dict(t=float(edge), ground=round(float(ground[mask].mean()), 2), drive=round(float(np.nanmean(drive[mask])), 2),
                         safe=round(float(safe[mask].mean()), 2), gap=round(float(np.nanmean(gap[mask])), 2) if np.isfinite(gap[mask]).any() else None,
                         plan_mean=round(float(np.mean([c[1] for c in calls])), 2) if calls else None,
                         behavior=dict(Counter(b for b, m in zip(behavior, mask) if m)),
                         reasons={k: v for k, v in reasons.most_common(4)}))
    zero = [c[0] for c in decisions if c[1] <= .001]
    summary = dict(gates=evaluation.get('gates'), metrics=evaluation.get('case_metrics'), gap=evaluation.get('gap'), run_valid=report.get('run_valid'),
                   passed=report.get('passed'), integrity=report.get('integrity'), stop_checks=report.get('stop_checks'),
                   signal=report.get('official_signal_evidence'), merge=report.get('merge_motion') or report.get('merge_audit'),
                   green=green, start=[r['t']-green for r in trace if r['autonomy'].get('started')][:1],
                   zero_command_spans_after_green=spans(zero, [True]*len(zero)), timeline=rows, error=report.get('error'))
    (HERE/f'{name}_summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding='utf-8', newline='\n')
    print(json.dumps({k: summary[k] for k in ('run_valid', 'passed', 'gates', 'metrics', 'zero_command_spans_after_green', 'error')}, ensure_ascii=False)[:1500])
    for row in rows:
        print(row)


if __name__ == '__main__':
    main(sys.argv[1])
