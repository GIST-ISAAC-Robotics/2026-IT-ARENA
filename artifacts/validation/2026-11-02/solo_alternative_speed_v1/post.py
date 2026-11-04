"""대표 주행 평가 집계. 시작 구간 신호·마커·기준 벽, 0명령 구간, 11/1 선정 주행과의 같은 위치 비교. 평가 전용."""
from collections import Counter
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
PRIOR = HERE.parent/'official_facility_v1'


def load(run, metrics):
    m = json.loads(metrics.read_bytes())
    rows = [json.loads(line) for line in (run/'trajectory.jsonl').read_bytes().splitlines()]
    return m, rows


def span_table(run, begin, end, rows):
    green = [r for r in rows if r['light'] == 'green']
    T = np.array([r['time'] for r in green])
    S = np.array([r['progress_m'] for r in green])
    V = np.array([math.hypot(r['dynamics']['truth_longitudinal_mps'], r['dynamics']['truth_lateral_mps']) for r in green])
    spans, active, total = [], None, Counter()
    for line in (run/'follow_decisions.jsonl').open(encoding='utf-8'):
        e = json.loads(line)
        if e['kind'] != 'call':
            continue
        t = e['data']['input']['reference_s']
        v, _, d = e['data']['output']
        if not begin+2. <= t <= end or v > .001:
            continue
        total[d.get('reason')] += .02
        if active and t-active['end'] <= .13:
            active['end'] = t
            active['reasons'][d.get('reason')] += .02
        else:
            active = dict(start=t, end=t, reasons=Counter({d.get('reason'): .02}))
            spans.append(active)
    for span in spans:
        mask = (T >= span['start']) & (T <= span['end']+.3)
        span.update(s=float(np.interp(span['start'], T, S)), seconds=round(span['end']-span['start']+.02, 2),
                    min_speed=float(np.min(V[mask])) if mask.any() else None,
                    reasons={k: round(v, 2) for k, v in span['reasons'].items()})
    return dict(zero_command_s=round(sum(total.values()), 2), by_reason={k: round(v, 2) for k, v in total.items()}, spans=spans)


def timing(rows, begin):
    green = [r for r in rows if r['light'] == 'green']
    t = np.array([r['time'] for r in green])-begin
    s = np.array([r['progress_m'] for r in green])
    keep = np.r_[True, np.diff(s) > 0]
    return t[keep], s[keep]


def main(name='run1'):
    run = HERE/name
    m, rows = load(run, HERE/f'{name}_metrics.json')
    begin = m['first_green_sample_s']
    report = dict(scope='평가 전용. 평가 정답은 제어 입력이 아니다. 비교 대상 11/2 원래 단독 구성과 동일 월드다.')
    report['report'] = {k: v for k, v in json.loads((run/'report.json').read_bytes()).items() if k in (
        'passed', 'route_complete', 'stop_stable', 'stop_collision_samples', 'collision_samples', 'false_start',
        'signal_states', 'marker_ids', 'sides_seen', 'official_signal_evidence', 'error')}
    pre, seen = [r for r in rows if r['time'] < begin+1.], {}
    for row in rows:
        for marker in row['autonomy'].get('marker_ids', []):
            seen.setdefault(str(marker), dict(first_s=row['time']-begin, first_progress_m=row['progress_m'], side_after=row['autonomy'].get('side')))
    both = [r for r in rows if {0, 45} <= set(r['autonomy'].get('marker_ids', []))]
    report['markers'] = dict(first_seen=seen, sides_before_green_plus_1s=sorted({r['autonomy'].get('side') for r in pre}),
                             samples_with_0_and_45=len(both),
                             side_while_0_and_45=sorted({r['autonomy'].get('side') for r in both}),
                             progress_range_0_and_45=[min(r['progress_m'] for r in both), max(r['progress_m'] for r in both)] if both else None)
    report['stops'] = span_table(run, begin, begin+m['lap_s_from_first_green_sample'], rows)
    prior_m, prior_rows = load(PRIOR/'run1', PRIOR/'run1_metrics.json')
    ta, sa = timing(prior_rows, prior_m['first_green_sample_s'])
    tb, sb = timing(rows, begin)
    edges = np.arange(0., 47., 2.)
    a, b = np.interp(edges, sa, ta), np.interp(edges, sb, tb)
    report['same_progress_vs_facility_baseline'] = [dict(s0=float(x), s1=float(y), prior_s=float(a[i+1]-a[i]), now_s=float(b[i+1]-b[i]),
                                             delta_s=float((b[i+1]-b[i])-(a[i+1]-a[i])))
                                        for i, (x, y) in enumerate(zip(edges, edges[1:]))]
    report['lap'] = dict(now_s=m['lap_s_from_first_green_sample'], prior_facility_baseline_s=prior_m['lap_s_from_first_green_sample'])
    output = HERE/f'{name}_summary.json'
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8', newline='\n')
    print(json.dumps(report['lap']))


if __name__ == '__main__':
    main(*sys.argv[1:])
