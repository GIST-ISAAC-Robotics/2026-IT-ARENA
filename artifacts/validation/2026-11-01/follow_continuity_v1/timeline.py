"""저장 판단 기록을 호출별 표로 풀고 차체 속도·실제 간격(평가 전용)을 시간축에 붙인다.

제어기 입력이 아니라 사후 분석 도구다. 차체 속도/상대 자세는 평가 기록에서만 읽는다.
"""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def binding(v, q):
    state = q.get('handoff_state', 'unavailable')
    if state != 'FOLLOW':
        return state
    if (q.get('relative_yield') or {}).get('final_command_clipped'):
        return 'yield'
    if (q.get('entry_space') or {}).get('hold') and v == 0:
        return 'prepare_hold'
    n = q.get('normal') or {}
    if n.get('behavior') == 'HOLD':
        return 'G1:' + str(n.get('reason'))
    f = q.get('follow_entry_space') or {}
    if f.get('applied') and f.get('before_mps', v) > v + 1e-7:
        return 'entry:' + str(f.get('reason'))
    if (q.get('preparation') or {}).get('applied'):
        return 'prepare_cap'
    if v == 0:
        return 'other_zero:' + str(q.get('reason'))
    return 'G1_or_unchanged'


def rows(run):
    report = json.loads((run / 'report.json').read_bytes())
    green = report['case_window']['green_sim_s']
    out = []
    with (run / 'follow_decisions.jsonl').open('rb') as f:
        for line in f:
            e = json.loads(line)
            if e['kind'] != 'call':
                continue
            d = e['data']
            v, a, q = d['output']
            n = q.get('normal') or {}
            p = q.get('preparation') or {}
            s = q.get('follow_entry_space') or {}
            pers = q.get('persistence') or {}
            es = q.get('entry_space') or {}
            out.append(dict(
                seq=e['seq'], t=d['input']['reference_s'] - green, measured=d['input']['speed_mps'],
                out=v, steer=a, state=q.get('handoff_state'), bind=binding(v, q),
                g1=n.get('target_speed_before_hold_mps'), normal_behavior=n.get('behavior'),
                normal_reason=n.get('reason'), selector=n.get('selector_choice'),
                ahead=n.get('observed_path_ahead_m'), swept=n.get('swept_clearance_m'),
                prep_reason=p.get('reason'), prep_applied=p.get('applied'), rate=p.get('motion_rate_mps'),
                fes_reason=s.get('reason'), fes_cap=s.get('cap_mps'), fes_before=s.get('before_mps'),
                fixed_cap=s.get('fixed_reserve_cap_mps'), axle_to_patch=s.get('rear_axle_to_patch_m'),
                pers_observed=pers.get('observed'), pers_ready=pers.get('ready'), pers_reason=pers.get('reason'),
                pers_n=pers.get('sample_count'), pers_span=pers.get('source_span_s'),
                pers_disp=pers.get('max_compensated_displacement_m'),
                patch_x=(pers.get('front_median_xy') or [None])[0], patch_y=(pers.get('front_median_xy') or [None, None])[1],
                hold=es.get('hold'), entry_reason=es.get('reason')))
    return report, green, out


def attach(run, green, table):
    traj = [x for x in map(json.loads, (run / 'trajectory.jsonl').open(encoding='utf-8'))
            if x.get('elapsed_since_green_s') is not None]
    t = np.array([x['t'] - green for x in traj])
    body = np.array([x['ground_speed_mps'] for x in traj])
    gap_key = next((k for k in ('gap_m', 'body_gap_m', 'actor_gap_m') if k in traj[0]), None)
    gap = np.array([x.get(gap_key) if gap_key else np.nan for x in traj], dtype=float)
    for r in table:
        r['body'] = float(np.interp(r['t'], t, body))
        r['gap'] = float(np.interp(r['t'], t, gap)) if gap_key else None
    return traj, gap_key


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('run', type=Path)
    p.add_argument('out', type=Path)
    a = p.parse_args()
    report, green, table = rows(a.run)
    traj, gap_key = attach(a.run, green, table)
    with a.out.open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(table[0].keys()))
        w.writeheader()
        w.writerows(table)
    print(len(table), 'calls', 'trajectory keys', sorted(traj[0].keys()), 'gap_key', gap_key)
