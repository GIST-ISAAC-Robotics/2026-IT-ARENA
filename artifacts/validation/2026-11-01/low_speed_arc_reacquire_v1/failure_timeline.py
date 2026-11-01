"""run1의 마지막 코너 진입부터 정지까지 새 스캔 판단을 시간표로 남긴다. 평가 정답은 표에만 쓴다."""
import bisect
import collections
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
RUN = HERE/'run1'
R, T = .0325, .135
with (RUN/'trajectory.jsonl').open(encoding='utf-8') as stream:
    rows = [r for r in map(json.loads, stream) if r['light'] == 'green']
times = [r['dynamics']['sim_time_s'] for r in rows]


def near(t):
    return rows[min(bisect.bisect_left(times, t), len(rows)-1)]


out, reasons, last = [], collections.Counter(), None
print(' ref_s  prog   body  meas  vL    vR    appl  tgt  steer  remain clear  reason | sel cand fresh | cor n | sel_choice sel_reason')
with (RUN/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
    for line in stream:
        row = json.loads(line)
        if row['kind'] != 'call':
            continue
        i = row['data']['input']
        v, s, d = row['data']['output']
        t = i['reference_s']
        r = near(t)
        if r['progress_m'] < 42.:
            continue
        reasons[(d.get('reason'), d.get('underlying_reason'), d.get('selector_reason'))] += 1
        first = i['scan']['sec']+i['scan']['nanosec']*1e-9
        if first == last:
            continue
        last = first
        c = d.get('corridor_candidate') or {}
        dyn = r['dynamics']
        print(f"{t:6.2f} {r['progress_m']:6.2f} {dyn['truth_longitudinal_mps']:5.2f} {i['speed_mps']:+5.2f} "
              f"{dyn['left_speed_rad_s']*R:+5.2f} {dyn['right_speed_rad_s']*R:+5.2f} {dyn['applied_target_mps']:5.2f} {v:4.2f} {s:+.3f} "
              f"{(d.get('observed_path_ahead_m') or -1):5.2f} {(d.get('swept_clearance_m') or -1):5.2f} {d.get('reason')} | "
              f"{d.get('boundary_selection')} {d.get('boundary_candidate_ahead_m') and round(d['boundary_candidate_ahead_m'], 2)} {d.get('fresh_boundary_accepted')} | "
              f"{c.get('reason')} {c.get('confirmations')} | {d.get('selector_choice')} {d.get('selector_reason')} bil={d.get('bilateral_reference')}")
print('reasons after 42 m')
for k, n in reasons.most_common(12):
    print(n, k)
