"""42–44 m 전후 계획기 판단 요약(평가 전용, 사후 등록 외 추가 집계). 사용: window_42_44.py run1 → run1_window_42_44.json."""
from collections import Counter
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent


def main(name):
    run = HERE/name
    rows = [json.loads(x) for x in (run/'trajectory.jsonl').read_text(encoding='utf-8').splitlines() if x.strip()]
    green = [r for r in rows if r['light'] == 'green']
    t = np.array([r['time'] for r in green])
    s = np.array([r['progress_m'] for r in green])
    v = np.array([math.hypot(r['dynamics']['truth_longitudinal_mps'], r['dynamics']['truth_lateral_mps']) for r in green])
    keep = np.r_[True, np.diff(s) > 0]
    t0, t1 = float(np.interp(40., s[keep], t[keep])), float(np.interp(46., s[keep], t[keep]))
    calls, reasons = [], Counter()
    with (run/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        next(stream)
        for line in stream:
            e = json.loads(line)
            if e['kind'] != 'call':
                continue
            x = e['data']['input']['reference_s']
            if not t0 <= x <= t1:
                continue
            cmd, steer, d = e['data']['output']
            key = (d.get('reason'), d.get('behavior'), d.get('path_wall'), d.get('wall_fallback'), d.get('open_route_speed_enabled'))
            reasons[json.dumps(key)] += .02
            calls.append(dict(t=round(x, 2), s=round(float(np.interp(x, t, s)), 2), ground=round(float(np.interp(x, t, v)), 3),
                              cmd=round(cmd, 3), reason=d.get('reason'), behavior=d.get('behavior'), path_wall=d.get('path_wall'),
                              wall_fallback=d.get('wall_fallback'), open_route=d.get('open_route_speed_enabled'),
                              path_ahead_m=d.get('observed_path_ahead_m'), swept_m=d.get('swept_clearance_m'),
                              before_hold=d.get('target_speed_before_hold_mps'), boundary_age_s=d.get('boundary_age_s')))
    out = dict(scope='평가 전용. 진행 40–46 m 구간의 follow 판단(50 Hz)을 0.1 s마다 한 줄로 줄였다. 원인 확정이 아닌 진단 입력.',
               window_sim_s=[round(t0, 3), round(t1, 3)],
               time_by_reason_behavior_wall_fallback_openroute={k: round(x, 2) for k, x in reasons.most_common()},
               rows_every_0p1s=calls[::5])
    (HERE/f'{name}_window_42_44.json').write_text(json.dumps(out, ensure_ascii=False, indent=1, default=str), encoding='utf-8', newline='\n')
    print(json.dumps(out['time_by_reason_behavior_wall_fallback_openroute'], ensure_ascii=False))


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'run1')
