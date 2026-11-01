"""기준과 후보 실제 주행을 같은 지표로 나란히 둔다. 원래 판정 실패는 그대로 옮긴다(사후 평가).

한 번씩의 실행이며 궤적과 스캔 위상이 다르다. 차이 전부를 후보의 인과로 단정하지 않는다.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE.parent / 'rolling_lead_v1'))
from bindings import read as binding_read
from slip_events import body_speed, events


def windows(case, duration):
    return {'first_curve': (0., 6., 13., duration), 'stop_restart': (0., 4., 9., duration)}.get(case, (0., duration))


def drop(t, v, a, b, width=.30):
    grid = t[(t >= max(t[0], a) + width) & (t <= b)]
    if not len(grid):
        return None
    d = np.interp(grid - width, t, v) - np.interp(grid, t, v)
    i = int(np.argmax(d))
    return dict(mps=max(0., float(d[i])), start_s=float(grid[i] - width), end_s=float(grid[i]))


def summary(folder, surface):
    r = json.loads((folder / 'report.json').read_text(encoding='utf-8'))
    green = r['case_window']['green_sim_s']
    duration = r['case_window']['duration_s']
    with (folder / 'trajectory.jsonl').open(encoding='utf-8') as f:
        rows = [x for x in map(json.loads, f) if x.get('elapsed_since_green_s') is not None]
    tt = np.array([x['t'] - green for x in rows])
    gap = np.array([x['sampled_gap_m'] if x['sampled_gap_m'] is not None else np.nan for x in rows], dtype=float)
    reasons = [str(x['safety_reason']) for x in rows if 0 <= x['t'] - green <= duration]
    t, body, _, _ = body_speed(folder, green)
    keep = (t >= 0) & (t <= duration)
    t, body = t[keep], body[keep]
    dt = np.diff(t, append=t[-1])
    moved = np.flatnonzero(body >= .05)
    start = t[moved[0]] if len(moved) else duration
    cuts = windows(r['case'], duration)
    phases = []
    for a, b in zip(cuts, cuts[1:]):
        i = (t >= a) & (t < b)
        phases.append(dict(window_s=[a, b], mean_body_mps=float(np.sum(body[i] * dt[i]) / max(np.sum(dt[i]), 1e-9)),
                           min_body_after_start_mps=float(np.min(body[i & (t >= start)])) if (i & (t >= start)).any() else None,
                           worst_drop_030=drop(t, body, a, b)))
    bind = binding_read(folder)['phases'] if r['case'] == 'first_curve' else None
    s = json.loads(surface.read_text(encoding='utf-8')) if surface and surface.exists() else {}
    return dict(run=str(folder), case=r['case'], run_valid=r['run_valid'], source_unchanged=r['source_unchanged'],
                shutdown_clean=r['shutdown_clean'], passed=r['passed'],
                failed_gates=[k for k, v in r['gates'].items() if not v],
                stop_checks=r['stop_checks'], progress_m=r['evaluation']['progress']['at_window_end_m'],
                final_gap_m=r['evaluation']['progress']['sampled_gap_at_window_end_m'],
                min_gap_m=r['evaluation']['gap']['min_sampled_gap_m'],
                contact_messages=r['evaluation']['gap']['actor_contact_messages_with_ego'],
                wall_collision_samples=r['wall_collision_samples'], guard_samples={k: reasons.count(k) for k in sorted(set(reasons))},
                peak_body_mps=float(np.max(body)), after_start_below_005_s=float(np.sum(dt[(t >= start) & (body < .05)])),
                after_start_below_020_s=float(np.sum(dt[(t >= start) & (body < .20)])), phases=phases,
                wheel_body_mismatch=events(folder), binding_s=None if bind is None else [dict(window_s=p['window_s'], binding_s=p['binding_s'], zero_s=p['zero_s']) for p in bind],
                surface=dict(max_off_road_wheels=s.get('summary', {}).get('max_off_road_wheels', s.get('max_off_road_wheels')),
                             keys=sorted(s.get('summary', {}).keys()) if 'summary' in s else sorted(s.keys())),
                follow_decisions_sha256=r['follow_decisions']['sha256'], handoff_reserve=r.get('handoff_reserve'),
                final_sample_gap_series_tail=[float(x) for x in gap[-3:]])


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('base', type=Path)
    p.add_argument('new', type=Path)
    p.add_argument('out', type=Path)
    p.add_argument('--base-surface', type=Path)
    p.add_argument('--new-surface', type=Path)
    a = p.parse_args()
    base = summary(a.base, a.base_surface)
    new = summary(a.new, a.new_surface)
    if base['case'] != new['case']:
        raise ValueError('서로 다른 각본')
    with a.out.open('x', encoding='utf-8') as f:
        json.dump(dict(base=base, candidate=new,
                       limits='각 설정 한 번씩의 시뮬레이션 실행. 통계 우열·실차 성능을 증명하지 않는다.'), f, ensure_ascii=False, indent=2)
    print(json.dumps({k: {x: base[x] if k == 'base' else new[x] for x in
                          ('progress_m', 'final_gap_m', 'min_gap_m', 'contact_messages', 'failed_gates', 'after_start_below_005_s')}
                      for k in ('base', 'candidate')}, ensure_ascii=False))
