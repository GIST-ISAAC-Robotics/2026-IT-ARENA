"""준비 정지 호출 때 앞차 실제 속도와 모터축-차체 불일치를 함께 본다(사후 평가 전용).

상대 자세와 차체 자세는 평가 기록이다. 제어기에는 들어가지 않는다.
"""
import json
import sys
from pathlib import Path

import numpy as np


def speed(t, x, y, at, half=.15):
    out = []
    for u in at:
        i = (t >= u - half) & (t <= u + half)
        out.append(np.hypot(x[i][-1] - x[i][0], y[i][-1] - y[i][0]) / max(t[i][-1] - t[i][0], 1e-6) if i.sum() >= 2 else np.nan)
    return np.array(out)


def run(folder):
    report = json.loads((folder / 'report.json').read_bytes())
    green = report['case_window']['green_sim_s']
    poses = np.array([json.loads(x) for x in (folder / 'ego_poses.jsonl').open(encoding='utf-8')])
    ego_t = poses[:, 0] - green
    actor = [json.loads(x) for x in (folder / 'actor_poses.jsonl').open(encoding='utf-8')]
    if isinstance(actor[0], dict):
        keys = list(actor[0].keys())
        raise SystemExit(f'actor 표본 형식 확인 필요: {keys}')
    actor = np.array([x[:3] for x in actor], dtype=float)
    actor_t = actor[:, 0] - green
    calls = []
    with (folder / 'follow_decisions.jsonl').open('rb') as f:
        for line in f:
            e = json.loads(line)
            if e['kind'] != 'call':
                continue
            d = e['data']
            q = d['output'][2]
            calls.append((d['input']['reference_s'] - green, d['input']['speed_mps'], d['output'][0],
                          bool((q.get('entry_space') or {}).get('hold')) and d['output'][0] == 0.,
                          (q.get('follow_entry_space') or {}).get('reason')))
    t = np.array([c[0] for c in calls])
    duration = report['case_window']['duration_s']
    keep = (t >= 0) & (t <= duration)
    calls = [c for c, k in zip(calls, keep) if k]
    t = t[keep]
    wheel = np.convolve(np.array([c[1] for c in calls]), np.ones(5) / 5, mode='same')
    body = speed(ego_t, poses[:, 1], poses[:, 2], t, .04)
    lead = speed(actor_t, actor[:, 1], actor[:, 2], t)
    dt = np.diff(t, append=t[-1])
    hold = np.array([c[3] for c in calls]) & (dt <= .05)
    mismatch = body - wheel > .25
    moving = lead > .3
    return dict(hold_s=float(dt[hold].sum()), hold_lead_moving_s=float(dt[hold & moving].sum()),
                hold_lead_moving_mismatch_s=float(dt[hold & moving & mismatch].sum()),
                hold_lead_stopped_s=float(dt[hold & ~moving].sum()),
                lead_speed_at_hold=[float(np.nanmin(lead[hold])), float(np.nanmax(lead[hold]))] if hold.any() else None,
                mismatch_total_s=float(dt[mismatch].sum()))


if __name__ == '__main__':
    result = {name: run(Path(name)) for name in sys.argv[2:]}
    for k, v in result.items():
        print(k, v)
    Path(sys.argv[1]).write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding='utf-8')
