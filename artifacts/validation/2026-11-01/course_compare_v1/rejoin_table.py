"""앞차 재출발이 있는 실행마다 양보·복귀 진행을 같은 정의로 표로 만든다(사후 평가).

앞선 길이는 평가용 자세의 차체 중심 간 진행 방향 성분이다. 제어 입력이 아니다.
"""
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RUNS = {
    'closing 15초(기준)': ROOT / 'race_follow_closing_v1/stop',
    'moving 15초': ROOT / 'race_follow_pair_v1/moving',
    'recent 15초': ROOT / 'rolling_lead_v1/stop_recent',
    'low_motion 15초': ROOT / 'rolling_lead_v1/stop_low_motion',
    'paced 15초': ROOT / 'rolling_lead_v1/stop_paced',
    'closing 155초(run1)': ROOT / 'mixed_course_v1/run1',
    'paced 155초': ROOT / 'course_compare_v1/run_paced',
}


def row(run):
    r = json.loads((run / 'report.json').read_bytes())
    green = r['case_window']['green_sim_s']
    poses = np.array([json.loads(x) for x in (run / 'ego_poses.jsonl').open(encoding='utf-8')])
    actor = np.array([json.loads(x)[:3] for x in (run / 'actor_poses.jsonl').open(encoding='utf-8')])
    entry = rejoin = None
    budget_end = None
    capped = 0.
    zero = 0.
    prev = None
    expiry = None
    with (run / 'follow_decisions.jsonl').open('rb') as f:
        for line in f:
            e = json.loads(line)
            if e['kind'] != 'call':
                continue
            d = e['data']
            t = d['input']['reference_s'] - green
            v, _, q = d['output']
            state = q.get('handoff_state')
            c = q.get('bypass') or {}
            y = q.get('relative_yield') or {}
            if state == 'BYPASS' and entry is None:
                entry = t
            if entry is not None and rejoin is None:
                if c.get('entry_phase') == 'rejoin':
                    rejoin = t
                dt = 0. if prev is None else min(t - prev, .05)
                if state == 'BYPASS' and t >= 9.:
                    if v <= .001:
                        zero += dt
                    elif v <= .2001:
                        capped += dt
                        budget_end = t
                if expiry is None and 'budget' in str(y):
                    expiry = t
            prev = t
    def along(t):
        ego = poses[np.argmin(np.abs(poses[:, 0] - (t + green)))]
        act = actor[np.argmin(np.abs(actor[:, 0] - (t + green)))]
        return float((act[1] - ego[1]) * np.cos(ego[3]) + (act[2] - ego[2]) * np.sin(ego[3]))
    return dict(reserve=r.get('handoff_reserve'), bypass_entry_s=entry, rejoin_s=rejoin, zero_after_restart_s=round(zero, 2),
                yield_020_after_restart_s=round(capped, 2), last_yield_s=budget_end, budget_note_s=expiry,
                lead_ahead_at_last_yield_m=None if budget_end is None else round(along(budget_end), 3),
                lead_ahead_at_rejoin_m=None if rejoin is None else round(along(rejoin), 3))


result = {name: row(run) for name, run in RUNS.items()}
for name, value in result.items():
    print(name, value)
with Path(sys.argv[1]).open('x', encoding='utf-8') as f:
    json.dump(dict(rows=result, scope='앞선 길이는 평가 자세 기준 차체 중심 간 진행 방향 성분. 제어 입력 아님. 단일 실행 표본.'), f, ensure_ascii=False, indent=2)
