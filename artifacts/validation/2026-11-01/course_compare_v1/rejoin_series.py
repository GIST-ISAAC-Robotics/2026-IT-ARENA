"""스캔마다 복귀 근거(rejoin_band)와 양보 상태를 뽑는다. 앞차 거리는 사후 평가 자세다."""
import json
from pathlib import Path
import sys

import numpy as np


def main(run, a, b, step):
    r = json.loads((run / 'report.json').read_bytes())
    green = r['case_window']['green_sim_s']
    poses = np.array([json.loads(x) for x in (run / 'ego_poses.jsonl').open(encoding='utf-8')])
    actor = np.array([json.loads(x)[:3] for x in (run / 'actor_poses.jsonl').open(encoding='utf-8')])
    seen = None
    nxt = a
    with (run / 'follow_decisions.jsonl').open('rb') as f:
        for line in f:
            e = json.loads(line)
            if e['kind'] != 'call':
                continue
            d = e['data']
            t = d['input']['reference_s'] - green
            if t < nxt or t > b:
                continue
            nxt = t + step
            v, _, q = d['output']
            c = q.get('bypass') or {}
            band = c.get('rejoin_band') or {}
            y = q.get('relative_yield') or {}
            ego = poses[np.argmin(np.abs(poses[:, 0] - (t + green)))]
            act = actor[np.argmin(np.abs(actor[:, 0] - (t + green)))]
            dx, dy = act[1] - ego[1], act[2] - ego[2]
            along = dx * np.cos(ego[3]) + dy * np.sin(ego[3])
            cross = -dx * np.sin(ego[3]) + dy * np.cos(ego[3])
            brief = {k: band.get(k) for k in ('supported', 'front_unobserved', 'rear_unobserved', 'current_occupied_returns')}
            print(f"{t:6.2f} {q.get('handoff_state')} {c.get('entry_phase')} v={v:.2f} body_lead_along={along:+.2f} cross={cross:+.2f} conf={c.get('rejoin_confirmations')} band={brief}"
                  f" yield={ {k: y.get(k) for k in ('active', 'reason', 'final_command_clipped', 'target_mps', 'cap_mps') if k in y} }")


if __name__ == '__main__':
    main(Path(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
