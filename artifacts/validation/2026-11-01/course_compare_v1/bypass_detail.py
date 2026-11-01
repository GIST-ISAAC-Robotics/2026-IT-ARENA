"""BYPASS 진행과 후보 거부 이유를 시간대별로 비교한다. 상대 자세는 사후 평가에만 쓴다."""
from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'follow_continuity_v1'))


def actor_at(rows, t):
    i = min(range(len(rows)), key=lambda k: abs(rows[k][0] - t))
    return rows[i]


def main(run, stamps):
    r = json.loads((run / 'report.json').read_bytes())
    green = r['case_window']['green_sim_s']
    poses = np.array([json.loads(x) for x in (run / 'ego_poses.jsonl').open(encoding='utf-8')])
    actor = np.array([json.loads(x)[:3] for x in (run / 'actor_poses.jsonl').open(encoding='utf-8')])
    want = sorted(stamps)
    k = 0
    with (run / 'follow_decisions.jsonl').open('rb') as f:
        for line in f:
            e = json.loads(line)
            if e['kind'] != 'call':
                continue
            d = e['data']
            t = d['input']['reference_s'] - green
            if k >= len(want) or t < want[k]:
                continue
            k += 1
            v, a, q = d['output']
            b = q.get('bypass') or {}
            sel = b.get('selected') or {}
            cand = b.get('candidates') or []
            ego = poses[np.argmin(np.abs(poses[:, 0] - (t + green)))]
            act = actor[np.argmin(np.abs(actor[:, 0] - (t + green)))]
            n = q.get('normal') or {}
            print(f"t={t:6.2f} {q.get('handoff_state')} v={v:.2f} steer={a:+.3f} meas={d['input']['speed_mps']:.2f} phase={b.get('entry_phase')} reason={b.get('reason')}"
                  f" ego=({ego[1]:.2f},{ego[2]:.2f},{ego[3]:.2f}) lead=({act[1]:.2f},{act[2]:.2f}) d={np.hypot(ego[1]-act[1],ego[2]-act[2]):.2f}")
            print(f"        sel off={sel.get('offset_m')} tr={sel.get('transition_m')} speed={sel.get('speed_mps')} preview={sel.get('preview_point_gap_m')} g1={n.get('reason')} ahead={n.get('observed_path_ahead_m')} swept={n.get('swept_clearance_m')}")
            print('        cand', dict(Counter(str(c.get('reason')) for c in cand)), 'keys', sorted(k2 for k2 in b.keys() if k2 not in ('candidates', 'selected', 'corridor', 'corridor_plan', 'reference'))[:14])


if __name__ == '__main__':
    main(Path(sys.argv[1]), [float(x) for x in sys.argv[2:]])
