"""새 스캔이 처리된 판단만 진행 거리와 함께 나열한다."""
import bisect
import gzip
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
NAMES = ('rise_only', 'turn_only', 'combined_failed')


def load(name):
    with gzip.open(HERE / f'extract_{name}.json.gz', 'rt', encoding='utf-8') as f:
        return json.load(f)


lo, hi = float(sys.argv[1]), float(sys.argv[2])
for n in NAMES:
    d = load(n)
    rows = [r for r in d['trajectory'] if r['light'] == 'green']
    times = [r['sim_time_s'] for r in rows]
    print(f'# {n}')
    print(' ref_s   prog  body  meas  tgt   steer  remain clear sel cand_ahead fresh wall reason')
    for c in d['calls']:
        if c['kind'] != 'call' or c['boundary_selection'] is None:
            continue
        i = bisect.bisect_left(times, c['reference_s'])
        if i >= len(rows):
            continue
        r = rows[i]
        if not lo <= r['progress_m'] <= hi:
            continue
        print(f"{c['reference_s']:6.2f} {r['progress_m']:6.2f} {r['truth_longitudinal_mps']:5.2f} {c['speed_mps']:5.2f} "
              f"{c['target']:5.2f} {c['steering']:+6.3f} {(c['observed_path_ahead_m'] or -1):5.2f} "
              f"{(c['swept_clearance_m'] or -1):5.2f} {c['boundary_selection']} "
              f"{(c['boundary_candidate_ahead_m'] or -1):5.2f} {c['fresh_boundary_accepted']} {c['path_wall']} {c['reason']}")
