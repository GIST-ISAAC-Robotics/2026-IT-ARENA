import json, math, sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.local_path import mask_scan
run = Path(sys.argv[1]); stamp = float(sys.argv[2])
scans = [json.loads(l) for l in (run/'scans_received.jsonl').read_text().splitlines()]
row = min(scans, key=lambda r: abs(r['stamp_s']-stamp))
ego = np.array([json.loads(l) for l in (run/'ego_poses.jsonl').read_text().splitlines()])
i = np.argmin(abs(ego[:,0]-(row['stamp_s']+.05)))
print('scan', row['stamp_s'], 'ego truth (eval only)', ego[i])
a = row['angle_min']+np.arange(len(row['ranges']))*row['angle_increment']
r = np.array([np.nan if v is None else v for v in row['ranges']])
# static sensor-frame (no motion comp) points in base_link, lidar x=.06
p = np.column_stack((.06+r*np.cos(a), r*np.sin(a)))
sel = (p[:,1] > .1) & (p[:,0] > -.6) & (p[:,0] < 2.2) & np.isfinite(r)
q = p[sel]; qa = a[sel]
order = np.argsort(-qa)
q = q[order]
prev = None
for pt in q:
    gap = None if prev is None else np.linalg.norm(pt-prev)
    lim = None if prev is None else .10+.035*min(np.linalg.norm(pt), np.linalg.norm(prev))
    flag = 'BREAK' if gap is not None and gap > lim else ''
    print(f"{pt[0]:7.3f} {pt[1]:7.3f}  ang {math.degrees(math.atan2(pt[1],pt[0])):7.2f}  gap {gap if gap is None else round(gap,3)} lim {lim if lim is None else round(lim,3)} {flag}")
    prev = pt
