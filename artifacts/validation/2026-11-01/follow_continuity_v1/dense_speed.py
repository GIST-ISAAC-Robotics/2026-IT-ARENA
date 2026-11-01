"""고밀도 자세로 만든 차체 속도와 모터축 측정·명령을 비교한다(사후 평가 전용)."""
import json
import sys
from pathlib import Path

import numpy as np

run = Path(sys.argv[1])
a, b = float(sys.argv[2]), float(sys.argv[3])
report = json.loads((run / 'report.json').read_bytes())
green = report['case_window']['green_sim_s']
poses = np.array([json.loads(x) for x in (run / 'ego_poses.jsonl').open(encoding='utf-8')])
t = poses[:, 0] - green
grid = np.arange(a, b, .1)
for u in grid:
    i = (t >= u - .03) & (t <= u + .03)
    if i.sum() < 3:
        continue
    q = poses[i]
    d = np.hypot(q[-1, 1] - q[0, 1], q[-1, 2] - q[0, 2])
    print(f'{u:6.2f} body={d / (q[-1, 0] - q[0, 0]):.3f}')
