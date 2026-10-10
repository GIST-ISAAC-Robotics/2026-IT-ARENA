"""8.02초(scan 7.902) 이전 소스 대안 거절 이유 확인(분석 전용). path_agrees·local_path는 수정 전후 동일."""
import json, math, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface'), str(ROOT/'tests')]
from arena_autonomy.local_follow import FollowConfig, path_agrees
from arena_autonomy.local_path import local_path
from test_local_follow import history, preserved_scene  # noqa: E402
data = json.loads((HERE/'failure_window.json').read_text())
rows = {round(r['stamp_s'], 3): r for r in data['scans']}
h = history(end=10.2)
for stamp in (7.902, 9.802):
    pts = np.asarray(preserved_scene(rows[stamp], h, round(stamp+.118, 3)).occupied_endpoints_xy_m)
    L, li = local_path(pts, 'left'); R, ri = local_path(pts, 'right')
    print(stamp, 'left', round(li['boundary_observed_ahead_m'], 3), 'right', round(ri['boundary_observed_ahead_m'], 3),
          'agree(right,left)', path_agrees(R, L, FollowConfig().boundary_agreement_m))
