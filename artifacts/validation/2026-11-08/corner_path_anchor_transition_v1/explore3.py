"""표현 전환(일반 곡선) 경로에서 반환 경로 법선으로 ±0.2 m 평행 이동한 관측의 앵커 오차(탐색용)."""
import math, sys
import numpy as np
sys.path.insert(0, "src/arena_autonomy")
sys.path.insert(0, sys.argv[1])
from explore2 import corner_scan, O, R
from arena_autonomy.local_path import local_path
for mirror in (1, -1):
    side = 'right' if mirror == 1 else 'left'
    for deg, face in ((35, .3), (35, .2), (40, .3), (30, .2)):
        cloud = corner_scan(mirror, deg, 0., face)
        p, i = local_path(cloud, side, O, piecewise_corner=True, minimum_radius=R)
        k = int(np.argmin(np.linalg.norm(p, axis=1))); k = min(max(k, 1), len(p)-2)
        t = p[k+1]-p[k-1]; t /= np.linalg.norm(t); n = np.array([-t[1], t[0]])
        for d in (-.2, .2):
            q, j = local_path(cloud+d*n, side, O, piecewise_corner=True, minimum_radius=R)
            print(mirror, deg, face, i.get('wall_model'), round(i['path_anchor_error_m'], 4), d, j.get('reason'), j.get('wall_model'),
                  None if q is None else round(j['path_anchor_error_m'], 4), None if q is None else round(j['boundary_observed_ahead_m'], 2))
