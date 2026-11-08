#!/usr/bin/env python3
"""P22 진입 정적 기하 경로의 가능 조합(e, R) 목록 (static_audit.best_turn과 같은 조건, 차체/조향 외곽)."""
import json, math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
import static_audit as sa
from shapely.geometry import box
from shapely.ops import unary_union
info, links = sa.parse_sdf(sa.OF / 'world.sdf')
_, road = sa.link_union(links, 'track_surface', 'collision')
veh = sa.vehicle_params()
allowed = road.intersection(box(4.9, 3.6, 7.0, 6.2))
entry = ((6.14, 6.0), -math.pi / 2)
exit_ = ((6.14, 4.5), math.pi)
radii = [round(veh['rmin_rear_axle_m'], 4), 0.35, 0.45, 0.6, 0.8, 1.0, 1.2]
offs = [round(v, 3) for v in np.arange(-0.2, 0.2001, 0.025)]
out = {}
for dims_name, dims in (('body', veh['body_LxW_m']), ('steered_envelope', veh['steered_envelope_LxW_m'])):
    ok = []
    for e in offs:
        for R in radii:
            r_ = sa.best_turn(allowed, entry, exit_, veh, dims, [0.0], [e], [R], pre=0.5, post=0.5)
            if r_['feasible']:
                ok.append([e, R, r_['best']['margin_m']])
    out[dims_name] = ok
out['note'] = 'e: 진입 전 횡 위치(+는 진행 방향 왼쪽=+x), R: 후축 회전반경. 도로(백색)만 허용.'
(Path(__file__).resolve().parents[1] / 'checks/p22_entry_feasible.json').write_text(json.dumps(out, ensure_ascii=False, indent=1) + '\n', encoding='utf-8')
print(json.dumps(out, ensure_ascii=False))
