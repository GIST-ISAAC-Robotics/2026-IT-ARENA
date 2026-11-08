"""저장 센서의 두 벽에 대한 접선 원호 산술. 주행/자유 공간 판정이 아니다."""
from pathlib import Path
import math
import json

HERE = Path(__file__).resolve().parent
summary = json.loads((HERE/'summary.json').read_bytes())
cross = lambda v, w: v[0]*w[1]-v[1]*w[0]
rows = []
for q in summary['near_candidates']:
    c = q['candidate']
    a = math.radians(c['straight_deg'])
    b = a+math.radians(c['turn_deg'])
    t, u = (math.cos(a), math.sin(a)), (math.cos(b), math.sin(b))
    for incoming in (.505, .3):
        p = (c['x']+incoming*t[1], c['y']-incoming*t[0])
        r = (c['x']+.3*u[1], c['y']-.3*u[0])
        distance = cross((r[0]-p[0], r[1]-p[1]), u)/cross(t, u)
        lead = .36*math.tan(abs(b-a)/2)
        start = (p[0]+(distance-lead)*t[0], p[1]+(distance-lead)*t[1])
        rows.append(dict(t=q['t'], incoming_wall_offset_m=incoming,
            outgoing_wall_offset_m=.3, radius_m=.36, tangent_start_xy_m=start,
            first_straight_center_y_m=p[1]))
result = dict(scope='analytic tangent geometry from fixed observed lines; no control, no footprint/occupancy or dynamic proof',
    minimum_rear_radius_m=.135/math.tan(.37),
    minimum_center_radius_m=math.hypot(.135/math.tan(.37), .135/2), rows=rows)
(HERE/'tangent_comparison.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
