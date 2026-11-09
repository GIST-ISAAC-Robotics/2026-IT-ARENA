"""C 결과의 지주 간격(평가 전용 2D 외곽): run1/trace.json·ego_poses.jsonl과 등록 지주 상자. 물리 접촉 판정 아님."""
import json, math, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
POSTS = {'left': ((9.420, 9.435), (6.13769, 6.15231)), 'right': ((10.165, 10.180), (6.13769, 6.15231))}
FEET = {'left': ((9.4075, 9.4475), (6.120, 6.170)), 'right': ((10.1525, 10.1925), (6.120, 6.170))}
def rect(x, y, yaw, L=.20, W=.15):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x+lx*c-ly*s, y+lx*s+ly*c) for lx, ly in ((-L/2, -W/2), (L/2, -W/2), (L/2, W/2), (-L/2, W/2))]
def seg_pt(a, b, p):
    dx, dy = b[0]-a[0], b[1]-a[1]; t = max(0, min(1, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(a[0]+t*dx-p[0], a[1]+t*dy-p[1])
def inside(poly, p):
    s = [((b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])) for a, b in zip(poly, poly[1:]+poly[:1])]
    return all(v >= 0 for v in s) or all(v <= 0 for v in s)
def gap(poly, box):
    (x0, x1), (y0, y1) = box; corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    if any(inside(poly, c) for c in corners) or any(x0 <= p[0] <= x1 and y0 <= p[1] <= y1 for p in poly):
        return 0.
    return min(min(seg_pt(a, b, c) for a, b in zip(poly, poly[1:]+poly[:1]) for c in corners),
               min(math.hypot(p[0]-min(max(p[0], x0), x1), p[1]-min(max(p[1], y0), y1)) for p in poly))
run = Path(sys.argv[1]); poses = [json.loads(l) for l in open(run/'ego_poses.jsonl')]
best = {k: (math.inf, None) for k in ('post', 'foot')}; overlap = {'post': 0, 'foot': 0}
for t, x, y, yaw in poses[::2]:
    if not 5.6 <= y <= 6.7: continue
    poly = rect(x, y, yaw)
    for kind, boxes in (('post', POSTS), ('foot', FEET)):
        g = min(gap(poly, b) for b in boxes.values())
        overlap[kind] += g == 0.
        if g < best[kind][0]: best[kind] = (g, dict(t=t, x=round(x, 3), y=round(y, 3)))
out = dict(min_body_to_post_m=round(best['post'][0], 4), at=best['post'][1], min_body_to_foot_m=round(best['foot'][0], 4),
           post_overlap_samples=overlap['post'], foot_overlap_samples=overlap['foot'],
           max_y_reached=round(max(p[2] for p in poses), 3),
           scope='evaluation-only 2D 20x15 cm body vs registered STL-derived post/foot boxes (ego pose every 2nd sample)')
(run.parent/'post_clearance.json').write_text(json.dumps(out, indent=1)+'\n', encoding='utf-8', newline='\n')
print(json.dumps(out, indent=1))
