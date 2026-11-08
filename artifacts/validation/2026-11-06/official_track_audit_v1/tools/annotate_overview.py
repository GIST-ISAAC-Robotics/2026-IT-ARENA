#!/usr/bin/env python3
"""render_02 전체 상부 원본 PNG에 위치 표기만 덧그린 사본을 만든다(원본 PNG는 그대로 둔다).

WSL: python3 -I tools/annotate_overview.py
또한 보고서용 보조 수치(도로 구멍 면적, 도로+잔디 경계 총길이)를 checks/overview_numbers.json에 쓴다.
"""
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import cv2
import static_audit as sa

ART = Path(__file__).resolve().parents[1]
R = ART / 'checks/gazebo/render_02'
res = json.loads((R / 'result.json').read_text(encoding='utf-8'))
view = next(v for v in res['views'] if v['name'] == 'overview_top')
img = cv2.imread(str(R / view['png']))
H, W = img.shape[:2]
cx, cy, h = view['pos']
mpp = 2 * h * math.tan(view['hfov_rad'] / 2) / W


def px(x, y):
    return int(round(W / 2 + (x - cx) / mpp)), int(round(H / 2 - (y - cy) / mpp))


static = json.loads((ART / 'checks/static_audit.json').read_text(encoding='utf-8'))
labels = []
for c in static['centerline_csv']['corners']:
    labels.append((c['xy'], f"C{c['index']}{'L' if c['direction'] == 'left' else 'R'}", (0, 90, 200)))
for s in static['facilities']['savepoints']:
    labels.append((s['manual_xy'], f"SP{s['n']}", (0, 0, 220)))
for m in static['facilities']['markers']:
    off = {0: (0.12, -0.25), 45: (-0.55, -0.25), 20: (0.05, 0.15), 30: (0.08, 0.1)}[m['id']]
    labels.append(((m['sdf_pose'][0] + off[0], m['sdf_pose'][1] + off[1]), f"ID{m['id']}", (160, 0, 160)))
labels += [((9.1, 6.35), 'tl_post', (0, 0, 180)), ((9.85, 6.62), 'S/F s=0', (0, 0, 0)),
           ((5.0, 4.62), 'P22', (200, 90, 0)), ((3.3, 2.3), 'P21', (200, 90, 0)), ((2.75, 1.65), 'tri', (0, 100, 0)),
           ((10.0, 3.3), 'grid', (0, 0, 0))]
out = img.copy()
for (x, y), text, color in labels:
    u, v = px(x, y)
    cv2.circle(out, (u, v), 5, color, -1)
    cv2.putText(out, text, (u + 6, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 4, cv2.LINE_AA)
    cv2.putText(out, text, (u + 6, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
# 출발선(시뮬 월드에는 표시가 없다. scene/DXF 기준 위치만 표기)
cv2.line(out, px(9.495, 6.5), px(10.105, 6.5), (0, 0, 0), 2)
cv2.putText(out, 'v2026.11.06 raw world.sdf, Gazebo ogre2 top view (labels added; raw PNG unchanged)', (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2, cv2.LINE_AA)
cv2.putText(out, f'scale {mpp*1000:.2f} mm/px at z=0; C#L/R = corner turn, SP = save point (MANUAL)', (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)
target = R / 'annotated/overview_top_labeled.png'
cv2.imwrite(str(target), out)

info, links = sa.parse_sdf(sa.OF / 'world.sdf')
_, road = sa.link_union(links, 'track_surface', 'collision')
_, grass = sa.link_union(links, 'grass', 'collision')
rg = road.union(grass)
from shapely.geometry import Polygon
holes = sorted((Polygon(h).area for p in sa.poly_parts(road) for h in p.interiors), reverse=True)
# 경계 길이는 따로 병합한 도로·잔디 이음매의 수치 띠 때문에 신뢰할 수 없어 기록하지 않는다(벽 길이는 static_audit의 PDF 대조 사용).
nums = {'road_hole_areas_m2': [round(a, 4) for a in holes],
        'grass_only_triangle_area_m2_estimate': round(holes[-1], 4) if holes else None,
        'overview_mm_per_px': round(mpp * 1000, 3)}
(ART / 'checks/overview_numbers.json').write_text(json.dumps(nums, indent=1) + '\n', encoding='utf-8')
print(target, nums)
