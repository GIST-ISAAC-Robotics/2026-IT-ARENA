#!/usr/bin/env python3
"""static_audit의 기하로 특정 좌표 창을 고배율로 그린다 (WSL: python3 -I tools/zoom_detail.py)."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import static_audit as sa
from shapely.geometry import box

info, links = sa.parse_sdf(sa.OF / 'world.sdf')
road_c, road = sa.link_union(links, 'track_surface', 'collision')
grass_c, grass = sa.link_union(links, 'grass', 'collision')
wall_c, walls = sa.link_union(links, 'walls', 'collision')
rg = road.union(grass)
views = {'detail_left_edge_P15': (2.05, 2.35, 2.35, 3.05), 'detail_P21_diag_edge': (3.38, 2.30, 3.52, 2.44),
         'detail_P21_top_junction': (2.70, 2.55, 3.10, 2.95)}
for name, (a0, b0, a1, b1) in views.items():
    c = sa.Canvas(a0, b0, a1, b1, 3000 if (a1 - a0) < 0.2 else 1500)
    win = box(a0, b0, a1, b1)
    c.fill(win.difference(rg).difference(walls), (40, 90, 160))  # 노출 홀 바닥(갈색)
    for g in grass_c:
        if g.intersects(win):
            c.fill(g, (60, 160, 60)); c.outline(g, (30, 110, 30), 1)
    for g in road_c:
        if g.intersects(win):
            c.fill(g, (230, 230, 230)); c.outline(g, (160, 160, 160), 1)
    for g in wall_c:
        if g.intersects(win):
            c.fill(g, (110, 110, 110)); c.outline(g, (60, 60, 60), 1)
    c.text((a0 + (a1 - a0) * 0.02, b1 - (b1 - b0) * 0.04), name + ' raw boxes; brown=exposed hall floor', (0, 0, 0), 0.6)
    c.save(sa.PLOTS / f'{name}.png')
    print(name, 'exposed floor m2', round(win.intersection(sa.unary_union([w for w in [win]])).difference(rg).difference(walls).area, 8))
