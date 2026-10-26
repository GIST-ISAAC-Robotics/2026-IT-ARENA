"""탐색용: 실행 월드의 도로·잔디·벽 합집합과 겹침/공백 확인(평가 전용)."""
import sys, json, math, hashlib
from pathlib import Path
ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT/'scripts'))
import xml.etree.ElementTree as ET
from shapely.ops import unary_union
from build_experimental_track import rectangle
W = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf'
m = ET.parse(W).find("./world/model[@name='it_arena_track_static']")
def union(name):
    l = m.find(f"link[@name='{name}']"); polys = []
    for c in l.findall('collision'):
        cp = list(map(float, c.findtext('pose', '0 0 0 0 0 0').split())); sx, sy, _ = map(float, c.findtext('geometry/box/size').split())
        polys.append(rectangle(cp[0], cp[1], cp[5], sx, sy))
    return unary_union(polys)
road, grass, walls = union('track_surface'), union('grass'), union('walls')
print('sha', hashlib.sha256(W.read_bytes()).hexdigest())
print('area road %.4f grass %.4f walls %.4f' % (road.area, grass.area, walls.area))
print('road&grass %.6f road&walls %.6f grass&walls %.6f' % (road.intersection(grass).area, road.intersection(walls).area, grass.intersection(walls).area))
for n, g in (('road', road), ('grass', grass), ('walls', walls)):
    gs = getattr(g, 'geoms', [g]); print(n, g.geom_type, len(gs), 'holes', sum(len(p.interiors) for p in gs))
# 벽 내부(도로+잔디 corridor) 중 도로·잔디 어느 것도 아닌 공백
inner = unary_union([road, grass]).buffer(0)
envelope = unary_union([road, grass, walls]).convex_hull
gaps = envelope.difference(unary_union([road, grass, walls]))
print('convex-hull minus surfaces area', gaps.area)
# 도로 경계에서 0.02 m 바깥 띠 중 잔디가 아닌 부분(벽 제외)
ring = road.buffer(.02).difference(road)
nongrass = ring.difference(grass).difference(walls)
print('2cm ring area %.5f non-grass-non-wall %.6f' % (ring.area, nongrass.area))
parts = [p for p in getattr(nongrass, 'geoms', [nongrass]) if p.area > 1e-6]
parts.sort(key=lambda p: -p.area)
for p in parts[:12]:
    c = p.centroid; print('  nongrass part area %.6f at (%.3f, %.3f)' % (p.area, c.x, c.y))
