"""탐색용: 본선 중심선 단면별 도로 끝·잔디·벽·기타 지면 거리(평가 전용 정답 기하)."""
import sys, json, math
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT/'scripts'))
import xml.etree.ElementTree as ET
from shapely.ops import unary_union
from shapely import prepared
from shapely.geometry import Point
from build_experimental_track import rectangle
WD = ROOT/'src/arena_gazebo/worlds/it_arena_official'
m = ET.parse(WD/'world.sdf').find("./world/model[@name='it_arena_track_static']")
def union(name):
    l = m.find(f"link[@name='{name}']"); polys = []
    for c in l.findall('collision'):
        cp = list(map(float, c.findtext('pose', '0 0 0 0 0 0').split())); sx, sy, _ = map(float, c.findtext('geometry/box/size').split())
        polys.append(rectangle(cp[0], cp[1], cp[5], sx, sy))
    return unary_union(polys)
road, grass, walls = union('track_surface'), union('grass'), union('walls')
P = {k: prepared.prep(v) for k, v in (('road', road), ('grass', grass), ('wall', walls))}
cl = np.genfromtxt(WD/'centerline.csv', delimiter=',', names=True)
xy = np.column_stack([cl['x_m'], cl['y_m']]); s = cl['s_m']
t = np.gradient(xy, axis=0); t /= np.linalg.norm(t, axis=1)[:, None]
n = np.column_stack([-t[:, 1], t[:, 0]])
steps = np.arange(0., .80, .002)
def classify(p):
    q = Point(p)
    if P['wall'].contains(q): return 'W'
    if P['road'].contains(q): return 'R'
    if P['grass'].contains(q): return 'G'
    return 'O'
rows = []
for i in range(0, len(xy), 5):
    row = {'s': float(s[i])}
    for side, sign in (('left', 1), ('right', -1)):
        labels = ''.join(classify(xy[i]+sign*d*n[i]) for d in steps)
        # 연속 구간 요약
        runs = []; start = 0
        for k in range(1, len(labels)+1):
            if k == len(labels) or labels[k] != labels[start]:
                runs.append((labels[start], round(steps[start], 3), round(steps[k-1]+.002, 3))); start = k
        row[side] = runs[:6]
    rows.append(row)
# 요약: 표준(R→G→(O<=4mm)→W) 여부와 비표준 구간
def summarize(runs):
    runs = [list(r) for r in runs]
    # 4 mm 이하 O는 표본 해상도/상자 이음 틈으로 합친다(값은 따로 남김)
    merged = []
    for r in runs:
        if r[0] == 'O' and r[2]-r[1] <= .004 and merged:
            merged[-1][2] = r[2]; continue
        if merged and merged[-1][0] == r[0]:
            merged[-1][2] = r[2]; continue
        merged.append(r)
    kinds = ''.join(r[0] for r in merged)
    edge = merged[0][2] if merged[0][0] == 'R' else 0.
    wall = next((r[1] for r in merged if r[0] == 'W'), None)
    grass = sum(r[2]-r[1] for r in merged if r[0] == 'G' and (wall is None or r[1] < wall))
    other = sum(r[2]-r[1] for r in merged if r[0] == 'O' and (wall is None or r[1] < wall))
    return kinds, edge, wall, grass, other
cats = {}
for r in rows:
    for side in ('left', 'right'):
        kinds, edge, wall, grass, other = summarize(r[side])
        std = kinds.startswith('RGW') and wall is not None and abs((wall-edge)-.20) <= .01 and other <= .004
        key = 'standard' if std else kinds[:4]
        c = cats.setdefault((side, key), {'n': 0, 's': [], 'edge': [], 'wall_minus_edge': [], 'other_before_wall': []})
        c['n'] += 1; c['s'].append(r['s']); c['edge'].append(edge)
        c['wall_minus_edge'].append(None if wall is None else round(wall-edge, 3)); c['other_before_wall'].append(round(other, 3))
out = {}
for (side, key), c in sorted(cats.items()):
    ss = c['s']; spans = []; a = ss[0]; b = ss[0]
    for v in ss[1:]:
        if v-b > .11: spans.append((round(a, 2), round(b, 2))); a = v
        b = v
    spans.append((round(a, 2), round(b, 2)))
    wm = [v for v in c['wall_minus_edge'] if v is not None]
    out[f'{side}:{key}'] = {'stations': c['n'], 's_spans': spans[:15],
        'edge_m_min_max': [min(c['edge']), max(c['edge'])],
        'wall_minus_edge_min_max': [min(wm), max(wm)] if wm else None,
        'other_before_wall_max_m': max(c['other_before_wall'])}
print(json.dumps(out, indent=1))
