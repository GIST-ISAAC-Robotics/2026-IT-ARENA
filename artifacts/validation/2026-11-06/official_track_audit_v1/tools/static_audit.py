#!/usr/bin/env python3
"""v2026.11.06 배포 원본 정적 감사 (WSL: python3 -I tools/static_audit.py).

배포 파일을 읽기만 하며 수정하지 않는다. 결과는 checks/static_audit.json과 checks/plots/*.png.
판정 범위: SDF 구조·재질·물성·URI, 도로/잔디/벽 기하, PDF 표·scene·CSV·map 정합, 그리드·신호·마커·
세이브포인트 위치, 15×20 cm 차체의 정적 기하 통과 가능성(동역학·제어 완주 아님).
"""
from __future__ import annotations

import csv
import hashlib
import io
import json
import math
import sys
import zipfile
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from shapely import affinity
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box
from shapely.ops import unary_union

ART = Path(__file__).resolve().parents[1]
ROOT = Path(__file__).resolve().parents[5]
SRC = ART / 'extracted'
OF = SRC / 'output_final'
OUT = ART / 'checks'
PLOTS = OUT / 'plots'
OLD_ZIP = ROOT / 'assets/track/official/v2026.11.01/it_arena_track_v2026.11.01.zip'
VEHICLE_YAML = ROOT / 'src/arena_description/config/vehicle.yaml'
PDF = json.loads((ART / 'tools/pdf_tables_transcribed.json').read_text(encoding='utf-8'))
MANUAL_SAVEPOINTS = [(1, 5.57, 9.80, 12.07), (2, 14.61, 7.97, 7.12), (3, 19.56, 6.14, 4.00),
                     (4, 24.50, 4.31, 5.11), (5, 31.17, 2.48, 2.05), (6, 38.54, 9.00, 1.20)]
EPS = 1e-6


def r(x, n=4):
    if isinstance(x, (list, tuple)):
        return [r(v, n) for v in x]
    if isinstance(x, dict):
        return {k: r(v, n) for k, v in x.items()}
    if isinstance(x, (float, np.floating)):
        return round(float(x), n)
    return x


def floats(text, default='0 0 0 0 0 0'):
    return [float(v) for v in (text or default).split()]


# ---------------------------------------------------------------- SDF
def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def compose(a, b):
    ra, rb = rpy_matrix(*a[3:]), rpy_matrix(*b[3:])
    p = np.array(a[:3]) + ra @ np.array(b[:3])
    m = ra @ rb
    pitch = -math.asin(max(-1, min(1, m[2, 0])))
    roll = math.atan2(m[2, 1], m[2, 2])
    yaw = math.atan2(m[1, 0], m[0, 0])
    return [*p.tolist(), roll, pitch, yaw]


def parse_sdf(path):
    root = ET.parse(path).getroot()
    world = root.find('world')
    info = {'sdf_version': root.attrib.get('version'), 'world_name': world.attrib.get('name'),
            'physics': {c.tag: c.text for c in world.find('physics')} if world.find('physics') is not None else None,
            'lights': [{'name': l.attrib.get('name'), 'type': l.attrib.get('type'),
                        'cast_shadows': l.findtext('cast_shadows'), 'pose': l.findtext('pose'),
                        'direction': l.findtext('direction'), 'diffuse': l.findtext('diffuse')}
                       for l in world.findall('light')],
            'models': [], 'plugins': len(root.findall('.//plugin')), 'includes': len(root.findall('.//include')),
            'scripts': len(root.findall('.//script')), 'uris': [e.text for e in root.iter() if e.tag in ('uri', 'albedo_map', 'normal_map', 'roughness_map', 'metalness_map', 'emissive_map', 'light_map')],
            'scene_element': world.find('scene') is not None, 'gravity': world.findtext('gravity'),
            'world_children': sorted({c.tag for c in world})}
    links = {}
    for model in world.findall('model'):
        mpose = floats(model.findtext('pose'))
        info['models'].append({'name': model.attrib['name'], 'static': model.findtext('static'),
                               'links': len(model.findall('link'))})
        for link in model.findall('link'):
            lpose = compose(mpose, floats(link.findtext('pose')))
            items = []
            for tag in ('collision', 'visual'):
                for el in link.findall(tag):
                    pose = compose(lpose, floats(el.findtext('pose')))
                    g = el.find('geometry')
                    gtype = g[0].tag
                    gdata = {}
                    if gtype == 'box':
                        gdata['size'] = floats(g.findtext('box/size'))
                    elif gtype == 'sphere':
                        gdata['radius'] = float(g.findtext('sphere/radius'))
                    elif gtype == 'plane':
                        gdata['size'] = floats(g.findtext('plane/size'), '0 0')
                        gdata['normal'] = floats(g.findtext('plane/normal'), '0 0 1')
                    elif gtype == 'cylinder':
                        gdata['radius'] = float(g.findtext('cylinder/radius'))
                        gdata['length'] = float(g.findtext('cylinder/length'))
                    mat = el.find('material')
                    material = None
                    if mat is not None:
                        material = {c.tag: (c.text or '').strip() for c in mat if c.tag != 'pbr'}
                        if mat.find('pbr') is not None:
                            material['albedo_map'] = mat.findtext('pbr/metal/albedo_map')
                    surf = el.find('surface')
                    friction = None
                    if surf is not None:
                        friction = {'mu': surf.findtext('friction/ode/mu'), 'mu2': surf.findtext('friction/ode/mu2')}
                    items.append({'tag': tag, 'name': el.attrib.get('name'), 'pose': pose, 'type': gtype,
                                  **gdata, 'material': material, 'friction': friction})
            links[link.attrib['name']] = {'pose': lpose, 'items': items, 'model': model.attrib['name']}
    return info, links


def box_footprint(item):
    x, y, z, roll, pitch, yaw = item['pose']
    sx, sy, sz = item['size']
    assert abs(roll) < 1e-9 and abs(pitch) < 1e-9, item['name']
    p = box(-sx / 2, -sy / 2, sx / 2, sy / 2)
    p = affinity.rotate(p, yaw, origin=(0, 0), use_radians=True)
    return affinity.translate(p, x, y)


def z_range(item):
    x, y, z, roll, pitch, yaw = item['pose']
    if item['type'] == 'box':
        R = rpy_matrix(roll, pitch, yaw)
        half = np.array(item['size']) / 2
        corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
        zs = (R @ corners.T)[2] + z
        return float(zs.min()), float(zs.max())
    if item['type'] == 'sphere':
        return z - item['radius'], z + item['radius']
    return z, z


MERGE_EPS = 1e-4  # 0.1 mm: 생성기의 4자리 반올림으로 생긴 상자 사이 미세 틈(최대 0.112 mm)을 위상 계산에서만 닫는다.


def merged(polys, eps=MERGE_EPS):
    return unary_union([p.buffer(eps, join_style=2) for p in polys]).buffer(-eps, join_style=2)


def link_union(links, prefix_link, tag):
    polys = [box_footprint(it) for it in links[prefix_link]['items'] if it['tag'] == tag and it['type'] == 'box']
    return polys, merged(polys)


def raw_parts(polys):
    return len(poly_parts(unary_union(polys)))


# ---------------------------------------------------------------- 공통 기하
def read_csv(path):
    with open(path, newline='', encoding='utf-8') as fh:
        rows = list(csv.DictReader(fh))
    return rows


def polyline_closest(line: LineString, p):
    s = line.project(Point(p))
    q = line.interpolate(s)
    return s, (q.x, q.y), Point(p).distance(q)


def heading_at(line: LineString, s, ds=0.02):
    L = line.length
    a = line.interpolate(max(0.0, s - ds))
    b = line.interpolate(min(L, s + ds))
    return math.atan2(b.y - a.y, b.x - a.x)


def poly_parts(g):
    if g.is_empty:
        return []
    if isinstance(g, Polygon):
        return [g]
    return [p for p in getattr(g, 'geoms', []) if isinstance(p, Polygon)]


def describe_parts(g, min_area=0.0, limit=30):
    out = []
    for p in sorted(poly_parts(g), key=lambda q: -q.area):
        if p.area < min_area:
            continue
        c = p.representative_point()
        minx, miny, maxx, maxy = p.bounds
        out.append({'area_m2': r(p.area, 6), 'point': r([c.x, c.y]), 'bbox': r([minx, miny, maxx, maxy])})
        if len(out) >= limit:
            break
    return out


def rect_poly(cx, cy, yaw, length, width):
    p = box(-length / 2, -width / 2, length / 2, width / 2)
    p = affinity.rotate(p, yaw, origin=(0, 0), use_radians=True)
    return affinity.translate(p, cx, cy)


def cross_width(region, p, heading, half=1.2):
    """p에서 진행 방향에 수직인 선분과 region의 교차 중 p를 포함하는 구간 길이."""
    nx, ny = -math.sin(heading), math.cos(heading)
    seg = LineString([(p[0] - half * nx, p[1] - half * ny), (p[0] + half * nx, p[1] + half * ny)])
    inter = seg.intersection(region)
    pieces = [inter] if isinstance(inter, LineString) else list(getattr(inter, 'geoms', []))
    for piece in pieces:
        if piece.distance(Point(p)) < 1e-7:
            return piece.length
    return 0.0


# ---------------------------------------------------------------- 차량 기하 경로 탐색
def vehicle_params():
    import yaml
    v = yaml.safe_load(VEHICLE_YAML.read_text(encoding='utf-8'))['vehicle']
    d = v['drivetrain']
    L, delta = float(d['wheelbase_m']), float(d['max_steering_angle_rad'])
    radius, ww = float(d['wheel_radius_m']), float(d['wheel_width_m'])
    cand = min(delta, math.atan2(ww, 2 * radius))
    half_x = max(radius, radius * math.cos(cand) + ww / 2 * math.sin(cand))
    cand = min(delta, math.atan2(2 * radius, ww))
    half_y = max(ww / 2, radius * math.sin(cand) + ww / 2 * math.cos(cand))
    body = (float(v['footprint']['length_m']), float(v['footprint']['width_m']))
    steered = (max(body[0], L + 2 * half_x), max(body[1], float(d['track_width_m']) + 2 * half_y))
    return {'wheelbase_m': L, 'max_steer_rad': delta, 'track_width_m': float(d['track_width_m']),
            'rmin_rear_axle_m': L / math.tan(delta), 'body_LxW_m': body, 'steered_envelope_LxW_m': steered,
            'source': 'src/arena_description/config/vehicle.yaml (임시 축거·조향값, 변경하지 않음)'}


def sweep_path(start, heading, pre, radius, turn, post, ds=0.01):
    """후축 중심 기준 직선-원호-직선 자세 열."""
    poses = []
    x, y, h = start
    n = max(1, int(pre / ds))
    for i in range(n + 1):
        t = pre * i / n
        poses.append((x + t * math.cos(h), y + t * math.sin(h), h))
    x, y = poses[-1][0], poses[-1][1]
    sgn = 1 if turn > 0 else -1
    cx, cy = x - sgn * radius * math.sin(h), y + sgn * radius * math.cos(h)
    m = max(2, int(abs(turn) * radius / ds))
    for i in range(1, m + 1):
        a = turn * i / m
        hh = h + a
        poses.append((cx + sgn * radius * math.sin(hh), cy - sgn * radius * math.cos(hh), hh))
    x, y, h = poses[-1]
    n = max(1, int(post / ds))
    for i in range(1, n + 1):
        t = post * i / n
        poses.append((x + t * math.cos(h), y + t * math.sin(h), h))
    return poses


def footprint_at(pose, length, width, wheelbase):
    # 후축 중심에서 차체 중심은 +wheelbase/2 (축거 중앙 기준 원점, 대칭 오버행 가정)
    x, y, h = pose
    cx, cy = x + wheelbase / 2 * math.cos(h), y + wheelbase / 2 * math.sin(h)
    return rect_poly(cx, cy, h, length, width)


def best_turn(allowed, entry_axis, exit_axis, veh, dims, exit_offsets, lateral_offsets, radii, pre=0.6, post=0.6):
    """entry_axis/exit_axis: ((x0,y0), heading). 진입 축에서 횡 offset e로 달리다 반경 R로 돌아
    출구 축(+offset e2)에 정렬되는 직선-원호-직선 경로 중 여유가 최대인 것을 찾는다."""
    (ex, ey), h0 = entry_axis
    (xx, xy), h1 = exit_axis
    turn = (h1 - h0 + math.pi) % (2 * math.pi) - math.pi
    sgn = 1 if turn > 0 else -1
    boundary = allowed.boundary
    best = None
    feasible = 0
    tried = 0
    nx0, ny0 = -math.sin(h0), math.cos(h0)
    nx1, ny1 = -math.sin(h1), math.cos(h1)
    for e in lateral_offsets:
        for e2 in exit_offsets:
            for R in radii:
                # 원호 끝점이 출구 축(+e2) 위에 오도록 진입 축 위 시작 위치 u를 푼다.
                # 시작점 S = E0 + u*d0 + e*n0; 원호 끝 = S + R*(sin|t|*d0 + sgn*(1-cos|t|)*n0) (진입 좌표계)
                d0 = (math.cos(h0), math.sin(h0))
                at = abs(turn)
                off = (R * math.sin(at) * d0[0] + sgn * R * (1 - math.cos(at)) * nx0,
                       R * math.sin(at) * d0[1] + sgn * R * (1 - math.cos(at)) * ny0)
                base = (ex + e * nx0 + off[0], ey + e * ny0 + off[1])
                target_pt = (xx + e2 * nx1, xy + e2 * ny1)
                # (base + u*d0 - target)·n1 = 0
                denom = d0[0] * nx1 + d0[1] * ny1
                if abs(denom) < 1e-9:
                    continue
                u = ((target_pt[0] - base[0]) * nx1 + (target_pt[1] - base[1]) * ny1) / denom
                start = (ex + e * nx0 + (u - pre) * d0[0], ey + e * ny0 + (u - pre) * d0[1], h0)
                poses = sweep_path(start, h0, pre, R, turn, post)
                tried += 1
                swept = unary_union([footprint_at(p, dims[0], dims[1], veh['wheelbase_m']) for p in poses[::2]])
                inside = allowed.contains(swept)
                margin = boundary.distance(swept) if inside else -swept.difference(allowed).area
                if inside:
                    feasible += 1
                if best is None or margin > best['margin_m']:
                    best = {'margin_m': margin, 'inside': inside, 'e_m': e, 'e2_m': e2, 'R_m': R,
                            'turn_start_xy': [start[0] + pre * d0[0], start[1] + pre * d0[1]]}
    return {'tried': tried, 'feasible': feasible, 'best': r(best, 4)}


# ---------------------------------------------------------------- 그림
class Canvas:
    def __init__(self, xmin, ymin, xmax, ymax, ppm):
        self.xmin, self.ymin, self.xmax, self.ymax, self.ppm = xmin, ymin, xmax, ymax, ppm
        self.w, self.h = int((xmax - xmin) * ppm), int((ymax - ymin) * ppm)
        self.img = np.full((self.h, self.w, 3), 255, np.uint8)

    def px(self, x, y):
        return int(round((x - self.xmin) * self.ppm)), int(round((self.ymax - y) * self.ppm))

    def fill(self, geom, color):
        for p in poly_parts(geom):
            # 외곽과 구멍을 함께 넘겨 구멍 안의 기존 그림을 지우지 않는다(짝-홀 채움).
            rings = [np.array([self.px(*c) for c in p.exterior.coords], np.int32)]
            rings += [np.array([self.px(*c) for c in hole.coords], np.int32) for hole in p.interiors]
            cv2.fillPoly(self.img, rings, color)

    def outline(self, geom, color, t=1):
        for p in poly_parts(geom):
            cv2.polylines(self.img, [np.array([self.px(*c) for c in p.exterior.coords], np.int32)], True, color, t)
            for hole in p.interiors:
                cv2.polylines(self.img, [np.array([self.px(*c) for c in hole.coords], np.int32)], True, color, t)

    def line(self, a, b, color, t=1):
        cv2.line(self.img, self.px(*a), self.px(*b), color, t)

    def text(self, xy, s, color=(0, 0, 0), scale=0.45):
        cv2.putText(self.img, s, self.px(*xy), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)

    def dot(self, xy, rad, color):
        cv2.circle(self.img, self.px(*xy), rad, color, -1)

    def save(self, path):
        PLOTS.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(path), self.img)


def main():
    OUT.mkdir(exist_ok=True)
    PLOTS.mkdir(parents=True, exist_ok=True)
    res = {'scope': '배포 원본 정적 감사. 시뮬레이션 주행·실물 측정 아님. 차량 경로는 정적 기하 가능성만.',
           'inputs': {}}
    for p in sorted(SRC.rglob('*')):
        if p.is_file():
            res['inputs'][p.relative_to(SRC).as_posix()] = hashlib.sha256(p.read_bytes()).hexdigest()
    info, links = parse_sdf(OF / 'world.sdf')
    res['sdf'] = info
    # ---- 링크 목록·충돌/시각·z·재질·마찰
    inv = {}
    for name, link in links.items():
        cols = [i for i in link['items'] if i['tag'] == 'collision']
        vis = [i for i in link['items'] if i['tag'] == 'visual']
        zc = [z_range(i) for i in cols]
        zv = [z_range(i) for i in vis]
        mats = sorted({json.dumps(i['material'], sort_keys=True) for i in vis if i['material']})
        fr = sorted({json.dumps(i['friction'], sort_keys=True) for i in cols if i['friction']})
        # collision/visual 쌍 형상 일치
        def key(i):
            return (r(i['pose'], 5), i['type'], r(i.get('size'), 5), r(i.get('radius'), 5))
        cset = sorted(json.dumps(key(i)) for i in cols)
        vset = sorted(json.dumps(key(i)) for i in vis)
        inv[name] = {'collisions': len(cols), 'visuals': len(vis),
                     'collision_z': r([min(z[0] for z in zc), max(z[1] for z in zc)], 5) if zc else None,
                     'visual_z': r([min(z[0] for z in zv), max(z[1] for z in zv)], 5) if zv else None,
                     'collision_equals_visual_geometry': cset == vset if cols else None,
                     'materials': [json.loads(m) for m in mats][:4], 'material_variants': len(mats),
                     'friction': [json.loads(f) for f in fr], 'collision_without_friction': sum(1 for i in cols if not i['friction'])}
        if name.startswith(('grid_slot', 'tl_', 'lamp_', 'aruco_')):
            inv[name]['pose'] = r(link['pose'], 5)
            inv[name]['items'] = [{k: r(v, 5) for k, v in i.items() if k in ('tag', 'type', 'pose', 'size', 'radius')} for i in link['items']]
    res['links'] = inv
    # URI 존재
    res['uri_check'] = [{'uri': u, 'exists_relative_to_world': (OF / u).is_file()} for u in info['uris']]
    # ---- 기하
    road_c, road = link_union(links, 'track_surface', 'collision')
    road_v_list, road_v = link_union(links, 'track_surface', 'visual')
    grass_c, grass = link_union(links, 'grass', 'collision')
    wall_c, walls = link_union(links, 'walls', 'collision')
    _, walls_v = link_union(links, 'walls', 'visual')
    # 인접 상자 간 원시 틈(병합 전)
    def max_neighbor_gap(polys):
        u = unary_union(polys)
        parts = poly_parts(u)
        if len(parts) < 2:
            return 0.0
        big = max(parts, key=lambda q: q.area)
        return max(min(p.distance(o) for o in parts if o is not p) for p in parts)
    geo = {'merge_eps_m_for_topology': MERGE_EPS,
           'raw_union_parts': {'road': raw_parts(road_c), 'grass': raw_parts(grass_c), 'walls': raw_parts(wall_c)},
           'raw_max_gap_between_fragments_m': {'road': r(max_neighbor_gap(road_c), 7), 'grass': r(max_neighbor_gap(grass_c), 7),
                                               'walls': r(max_neighbor_gap(wall_c), 7)},
           'road_boxes': len(road_c), 'grass_boxes': len(grass_c), 'wall_boxes': len(wall_c),
           'road_area_m2': r(road.area, 5), 'grass_area_m2': r(grass.area, 5), 'wall_footprint_area_m2': r(walls.area, 5),
           'road_parts': len(poly_parts(road)), 'grass_parts': len(poly_parts(grass)), 'wall_parts': len(poly_parts(walls)),
           'road_holes': sum(len(p.interiors) for p in poly_parts(road)),
           'road_visual_equals_collision_area_diff_m2': r(road.symmetric_difference(road_v).area, 8),
           'wall_visual_equals_collision_area_diff_m2': r(walls.symmetric_difference(walls_v).area, 8)}
    # 상자 크기 분포 (작은 계단 조각)
    sizes = {}
    for b in road_c:
        pass
    small_road = [i for i in links['track_surface']['items'] if i['tag'] == 'collision' and min(i['size'][:2]) <= 0.0051]
    small_grass = [i for i in links['grass']['items'] if i['tag'] == 'collision' and min(i['size'][:2]) <= 0.0051]
    geo['road_boxes_thin_5mm'] = len(small_road)
    geo['grass_boxes_thin_5mm'] = len(small_grass)
    geo['road_thin_boxes_bbox'] = r(list(unary_union([box_footprint(i) for i in small_road]).bounds)) if small_road else None
    geo['grass_thin_boxes_bbox'] = r(list(unary_union([box_footprint(i) for i in small_grass]).bounds)) if small_grass else None
    # 미세 틈: 0.5 mm 팽창 후 수축으로 닫히는 틈
    closed = road.buffer(0.0005, join_style=2).buffer(-0.0005, join_style=2)
    geo['road_micro_gap_area_m2'] = r(closed.difference(road).area, 8)
    geo['road_micro_gaps'] = describe_parts(closed.difference(road), 1e-9, 10)
    rg = road.union(grass)
    closed_rg = rg.buffer(0.0005, join_style=2).buffer(-0.0005, join_style=2)
    geo['road_grass_micro_gap_area_m2'] = r(closed_rg.difference(rg).area, 8)
    geo['road_grass_overlap_m2'] = r(road.intersection(grass).area, 6)
    geo['road_grass_overlap_parts'] = describe_parts(road.intersection(grass), 1e-6, 10)
    geo['road_wall_overlap_m2'] = r(road.intersection(walls).area, 8)
    geo['grass_wall_overlap_m2'] = r(grass.intersection(walls).area, 8)
    geo['wall_min_distance_to_road_m'] = r(road.distance(walls), 5)
    from shapely.ops import nearest_points
    pa, pb = nearest_points(road, walls)
    geo['wall_road_nearest_points'] = r([[pa.x, pa.y], [pb.x, pb.y]], 4)
    near = road.buffer(0.199, join_style=2).intersection(walls)
    geo['walls_within_0199_of_road_parts'] = describe_parts(near, 1e-8, 10)
    # 벽 안쪽 공간(외곽 벽 내부 - 섬) 중 도로·잔디로 덮이지 않은 곳 = 홀 바닥 노출
    wall_parts = poly_parts(walls)
    hull_holes = []
    for p in wall_parts:
        for hole in p.interiors:
            hull_holes.append(Polygon(hole))
    # 가장 큰 구멍을 가진 벽 다각형의 내부 = 주행 공간 외곽, 나머지 다각형은 섬 벽
    enclosure = None
    if hull_holes:
        enclosure = max(hull_holes, key=lambda q: q.area)
        islands = [Polygon(p.exterior) for p in wall_parts if Polygon(p.exterior).within(enclosure)]
        free_space = enclosure.difference(unary_union(islands)) if islands else enclosure
        uncovered_raw = free_space.difference(rg)
        # 0.5 mm 미만 두께의 수치 띠는 제외(opening)
        uncovered = uncovered_raw.buffer(-0.00025, join_style=2).buffer(0.00025, join_style=2)
        geo['uncovered_floor_raw_m2'] = r(uncovered_raw.area, 6)
        geo['enclosure_area_m2'] = r(enclosure.area, 5)
        geo['islands'] = [{'area_m2': r(i.area, 4), 'bbox': r(list(i.bounds))} for i in islands]
        geo['free_space_area_m2'] = r(free_space.area, 5)
        geo['uncovered_floor_inside_walls_m2'] = r(uncovered.area, 6)
        geo['uncovered_floor_parts'] = describe_parts(uncovered, 1e-6, 20)
        geo['road_grass_outside_free_space_m2'] = r(rg.difference(free_space).area, 6)
        geo['road_grass_outside_free_space_parts'] = describe_parts(rg.difference(free_space), 1e-6, 10)
    else:
        geo['enclosure_error'] = '벽 다각형에 내부 구멍이 없음(외곽 폐합 실패 가능)'
        free_space = None
    # 벽 폐합: 외곽 벽 다각형 개수 및 섬
    geo['wall_polygons'] = [{'area_m2': r(p.area, 5), 'holes': len(p.interiors), 'bbox': r(list(p.bounds))} for p in wall_parts]
    res['geometry'] = geo

    # ---- PDF 표 대조
    pdf = {}
    pieces = []
    for name, (x0, y0, w, h) in PDF['floor_pieces_left_bottom_m_size_mm'].items():
        pieces.append((name, box(x0, y0, x0 + w / 1000, y0 + h / 1000)))
    p21 = Polygon(PDF['P21_corners_m'])
    if not p21.is_valid:
        p21 = Polygon([PDF['P21_corners_m'][i] for i in (0, 1, 3, 2)])
    pieces.append(('P21', p21 if p21.is_valid else p21.buffer(0)))
    pdf_road = unary_union([g for _, g in pieces])
    overlaps = []
    for i in range(len(pieces)):
        for j in range(i + 1, len(pieces)):
            a = pieces[i][1].intersection(pieces[j][1]).area
            if a > 1e-9:
                overlaps.append([pieces[i][0], pieces[j][0], r(a, 6)])
    diff_pdf_sdf = pdf_road.symmetric_difference(road)
    pdf['pdf_floor_area_m2'] = r(pdf_road.area, 5)
    pdf['pdf_piece_overlaps'] = overlaps
    pdf['pdf_vs_sdf_road_symdiff_m2'] = r(diff_pdf_sdf.area, 6)
    pdf['pdf_minus_sdf_parts'] = describe_parts(pdf_road.difference(road), 1e-7, 10)
    pdf['sdf_minus_pdf_parts'] = describe_parts(road.difference(pdf_road), 1e-7, 10)
    pdf['pdf_vs_sdf_hausdorff_m'] = r(pdf_road.hausdorff_distance(road), 5)
    pdf['P21_area_m2'] = r(p21.area, 5)
    # 벽선: 주행면 쪽 면 → 바깥(도로 반대) 쪽 두께 0.05 띠로 만들어 SDF 벽과 대조
    wall_rows = PDF['walls_road_face_lines']
    total_mm = sum(w[2] for w in wall_rows)
    pdf['wall_count'] = len(wall_rows)
    pdf['wall_total_stated_lengths_m'] = r(total_mm / 1000, 3)
    geom_len = sum(math.dist(a, b) for _, _, _, a, b in wall_rows)
    pdf['wall_total_geometric_m'] = r(geom_len, 3)
    wall_checks = []
    pdf_wall_polys = []
    for wid, mat, mm, a, b in wall_rows:
        seg = LineString([a, b])
        mid = seg.interpolate(0.5, normalized=True)
        dx, dy = b[0] - a[0], b[1] - a[1]
        n = math.hypot(dx, dy)
        nx, ny = -dy / n, dx / n
        # 도로·잔디 쪽 판정: 법선 ±0.1 m 지점 중 road∪grass에 가까운 쪽
        pa = Point(mid.x + 0.1 * nx, mid.y + 0.1 * ny)
        pb = Point(mid.x - 0.1 * nx, mid.y - 0.1 * ny)
        road_side = 1 if rg.distance(pa) < rg.distance(pb) else -1
        off = -road_side * 0.05
        band = Polygon([a, b, (b[0] + off * nx, b[1] + off * ny), (a[0] + off * nx, a[1] + off * ny)])
        pdf_wall_polys.append(band)
        covered = band.intersection(walls).area / band.area
        # 주행면 선과 SDF 벽 경계의 거리(선 위 표본)
        samples = [seg.interpolate(t, normalized=True) for t in np.linspace(0.02, 0.98, 25)]
        dist_face = max(walls.boundary.distance(sp) for sp in samples)
        stated_vs_geom_mm = mm - n * 1000
        clearance_to_road = min(road.distance(sp) for sp in samples)
        wall_checks.append({'id': wid, 'material': mat, 'stated_mm': mm, 'geom_mm': r(n * 1000, 1),
                            'stated_minus_geom_mm': r(stated_vs_geom_mm, 1), 'sdf_cover_frac': r(covered, 4),
                            'face_line_to_sdf_boundary_max_m': r(dist_face, 5),
                            'face_line_min_dist_to_road_m': r(clearance_to_road, 4)})
    pdf['walls'] = wall_checks
    pdf_walls = unary_union(pdf_wall_polys)
    pdf['pdf_wall_bands_vs_sdf_walls_symdiff_m2'] = r(pdf_walls.symmetric_difference(walls).area, 6)
    pdf['sdf_walls_not_in_pdf_parts'] = describe_parts(walls.difference(pdf_walls.buffer(0.001, join_style=2)), 1e-6, 20)
    pdf['pdf_walls_not_in_sdf_parts'] = describe_parts(pdf_walls.difference(walls.buffer(0.001, join_style=2)), 1e-6, 20)
    pdf['walls_length_mismatch_over_5mm'] = [w for w in wall_checks if abs(w['stated_minus_geom_mm']) > 5]
    pdf['walls_poorly_covered'] = [w for w in wall_checks if w['sdf_cover_frac'] < 0.98 or w['face_line_to_sdf_boundary_max_m'] > 0.002]
    pdf['grey_m'] = r(sum(w[2] for w in wall_rows if w[1].startswith('G')) / 1000, 3)
    pdf['eva_m'] = r(sum(w[2] for w in wall_rows if w[1].startswith('E')) / 1000, 3)
    bufbox = rg.bounds
    pdf['road_grass_bbox_m'] = r(list(bufbox))
    pdf['stated_buffer_bbox_m'] = PDF['stated_totals']['buffer_outer_bbox_m']
    pdf['road_grass_area_m2'] = r(rg.area, 4)
    pdf['road_grass_outer_perimeter_m'] = r(sum(p.exterior.length for p in poly_parts(rg)), 3)
    pdf['stated_totals'] = PDF['stated_totals']
    res['pdf_compare'] = pdf

    # ---- 중심선·경계·분기 CSV, scene
    scene = json.loads((OF / 'scene.json').read_text(encoding='utf-8'))
    design = json.loads((SRC / 'design_final.json').read_text(encoding='utf-8'))
    cl_rows = read_csv(OF / 'centerline.csv')
    cl = np.array([[float(r_['x_m']), float(r_['y_m']), float(r_['s_m']), float(r_['track_width_m'])] for r_ in cl_rows])
    loop = design['centerline'] + [design['centerline'][0]]
    main = LineString(loop)
    csvc = {'centerline_points': len(cl), 'centerline_last_s': r(cl[-1, 2], 5), 'design_length_m': r(main.length, 5),
            'scene_lap_length_m': scene['track']['lap_length_m'], 'width_values': sorted(set(cl[:, 3].round(4).tolist())),
            'max_dist_to_design_polyline_m': r(max(main.distance(Point(p)) for p in cl[:, :2]), 7),
            'ds_min_max': r([float(np.diff(cl[:, 2]).min()), float(np.diff(cl[:, 2]).max())], 5),
            'closing_gap_m': r(float(np.hypot(*(cl[0, :2] - cl[-1, :2]))), 5)}
    # s 일치
    s_err = max(abs(main.project(Point(p[:2])) - p[2]) for p in cl[::10])
    csvc['s_vs_design_projection_max_err_m'] = r(s_err, 6)
    # 경계: 진행 방향 왼/오른쪽 및 오프셋
    for side in ('left', 'right'):
        rows = read_csv(OF / f'{side}_boundary.csv')
        pts = np.array([[float(q['x_m']), float(q['y_m'])] for q in rows])
        d = np.array([main.distance(Point(p)) for p in pts])
        signs = []
        for p in pts[::25]:
            s = main.project(Point(p))
            h = heading_at(main, s)
            q = main.interpolate(s)
            signs.append(np.sign(math.cos(h) * (p[1] - q.y) - math.sin(h) * (p[0] - q.x)))
        csvc[f'{side}_boundary'] = {'points': len(pts), 'dist_to_centerline_min_max_m': r([float(d.min()), float(d.max())], 5),
                                    'side_sign_majority': int(np.sign(sum(signs))),
                                    'on_road_edge_max_m': r(max(road.boundary.distance(Point(p)) for p in pts[::5]), 6)}
    # 분기
    br = []
    for i, b in enumerate(scene['branches']):
        rows = read_csv(OF / f'branch_{i}.csv')
        pts = np.array([[float(q['x_m']), float(q['y_m']), float(q['s_m']), float(q['width_m'])] for q in rows])
        line = LineString(pts[:, :2])
        s0 = main.project(Point(pts[0, :2]))
        s1 = main.project(Point(pts[-1, :2]))
        dsg = design['branches'][i]
        # 분기 판 도로폭 (본선 띠 밖 구간에서)
        widths = []
        for t in np.linspace(0.05, 0.95, 37):
            p = line.interpolate(t, normalized=True)
            hh = heading_at(line, line.project(p))
            in_main = main.distance(p) < 0.305 - 1e-6
            if not in_main:
                widths.append(cross_width(road, (p.x, p.y), hh))
        # 지름길 축 주변 잔디 폭·벽까지 거리
        wall_clear = min(walls.distance(line.interpolate(t, normalized=True)) for t in np.linspace(0, 1, 50))
        br.append({'name': b['name'], 'csv_points': len(pts), 'csv_len_m': r(line.length, 5), 'scene_length_m': b['length_m'],
                   'scene_s0_s1': [b['s0_m'], b['s1_m']], 'projected_s0_s1': r([s0, s1], 4),
                   'design_s0_s1': r([dsg['s0'], dsg['s1']], 4), 'width_values': sorted(set(pts[:, 3].round(4).tolist())),
                   'plate_cross_width_min_max_m': r([min(widths), max(widths)], 5) if widths else None,
                   'centerline_min_dist_to_wall_m': r(wall_clear, 4),
                   'centerline_inside_road_frac': r(sum(road.buffer(1e-6).contains(line.interpolate(t, normalized=True)) for t in np.linspace(0, 1, 101)) / 101, 4),
                   'main_path_length_replaced_m': r((s1 - s0) % main.length, 4)})
    csvc['branches'] = br
    # 본선 폭 표본: 코너·분기 입구 제외
    corners = [tuple(p) for p in design['centerline']]
    corner_s = sorted(main.project(Point(c)) for c in corners[1:])
    branch_pts = [Point(b['centerline_polyline'][0]) for b in scene['branches']] + [Point(b['centerline_polyline'][-1]) for b in scene['branches']]
    widths = []
    for s in np.arange(0.0, main.length, 0.05):
        p = main.interpolate(s)
        if min(abs(s - cs) for cs in corner_s) < 0.36:
            continue
        if min(p.distance(bp) for bp in branch_pts) < 0.5:
            continue
        widths.append((s, cross_width(road, (p.x, p.y), heading_at(main, s))))
    wv = np.array([w for _, w in widths])
    csvc['main_cross_width'] = {'samples': len(wv), 'min_m': r(float(wv.min()), 5), 'max_m': r(float(wv.max()), 5),
                                'not_0610_samples': [r([s, w], 4) for s, w in widths if abs(w - 0.61) > 1e-3][:20]}
    # 코너 정의
    corner_info = []
    for i in range(1, len(loop) - 1):
        a, c, b = loop[i - 1], loop[i], loop[i + 1]
        h_in = math.atan2(c[1] - a[1], c[0] - a[0])
        h_out = math.atan2(b[1] - c[1], b[0] - c[0])
        turn = (h_out - h_in + math.pi) % (2 * math.pi) - math.pi
        sq = box(c[0] - 0.305, c[1] - 0.305, c[0] + 0.305, c[1] + 0.305)
        corner_info.append({'index': len(corner_info) + 1, 'xy': c, 's_m': r(main.project(Point(c)), 3),
                            'turn_deg': r(math.degrees(turn), 2), 'direction': 'left' if turn > 0 else 'right',
                            'corner_square_covered_frac': r(sq.intersection(road).area / sq.area, 6)})
    # 마지막 코너(9.8,1.2): loop 끝 처리
    a, c, b = loop[-2], loop[0], loop[1]
    csvc['corners'] = corner_info
    res['centerline_csv'] = csvc

    # ---- map.png 대조
    import yaml
    mp = {}
    for name in ('map', 'map_with_grass'):
        meta = yaml.safe_load((OF / f'{name}.yaml').read_text(encoding='utf-8'))
        img = cv2.imread(str(OF / meta['image']), cv2.IMREAD_GRAYSCALE)
        H, W = img.shape
        res_m = meta['resolution']
        ox, oy = meta['origin'][:2]
        vals, counts = np.unique(img, return_counts=True)
        # 도로∪분기 래스터 (픽셀 중심)
        rr = np.zeros_like(img, np.uint8)
        ys, xs = np.mgrid[0:H, 0:W]
        wx = ox + (xs + 0.5) * res_m
        wy = oy + (H - ys - 0.5) * res_m
        canvas_road = np.zeros((H, W), np.uint8)
        for p in poly_parts(road):
            pts = np.array([[(x - ox) / res_m, H - (y - oy) / res_m] for x, y in p.exterior.coords], np.float32)
            cv2.fillPoly(canvas_road, [np.round(pts * 16).astype(np.int32)], 1, shift=4)
            for hole in p.interiors:  # 고리형 본선 내부(섬)는 도로가 아니다
                pts = np.array([[(x - ox) / res_m, H - (y - oy) / res_m] for x, y in hole.coords], np.float32)
                cv2.fillPoly(canvas_road, [np.round(pts * 16).astype(np.int32)], 0, shift=4)
        canvas_grass = np.zeros((H, W), np.uint8)
        for p in poly_parts(grass.difference(road)):
            pts = np.array([[(x - ox) / res_m, H - (y - oy) / res_m] for x, y in p.exterior.coords], np.float32)
            cv2.fillPoly(canvas_grass, [np.round(pts * 16).astype(np.int32)], 1, shift=4)
            for hole in p.interiors:
                pts = np.array([[(x - ox) / res_m, H - (y - oy) / res_m] for x, y in hole.coords], np.float32)
                cv2.fillPoly(canvas_grass, [np.round(pts * 16).astype(np.int32)], 0, shift=4)
        free = img >= 254
        inter = np.logical_and(free, canvas_road == 1).sum()
        union = np.logical_or(free, canvas_road == 1).sum()
        entry = {'size_px': [W, H], 'resolution': res_m, 'origin': meta['origin'], 'pixel_values': {int(v): int(c) for v, c in zip(vals, counts)},
                 'free_vs_sdf_road_iou': r(inter / union, 5), 'free_px': int(free.sum()), 'sdf_road_px': int((canvas_road == 1).sum()),
                 'covers_hall': [r(ox, 3), r(oy, 3), r(ox + W * res_m, 3), r(oy + H * res_m, 3)]}
        # 1픽셀 이동 대조와 불일치 픽셀이 경계 근처에만 있는지
        shifts = {}
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                sh = np.roll(np.roll(canvas_road == 1, dy, axis=0), dx, axis=1)
                ref = free if name == 'map_with_grass' else np.logical_or(canvas_road == 1, canvas_grass == 1)
                shifts[f'{dx},{dy}'] = r(np.logical_and(free, sh if name == 'map_with_grass' else np.roll(np.roll(ref, dy, axis=0), dx, axis=1)).sum() /
                                         np.logical_or(free, sh if name == 'map_with_grass' else np.roll(np.roll(ref, dy, axis=0), dx, axis=1)).sum(), 5)
        entry['iou_by_pixel_shift'] = shifts
        target = (canvas_road == 1) if name == 'map_with_grass' else np.logical_or(canvas_road == 1, canvas_grass == 1)
        entry['free_vs_sdf_road_or_roadgrass_iou'] = r(np.logical_and(free, target).sum() / np.logical_or(free, target).sum(), 5)
        mism = np.logical_xor(free, target)
        edge = cv2.dilate(cv2.morphologyEx(target.astype(np.uint8), cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8)), np.ones((3, 3), np.uint8)) > 0
        entry['mismatch_px'] = int(mism.sum())
        entry['mismatch_px_away_from_boundary'] = int(np.logical_and(mism, ~edge).sum())
        if name == 'map_with_grass':
            g150 = img == 150
            gi = np.logical_and(g150, canvas_grass == 1).sum()
            gu = np.logical_or(g150, canvas_grass == 1).sum()
            entry['grass150_vs_sdf_grass_iou'] = r(gi / gu, 5)
        mp[name] = entry
    res['maps'] = mp

    # ---- scene 대 SDF: 그리드·신호·마커·출발선
    fac = {}
    slots = scene['starting_grid']['slots']
    gs = []
    veh = vehicle_params()
    body_L, body_W = veh['body_LxW_m']
    car_polys = []
    for s_ in slots:
        link = links[f"grid_slot_{s_['index']}"]
        lp = link['pose']
        vis = link['items'][0]
        slot_poly = rect_poly(lp[0], lp[1], lp[5], vis['size'][0], vis['size'][1])
        car = rect_poly(s_['x'], s_['y'], s_['yaw_rad'], body_L, body_W)
        car_polys.append(car)
        sp, _, lat = polyline_closest(main, (s_['x'], s_['y']))
        front_edge = s_['y'] + vis['size'][0] / 2 if abs(s_['yaw_rad'] - math.pi / 2) < 1e-3 else None
        gs.append({'index': s_['index'], 'col': s_['col'], 'row': s_['row'], 'scene_xy_yaw': r([s_['x'], s_['y'], s_['yaw_rad']], 4),
                   'sdf_xy_z_yaw': r([lp[0], lp[1], lp[2], lp[5]], 4), 'scene_eq_sdf': abs(lp[0] - s_['x']) < 1e-4 and abs(lp[1] - s_['y']) < 1e-4 and abs(lp[5] - s_['yaw_rad']) < 1e-3,
                   'scene_s': s_['s_m'], 'projected_s': r(sp, 4), 'lateral_from_centerline_m': r(lat * (1 if s_['x'] > 9.8 else -1), 4),
                   'slot_inside_road': road.contains(slot_poly), 'slot_min_dist_to_road_edge_m': r(road.boundary.distance(slot_poly), 4),
                   'car_centered_inside_slot': slot_poly.contains(car), 'slot_front_edge_y': r(front_edge, 4),
                   'slot_has_collision': any(i['tag'] == 'collision' for i in link['items']),
                   'slot_top_z_above_road_top_mm': r((lp[2] + vis['size'][2] / 2 - 0.003) * 1000, 2)})
    fac['grid'] = gs
    ov = []
    for i in range(len(car_polys)):
        for j in range(i + 1, len(car_polys)):
            d = car_polys[i].distance(car_polys[j])
            ov.append([i, j, r(d, 4)])
    fac['grid_car_pair_min_gap_m'] = min(o[2] for o in ov)
    fac['grid_car_pairs_closest'] = sorted(ov, key=lambda o: o[2])[:4]
    front = max(gs, key=lambda g: g['slot_front_edge_y'])
    tl = scene['traffic_light']
    fac['front_slot'] = front['index']
    fac['signal_minus_front_slot_front_edge_m'] = r(tl['pose']['y'] - front['slot_front_edge_y'], 4)
    fac['start_line_minus_signal_m'] = r(scene['start_finish']['pose']['y'] - tl['pose']['y'], 4)
    fac['start_line_minus_front_slot_front_edge_m'] = r(scene['start_finish']['pose']['y'] - front['slot_front_edge_y'], 4)
    fac['start_finish_scene'] = scene['start_finish']['pose']
    fac['start_finish_on_centerline_s0'] = r(main.project(Point(scene['start_finish']['pose']['x'], scene['start_finish']['pose']['y'])), 5)
    fac['grid_col0_lateral_side'] = 'left(-x, 진행 +y 기준)'
    # 신호
    post = links['tl_post']['items'][0]
    post_poly = box_footprint(post)
    beam = links['tl_beam']['items'][0]
    beam_poly = box_footprint(beam)
    panel_pose = links['tl_panel']['pose']
    Rp = rpy_matrix(*panel_pose[3:])
    normal = (Rp @ np.array([1.0, 0, 0])).tolist()
    lamps = {n: r(links[n]['pose'][:3], 4) for n in links if n.startswith('lamp_')}
    lamp_mats = {n: links[n]['items'][0]['material'] for n in links if n.startswith('lamp_')}
    fac['signal'] = {'scene_pose': tl['pose'], 'scene_s_m': tl['s_m'], 'projected_s': r(main.project(Point(tl['pose']['x'], tl['pose']['y'])), 4),
                     'scene_note': tl.get('note'), 'post_xy': r(post['pose'][:2], 4), 'post_size': post['size'],
                     'post_z': r(z_range(post), 4), 'post_inside_road': post_poly.intersects(road),
                     'post_min_dist_to_road_m': r(post_poly.distance(road), 4), 'post_inside_grass': post_poly.within(grass.buffer(1e-6)),
                     'post_min_dist_to_wall_m': r(post_poly.distance(walls), 4), 'post_has_collision': post['tag'] == 'collision',
                     'beam_x_span': r([beam_poly.bounds[0], beam_poly.bounds[2]], 4), 'beam_z': r(z_range(beam), 4),
                     'beam_gap_to_walls_m': r(beam_poly.distance(walls), 4),
                     'panel_pose': r(panel_pose, 5), 'panel_normal_xyz': r(normal, 4), 'panel_z': r(z_range(links['tl_panel']['items'][0]), 4),
                     'lamp_centers': lamps, 'lamp_center_mean_z': r(float(np.mean([v[2] for v in lamps.values()])), 4),
                     'lamp_materials_static': sorted({json.dumps(m, sort_keys=True) for m in lamp_mats.values()}),
                     'lamp_has_collision': {n: any(i['tag'] == 'collision' for i in links[n]['items']) for n in lamps}}
    # 마커
    mk = []
    scene_markers = {m['id']: m for m in scene['aruco_markers']['markers']}
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters_create()
    params.minMarkerDistanceRate = 0.02
    for mid in (0, 20, 30, 45):
        link = links[f'aruco_{mid}']
        lp = link['pose']
        vis = [i for i in link['items'] if i['tag'] == 'visual'][0]
        face = (math.cos(lp[5]), math.sin(lp[5]))
        sm = scene_markers.get(mid, {})
        s_proj, q, lat = polyline_closest(main, lp[:2])
        h = heading_at(main, s_proj)
        # 접근 차량 방향(마커 직전 0.3 m의 진행 방향)과 판 법선 비교
        h_app = heading_at(main, (s_proj - 0.3) % main.length)
        facing_dot = -(face[0] * math.cos(h_app) + face[1] * math.sin(h_app))
        img = cv2.imread(str(OF / f'aruco/aruco_id{mid}.png'), cv2.IMREAD_GRAYSCALE)
        _, ids, _ = cv2.aruco.detectMarkers(img, dictionary, parameters=params)
        dark = np.where(img < 128)
        code_px = [int(dark[1].min()), int(dark[0].min()), int(dark[1].max()), int(dark[0].max())] if len(dark[0]) else None
        mk.append({'id': mid, 'sdf_pose': r(lp, 5), 'scene': sm, 'board_size': vis['size'], 'albedo': vis['material'].get('albedo_map'),
                   'diffuse': vis['material'].get('diffuse'), 'has_collision': any(i['tag'] == 'collision' for i in link['items']),
                   'center_z_above_road_top_m': r(lp[2] - 0.003, 4), 'bottom_z_m': r(lp[2] - 0.05, 4),
                   'projected_s': r(s_proj, 4), 'lateral_from_centerline_m': r(lat, 4),
                   'face_normal_xy': r(face, 4), 'approach_heading_deg': r(math.degrees(h_app), 2),
                   'face_toward_approach_dot': r(facing_dot, 4),
                   'png_size': list(img.shape), 'png_decoded_ids': [] if ids is None else ids.flatten().tolist(),
                   'png_code_bbox_px': code_px})
    fac['markers'] = mk
    # 0/45 와 패널 겹침(위에서 본 x 범위)
    p_half = 0.17 / 2
    fac['marker_vs_panel_x_gap_m'] = {str(m['id']): r(abs(m['sdf_pose'][0] - 9.8) - 0.05 - p_half, 4) for m in mk if m['id'] in (0, 45)}
    # 세이브포인트
    sv = []
    for n, s, x, y in MANUAL_SAVEPOINTS:
        q = main.interpolate(s)
        sv.append({'n': n, 'manual_s': s, 'manual_xy': [x, y], 'centerline_xy_at_s': r([q.x, q.y], 4),
                   'xy_error_m': r(math.dist((x, y), (q.x, q.y)), 4), 'inside_road': road.contains(Point(x, y)),
                   'dist_to_road_edge_m': r(road.boundary.distance(Point(x, y)), 4),
                   'nearest_corner_ds_m': r(min(abs(s - cs) for cs in corner_s), 3),
                   'in_branch_zone': [b['name'] for b in scene['branches'] if b['s0_m'] <= s <= b['s1_m']],
                   'dist_to_branch_centerline_m': r(min(LineString(b['centerline_polyline']).distance(Point(x, y)) for b in scene['branches']), 4)})
    fac['savepoints'] = sv
    fac['scene_has_savepoints'] = any('save' in k for k in scene)
    # LiDAR 스캔면(0.075 m) 높이를 지나는 충돌체(벽 제외)
    low = []
    for name, link in links.items():
        if name in ('walls', 'ground_plane_link', 'track_surface', 'grass'):
            continue
        for it in link['items']:
            if it['tag'] != 'collision':
                continue
            z0, z1 = z_range(it)
            if z0 <= 0.075 <= z1 or z0 <= 0.15 and z1 >= 0.0 and z0 < 0.2:
                low.append({'link': name, 'z': r([z0, z1], 4), 'xy': r(it['pose'][:2], 4)})
    fac['low_collision_objects_excluding_walls'] = low
    # 벽 높이·두께 분포
    wz = sorted({json.dumps(r([*z_range(i)], 4)) for i in links['walls']['items'] if i['tag'] == 'collision'})
    wt = sorted({r(min(i['size'][:2]), 4) for i in links['walls']['items'] if i['tag'] == 'collision'})
    fac['wall_z_variants'] = [json.loads(z) for z in wz]
    fac['wall_thickness_variants'] = wt
    fac['road_top_z'] = inv['track_surface']['collision_z'][1]
    fac['grass_top_z'] = inv['grass']['collision_z'][1]
    fac['road_minus_grass_step_mm'] = r((inv['track_surface']['collision_z'][1] - inv['grass']['collision_z'][1]) * 1000, 3)
    res['facilities'] = fac

    # ---- 구버전 대조
    old = {}
    with zipfile.ZipFile(OLD_ZIP) as z:
        oscene = json.loads(z.read('output_final/scene.json'))
        oworld = z.read('output_final/world.sdf')
    tmp = OUT / '_old_world.sdf'
    tmp.write_bytes(oworld)
    oinfo, olinks = parse_sdf(tmp)
    tmp.unlink()
    old['lap_length_m'] = [oscene['track']['lap_length_m'], scene['track']['lap_length_m']]
    old['width_m'] = [oscene['track']['width_m'], scene['track']['width_m']]
    old['branches'] = [[{'name': b['name'], 's0': b['s0_m'], 's1': b['s1_m'], 'len': b['length_m']} for b in oscene['branches']],
                       [{'name': b['name'], 's0': b['s0_m'], 's1': b['s1_m'], 'len': b['length_m']} for b in scene['branches']]]
    old['markers'] = [[{'id': m['id'], 's': m.get('s_m'), 'pose': m.get('pose')} for m in oscene['aruco_markers']['markers']],
                      [{'id': m['id'], 's': m.get('s_m'), 'pose': m.get('pose')} for m in scene['aruco_markers']['markers']]]
    old['grid_s'] = [[s_['s_m'] for s_ in oscene['starting_grid']['slots']], [s_['s_m'] for s_ in slots]]
    old['signal_s'] = [oscene['traffic_light'].get('s_m'), tl['s_m']]
    old['tl_post'] = [r(olinks['tl_post']['items'][0]['pose'][:3], 4), r(post['pose'][:3], 4)]
    old_road = unary_union([box_footprint(i) for i in olinks['track_surface']['items'] if i['tag'] == 'collision' and i['type'] == 'box'])
    opost = box_footprint(olinks['tl_post']['items'][0])
    old['tl_post_inside_old_road'] = opost.intersects(old_road)
    old['tl_post_min_dist_old_road_m'] = r(opost.distance(old_road), 4)
    old['road_top_z'] = [r(max(z_range(i)[1] for i in olinks['track_surface']['items'] if i['tag'] == 'collision'), 5), fac['road_top_z']]
    old['grass_top_z'] = [r(max(z_range(i)[1] for i in olinks['grass']['items'] if i['tag'] == 'collision'), 5), fac['grass_top_z']]
    old['wall_friction'] = [sorted({json.dumps(i['friction']) for i in olinks['walls']['items'] if i['tag'] == 'collision'}),
                            sorted({json.dumps(i['friction']) for i in links['walls']['items'] if i['tag'] == 'collision'})]
    old['link_counts'] = [len(olinks), len(links)]
    old['road_boxes'] = [sum(1 for i in olinks['track_surface']['items'] if i['tag'] == 'collision'), len(road_c)]
    old['lights'] = [oinfo['lights'], info['lights']]
    old['physics'] = [oinfo['physics'], info['physics']]
    old['scene_keys_added'] = sorted(set(scene) - set(oscene))
    old['scene_keys_removed'] = sorted(set(oscene) - set(scene))
    res['old_vs_new'] = old

    # ---- 차량 정적 기하 경로 가능성
    vp = {'vehicle': r(veh, 4), 'method': '후축 중심 직선-원호-직선(단일 회전) 전진 경로를 격자 탐색해 차체/조향 외곽 사각형의 휩쓸린 영역이 허용 영역 안에 드는지 확인. 동역학·미끄럼·제어 지연 없음.'}
    radii = [round(veh['rmin_rear_axle_m'] + 0.0, 4)] + [0.35, 0.45, 0.6, 0.8, 1.0, 1.2]
    offs = [round(v, 3) for v in np.arange(-0.2, 0.2001, 0.025)]
    allowed_sets = {'road_only': road, 'road_plus_grass': rg}
    cases = []
    # 일반 직각 코너: 코너 1(왼쪽), 코너 3(오른쪽)
    def axis(i_from, i_to, back=0.0):
        a, b = loop[i_from], loop[i_to]
        h = math.atan2(b[1] - a[1], b[0] - a[0])
        return h
    for ci in (1, 3, 5):
        c = loop[ci]
        h_in = axis(ci - 1, ci)
        h_out = axis(ci, ci + 1)
        entry = ((c[0] - 1.2 * math.cos(h_in), c[1] - 1.2 * math.sin(h_in)), h_in)
        exit_ = ((c[0], c[1]), h_out)
        local = box(c[0] - 1.6, c[1] - 1.6, c[0] + 1.6, c[1] + 1.6)
        for nm, allowed in allowed_sets.items():
            for dims_name, dims in (('body', veh['body_LxW_m']), ('steered_envelope', veh['steered_envelope_LxW_m'])):
                out = best_turn(allowed.intersection(local), entry, exit_, veh, dims, offs, offs, radii)
                cases.append({'case': f'corner{corner_info[ci - 1]["index"]}_{corner_info[ci - 1]["direction"]}', 'allowed': nm, 'dims': dims_name, **out})
    # 지름길 진입/진출
    sc = scene['branches']
    p22a, p22b = sc[0]['centerline_polyline'][0], sc[0]['centerline_polyline'][-1]
    p21a, p21b = sc[1]['centerline_polyline'][0], sc[1]['centerline_polyline'][-1]
    h22 = math.atan2(p22b[1] - p22a[1], p22b[0] - p22a[0])
    h21 = math.atan2(p21b[1] - p21a[1], p21b[0] - p21a[0])
    sh = [0.0]
    branch_cases = [
        ('P22_entry', ((6.14, 6.0), -math.pi / 2), (tuple(p22a), h22), box(4.9, 3.6, 7.0, 6.2)),
        ('P22_exit', (tuple(p22a), h22), ((4.31, 4.5), math.pi / 2), box(3.4, 3.4, 5.4, 6.0)),
        ('P21_entry', ((2.48, 4.6), -math.pi / 2), (tuple(p21a), h21), box(1.9, 1.3, 4.0, 4.7)),
        ('P21_exit', (tuple(p21a), h21), ((4.185, 1.2), 0.0), box(2.0, 0.7, 6.0, 3.2)),
    ]
    for nm_case, entry, exit_, local in branch_cases:
        for nm, allowed in allowed_sets.items():
            for dims_name, dims in (('body', veh['body_LxW_m']), ('steered_envelope', veh['steered_envelope_LxW_m'])):
                ex_offs = sh if nm_case.endswith('entry') else offs
                en_offs = offs if nm_case.endswith('entry') else sh
                out = best_turn(allowed.intersection(local), entry, exit_, veh, dims, ex_offs, en_offs, radii, pre=0.5, post=0.5)
                cases.append({'case': nm_case, 'allowed': nm, 'dims': dims_name, **out})
    vp['cases'] = cases
    # 지름길 직선부 횡 여유
    vp['shortcut_lateral_clearance_each_side_m'] = {'body_0.15': r((0.20 - veh['body_LxW_m'][1]) / 2, 4),
                                                   'steered_envelope': r((0.20 - veh['steered_envelope_LxW_m'][1]) / 2, 4)}
    res['vehicle_geometry'] = vp

    # ---- 그림
    xmin, ymin, xmax, ymax = 1.5, 0.3, 10.8, 14.2
    cv = Canvas(xmin, ymin, xmax, ymax, 80)
    cv.fill(grass, (60, 160, 60))
    cv.fill(road, (230, 230, 230))
    cv.fill(walls, (110, 110, 110))
    for wid, mat, mm, a, b in PDF['walls_road_face_lines']:
        cv.line(a, b, (0, 0, 255) if mat.startswith('E') else (255, 0, 0), 1)
    for g in gs:
        sl = rect_poly(g['sdf_xy_z_yaw'][0], g['sdf_xy_z_yaw'][1], g['sdf_xy_z_yaw'][3], 0.25, 0.17)
        cv.fill(sl, (20, 20, 20))
    cv.fill(post_poly, (0, 0, 200))
    cv.outline(beam_poly, (0, 0, 200), 1)
    for m in mk:
        x, y, yaw = m['sdf_pose'][0], m['sdf_pose'][1], m['sdf_pose'][5]
        cv.dot((x, y), 4, (200, 0, 200))
        cv.line((x, y), (x + 0.4 * math.cos(yaw), y + 0.4 * math.sin(yaw)), (200, 0, 200), 2)
        cv.text((x + 0.08, y + 0.1), f"ID{m['id']}", (150, 0, 150))
    for s in sv:
        cv.dot(tuple(s['manual_xy']), 6, (0, 0, 220))
        cv.text((s['manual_xy'][0] + 0.1, s['manual_xy'][1] - 0.15), f"SP{s['n']}", (0, 0, 180))
    cv.line((9.495, 6.5), (10.105, 6.5), (0, 0, 0), 2)
    for c in corner_info:
        cv.text((c['xy'][0] + 0.05, c['xy'][1] + 0.05), f"C{c['index']}", (0, 90, 160), 0.4)
    for b in sc:
        cv.line(b['centerline_polyline'][0], b['centerline_polyline'][-1], (0, 140, 255), 1)
    cv.text((2.0, 13.9), 'v2026.11.06 world.sdf static: road(white) grass(green) wall(grey) / PDF wall lines G=blue E=red', (0, 0, 0), 0.5)
    cv.save(PLOTS / 'plan_overview.png')
    # 확대: P22, P21, 신호/그리드
    zooms = {'zoom_P22': (4.0, 2.4, 6.9, 5.2), 'zoom_P21': (1.8, 0.6, 4.6, 3.4), 'zoom_start_grid': (9.0, 3.0, 10.6, 7.0),
             'zoom_corner_C1': (9.0, 12.4, 10.5, 13.9)}
    for nm, (a0, b0, a1, b1) in zooms.items():
        z = Canvas(a0, b0, a1, b1, 300)
        z.fill(grass, (60, 160, 60))
        z.fill(road, (230, 230, 230))
        z.outline(road, (150, 150, 150), 1)
        z.fill(walls, (110, 110, 110))
        z.outline(MultiPolygon(road_c) if False else unary_union([]), (0, 0, 0))
        for wid, mat, mm, a, b in PDF['walls_road_face_lines']:
            z.line(a, b, (0, 0, 255) if mat.startswith('E') else (255, 0, 0), 1)
        if geo.get('uncovered_floor_parts'):
            z.fill(free_space.difference(rg), (0, 120, 255))
        for g in gs:
            sl = rect_poly(g['sdf_xy_z_yaw'][0], g['sdf_xy_z_yaw'][1], g['sdf_xy_z_yaw'][3], 0.25, 0.17)
            z.fill(sl, (20, 20, 20))
            car = rect_poly(g['scene_xy_yaw'][0], g['scene_xy_yaw'][1], g['scene_xy_yaw'][2], body_L, body_W)
            z.outline(car, (0, 200, 255), 1)
        z.fill(post_poly, (0, 0, 200))
        z.outline(beam_poly, (0, 0, 200), 1)
        for m in mk:
            x, y, yaw = m['sdf_pose'][0], m['sdf_pose'][1], m['sdf_pose'][5]
            mp_ = rect_poly(x, y, yaw, 0.005, 0.10)
            z.fill(mp_, (200, 0, 200))
            z.line((x, y), (x + 0.25 * math.cos(yaw), y + 0.25 * math.sin(yaw)), (200, 0, 200), 2)
            z.text((x + 0.03, y + 0.05), f"ID{m['id']}", (150, 0, 150), 0.6)
        for s in sv:
            z.dot(tuple(s['manual_xy']), 8, (0, 0, 220))
            z.text((s['manual_xy'][0] + 0.05, s['manual_xy'][1] - 0.08), f"SP{s['n']}", (0, 0, 180), 0.6)
        for b in sc:
            z.line(b['centerline_polyline'][0], b['centerline_polyline'][-1], (0, 140, 255), 1)
        z.line((9.495, 6.5), (10.105, 6.5), (0, 0, 0), 2)
        z.text((a0 + 0.05, b1 - 0.08), nm + ' (blue/red lines = PDF wall faces, orange = uncovered floor)', (0, 0, 0), 0.5)
        z.save(PLOTS / f'{nm}.png')
    (OUT / 'static_audit.json').write_text(json.dumps(res, ensure_ascii=False, indent=1, default=str) + '\n', encoding='utf-8', newline='\n')
    print('written', OUT / 'static_audit.json')


if __name__ == '__main__':
    main()
