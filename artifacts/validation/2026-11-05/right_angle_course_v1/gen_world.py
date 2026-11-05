#!/usr/bin/env python3
"""직각 코너 대표 가정 시험 월드 생성·형상 검사.

공식 v2026.11.01 팀 파생 월드(읽기 전용)에서 시설(신호·그리드·결승·ID0/45·바닥)을 그대로
가져오고, 노면/잔디/벽만 직각 코너 대표 가정 중심선으로 다시 만든다. 사진(11/5 주최 측 제안
도면)의 정확한 치수는 모르므로 재현이 아니다. 단면은 공식 생성기와 같은 규칙을 쓴다:
노면 0.45 m, 양옆 잔디 0.20 m, 벽 안쪽 면 0.425 m·두께 0.05 m·높이 0.40 m.

사용: python3 gen_world.py <공식 월드 디렉터리> <출력 디렉터리>
출력은 시험 전용이며 차량 제어 입력이 아니다(평가·실행 월드에만 쓰인다).
"""
import copy
import hashlib
import json
import math
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from shapely.geometry import LinearRing, LineString, Point, Polygon, box
from shapely.ops import unary_union
from shapely import affinity

ROAD_HALF = 0.225          # 공식 main_width_m 0.45 / 2
GRASS_W = 0.20             # 공식 grass.width_each_side_m
WALL_T = 0.05              # 공식 walls.thickness_m
WALL_H = 0.40              # 공식 월드 벽 상자 높이(0.4 m, z=0.2)
WALL_IN = ROAD_HALF + GRASS_W            # 0.425
WALL_OUT = WALL_IN + WALL_T              # 0.475
SPACING = 0.02

# 지역 좌표(u: 공식 출발 직선 진행 방향, v: 그 왼쪽). 원점은 공식 결승점.
# 진행 순서대로의 꼭짓점. 반시계(왼쪽 4회 순회) 폐곡선.
VERTICES = [
    ("V1", 6.5, 0.0),      # 긴 출발 직선 끝: 헤어핀 첫 왼쪽 90°
    ("V2", 6.5, 0.9),      # 헤어핀 둘째 왼쪽 90° (중심 간격 0.9 m = 벽 하나 공유)
    ("V3", 2.5, 0.9),      # 고립 오른쪽 90°
    ("V4", 2.5, 4.0),      # 고립 왼쪽 90°
    ("V5", 0.0, 4.0),      # 크랭크 오른쪽 90°
    ("V6", 0.0, 5.0),      # 크랭크 왼쪽 90° (1.0 m 뒤)
    ("V7", -4.4665, 5.0),  # 고립 왼쪽 90°
    ("V0", -4.4665, 0.0),  # 출발 직선 진입 왼쪽 90°
]
CORNER_ROLE = {"V1": "hairpin_left_1", "V2": "hairpin_left_2", "V3": "isolated_right",
               "V4": "isolated_left", "V5": "crank_right", "V6": "crank_left",
               "V7": "isolated_left", "V0": "isolated_left_before_start"}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fit_start_frame(official):
    c = np.genfromtxt(official / "centerline.csv", delimiter=",", names=True)
    # 그리드(약 y 4.4)부터 결승 뒤 0.5 m까지의 공식 중심선에 맞춘다(시설 정렬 우선).
    sel = (c["x_m"] > 9.5) & (c["y_m"] > 4.0) & (c["y_m"] < 7.0)
    pts = np.column_stack((c["x_m"][sel], c["y_m"][sel]))
    centered = pts - pts.mean(axis=0)
    direction = np.linalg.svd(centered, full_matrices=False)[2][0]
    if direction[1] < 0:
        direction = -direction
    theta = math.atan2(direction[1], direction[0])
    scene = json.loads((official / "scene.json").read_text(encoding="utf-8"))
    f = scene["start_finish"]["pose"]
    origin = np.array([f["x"], f["y"]])
    return origin, theta, pts


def to_global(origin, theta, u, v):
    h = np.array([math.cos(theta), math.sin(theta)])
    n = np.array([-math.sin(theta), math.cos(theta)])
    return origin + np.multiply.outer(u, h) + np.multiply.outer(v, n)


def to_local(origin, theta, xy):
    d = np.asarray(xy) - origin
    return np.stack((d[..., 0] * math.cos(theta) + d[..., 1] * math.sin(theta),
                     -d[..., 0] * math.sin(theta) + d[..., 1] * math.cos(theta)), axis=-1)


def rectangles(poly):
    """축 정렬 직교 다각형을 세로 띠로 잘라 직사각형 목록(u0,v0,u1,v1)으로 분해한다."""
    import shapely
    poly = shapely.set_precision(poly, 1e-7)
    us = sorted({round(x, 9) for g in getattr(poly, "geoms", [poly])
                 for ring in [g.exterior, *g.interiors] for x, _ in ring.coords})
    vmin, vmax = poly.bounds[1] - 1, poly.bounds[3] + 1
    out = []
    for u0, u1 in zip(us, us[1:]):
        if u1 - u0 < 1e-9:
            continue
        piece = poly.intersection(box(u0, vmin, u1, vmax))
        for g in getattr(piece, "geoms", [piece]):
            if g.is_empty or g.area < 1e-10 or g.geom_type != "Polygon":
                continue
            b = g.bounds
            if abs((b[2] - b[0]) * (b[3] - b[1]) - g.area) > 1e-8:
                raise ValueError(f"직교 분해 실패: {g.wkt[:300]}")
            out.append((round(b[0], 9), round(b[1], 9), round(b[2], 9), round(b[3], 9)))
    # 같은 v 구간의 인접 띠를 합친다.
    out.sort(key=lambda r: (r[1], r[3], r[0]))
    merged = []
    for r in out:
        if merged and merged[-1][1] == r[1] and merged[-1][3] == r[3] and abs(merged[-1][2] - r[0]) < 1e-9:
            merged[-1] = (merged[-1][0], r[1], r[2], r[3])
        else:
            merged.append(r)
    return merged


def box_link(name, prefix, rects, origin, theta, z, sz, rgba, friction=None):
    link = ET.Element("link", name=name)
    for i, (u0, v0, u1, v1) in enumerate(rects):
        cx, cy = to_global(origin, theta, (u0 + u1) / 2, (v0 + v1) / 2)
        pose = f"{cx:.6f} {cy:.6f} {z:.4f} 0 0 {theta:.6f}"
        size = f"{u1 - u0:.6f} {v1 - v0:.6f} {sz:.4f}"
        for kind in ("collision", "visual"):
            el = ET.SubElement(link, kind, name=f"{prefix}_{i}_{'col' if kind == 'collision' else 'vis'}")
            ET.SubElement(el, "pose").text = pose
            ET.SubElement(ET.SubElement(el, "geometry"), "box").append(ET.Element("size"))
            el.find("geometry/box/size").text = size
            if kind == "collision" and friction is not None:
                ode = ET.SubElement(ET.SubElement(ET.SubElement(el, "surface"), "friction"), "ode")
                ET.SubElement(ode, "mu").text = str(friction)
                ET.SubElement(ode, "mu2").text = str(friction)
            if kind == "visual":
                mat = ET.SubElement(el, "material")
                ET.SubElement(mat, "ambient").text = rgba
                ET.SubElement(mat, "diffuse").text = rgba
    return link


def sample_centerline(local_vertices):
    """결승점(0,0)에서 시작해 꼭짓점을 따라 0.02 m 간격 표본. 마지막 점(=시작)은 제외."""
    pts = [(0.0, 0.0)] + [(u, v) for _, u, v in local_vertices] + [(0.0, 0.0)]
    line = LineString(pts)
    total = line.length
    n = int(round(total / SPACING))
    s = np.linspace(0, total, n, endpoint=False)
    xy = np.array([line.interpolate(d).coords[0] for d in s])
    return xy, s, total, line


def main():
    official = Path(sys.argv[1]).resolve()
    out = Path(sys.argv[2]).resolve()
    if out.exists():
        raise SystemExit(f"출력 디렉터리가 이미 있다: {out}")
    out.mkdir(parents=True)
    origin, theta, official_pts = fit_start_frame(official)
    ring_local = LinearRing([(u, v) for _, u, v in VERTICES])
    if not ring_local.is_simple or not ring_local.is_ccw:
        raise ValueError("중심선 폐곡선이 단순·반시계가 아님")
    mitre = dict(join_style="mitre", mitre_limit=10.0)
    road = ring_local.buffer(ROAD_HALF, **mitre)
    grass_outer = ring_local.buffer(WALL_IN, **mitre)
    wall_outer = ring_local.buffer(WALL_OUT, **mitre)
    grass = grass_outer.difference(road)
    walls = wall_outer.difference(grass_outer)
    rr, gr, wr = rectangles(road), rectangles(grass), rectangles(walls)

    # ---- 월드 SDF: 공식 팀 파생본에서 노면/잔디/벽과 지름길 표지(ID20/30)만 교체 ----
    tree = ET.parse(official / "world.sdf")
    world = tree.getroot().find("world")
    model = world.find("model[@name='it_arena_track_static']")
    names = [l.get("name") for l in model.findall("link")]
    replaced = {"track_surface": box_link("track_surface", "surface_rect", rr, origin, theta, 0.0015, 0.003, "0.90 0.90 0.90 1"),
                "grass": box_link("grass", "grass_rect", gr, origin, theta, 0.0010, 0.002, "0.13 0.55 0.13 1"),
                "walls": box_link("walls", "walls_rect", wr, origin, theta, WALL_H / 2, WALL_H, "0.50 0.50 0.50 1", friction=0.8)}
    removed = []
    for i, link in enumerate(list(model)):
        if link.tag != "link":
            continue
        name = link.get("name")
        if name in replaced:
            idx = list(model).index(link)
            model.remove(link)
            model.insert(idx, replaced[name])
        elif name in ("aruco_20", "aruco_30"):
            model.remove(link)
            removed.append(name)
    tree.write(out / "world.sdf", encoding="utf-8", xml_declaration=True)
    shutil.copytree(official / "aruco", out / "aruco")

    # ---- 중심선 CSV(평가 전용: 진행 거리·랩 길이) ----
    xy_local, s, total, line_local = sample_centerline(VERTICES)
    xy = to_global(origin, theta, xy_local[:, 0], xy_local[:, 1])
    with open(out / "centerline.csv", "w", encoding="utf-8", newline="\n") as fh:
        fh.write("x_m,y_m,s_m,track_width_m\n")
        for (x, y), d in zip(xy, s):
            fh.write(f"{x:.5f},{y:.5f},{d:.5f},0.4500\n")

    def s_of(px, py):
        u, v = to_local(origin, theta, np.array([px, py]))
        return float(line_local.project(Point(u, v))) % total

    # ---- scene.json: 공식 시설 자세는 그대로, 경로 s만 새 중심선 기준으로 ----
    scene = json.loads((official / "scene.json").read_text(encoding="utf-8"))
    official_scene = copy.deepcopy(scene)
    scene["course_name"] = "TEAM TEST right-angle course v1 (representative assumption; not an official layout)"
    gv = to_global(origin, theta, np.array([u for _, u, _ in VERTICES]), np.array([v for _, _, v in VERTICES]))
    road_g = affinity.translate(affinity.rotate(road, theta, origin=(0, 0), use_radians=True), *origin)
    scene["track"].update(
        lap_length_m=round(total, 4),
        centerline_polyline=[[round(x, 4), round(y, 4)] for x, y in gv],
        left_boundary_polyline=[[round(x, 4), round(y, 4)] for x, y in road_g.interiors[0].coords],
        right_boundary_polyline=[[round(x, 4), round(y, 4)] for x, y in road_g.exterior.coords],
        min_centerline_radius_general_m=0.0, sectors=[], friction_zone=None,
        note="sharp 90-degree centerline vertices; mitre road/grass/wall offsets")
    scene["branches"] = []
    scene["alt_routes"] = {}
    for slot in scene["starting_grid"]["slots"]:
        slot["s_m"] = round(s_of(slot["x"], slot["y"]), 4)
    scene["starting_grid"]["grid_start_s_m"] = round(min(sl["s_m"] for sl in scene["starting_grid"]["slots"]) - 0.05, 4)
    tl = scene["traffic_light"]
    tl["s_m"] = round(s_of(tl["pose"]["x"], tl["pose"]["y"]), 4)
    scene["aruco_markers"]["markers"] = [m for m in scene["aruco_markers"]["markers"] if m["id"] in (0, 45)]
    for m in scene["aruco_markers"]["markers"]:
        m["s_m"] = tl["s_m"]
    scene["runtime_corrections"]["right_angle_test"] = {
        "status": "team_test_representative_assumption_not_official",
        "kept_official_runtime_facilities": ["2x5 signal links/beam/panel", "grid slot paints/poses", "finish checker",
                                             "ArUco ID0/ID45 gantry boards", "ground plane", "physics/plugins"],
        "removed": removed, "removed_reason": "ID20/ID30 belong to official shortcut gates; this test course has no branches",
        "replaced_links": sorted(replaced)}

    # ---- 형상 검사 ----
    checks = {}
    road_box_union = unary_union([box(*r) for r in rr])
    grass_box_union = unary_union([box(*r) for r in gr])
    wall_box_union = unary_union([box(*r) for r in wr])
    checks["box_decomposition"] = dict(
        road_boxes=len(rr), grass_boxes=len(gr), wall_boxes=len(wr),
        road_symdiff_m2=road_box_union.symmetric_difference(road).area,
        grass_symdiff_m2=grass_box_union.symmetric_difference(grass).area,
        wall_symdiff_m2=wall_box_union.symmetric_difference(walls).area)
    checks["overlap_m2"] = dict(road_grass=road.intersection(grass).area, road_wall=road.intersection(walls).area,
                                grass_wall=grass.intersection(walls).area,
                                road_box_pairs=sum(box(*a).intersection(box(*b)).area for i, a in enumerate(rr) for b in rr[i + 1:]),
                                grass_box_pairs=sum(box(*a).intersection(box(*b)).area for i, a in enumerate(gr) for b in gr[i + 1:]),
                                wall_box_pairs=sum(box(*a).intersection(box(*b)).area for i, a in enumerate(wr) for b in wr[i + 1:]))
    checks["coverage"] = dict(gap_inside_wall_outer_m2=wall_outer.difference(unary_union([road, grass, walls])).area,
                              road_area_m2=road.area, grass_area_m2=grass.area, wall_area_m2=walls.area)
    # 꼭짓점 회전각·직선 길이
    pts = [(0.0, 0.0)] + [(u, v) for _, u, v in VERTICES]
    corners = []
    for k, (name, u, v) in enumerate(VERTICES):
        prev = np.array(pts[k]) if k > 0 else np.array(pts[-1])
        prev = np.array([VERTICES[k - 1][1], VERTICES[k - 1][2]])
        nxt = np.array([VERTICES[(k + 1) % len(VERTICES)][1], VERTICES[(k + 1) % len(VERTICES)][2]])
        a, b = np.array([u, v]) - prev, nxt - np.array([u, v])
        turn = math.degrees(math.atan2(a[0] * b[1] - a[1] * b[0], a @ b))
        corners.append(dict(name=name, role=CORNER_ROLE[name], turn_deg=round(turn, 6),
                            s_m=round(float(line_local.project(Point(u, v))), 4),
                            straight_before_m=round(float(np.linalg.norm(a)), 4),
                            straight_after_m=round(float(np.linalg.norm(b)), 4),
                            xy=[round(float(c), 4) for c in to_global(origin, theta, u, v)]))
    checks["corners"] = corners
    checks["turn_sum_deg"] = sum(c["turn_deg"] for c in corners)
    # 직선 단면: 꼭짓점에서 0.5 m 이상 떨어진 표본에서 노면/잔디/벽 경계까지 거리
    sections = []
    for k, (name, u, v) in enumerate(VERTICES):
        prev = np.array([VERTICES[k - 1][1], VERTICES[k - 1][2]])
        cur = np.array([u, v])
        seg = cur - prev
        length = np.linalg.norm(seg)
        d = seg / length
        n = np.array([-d[1], d[0]])
        for t in np.arange(0.5, length - 0.5 + 1e-9, 0.25):
            p = prev + d * t
            row = {}
            for side, sign in (("left", 1), ("right", -1)):
                ray = [p + sign * n * r for r in np.arange(0.0, 0.60001, 0.0005)]
                lab = ["road" if road.contains(Point(q)) else "grass" if grass.contains(Point(q))
                       else "wall" if walls.contains(Point(q)) else "floor" for q in ray]
                def first(label, after=0):
                    for i in range(after, len(lab)):
                        if lab[i] == label:
                            return i
                    return None
                gi, wi = first("grass"), first("wall")
                fi = first("floor", wi or 0) if wi is not None else None
                other = first("road", gi) if gi is not None else None
                row[side] = dict(road_edge=None if gi is None else gi * 0.0005,
                                 wall_inner=None if wi is None else wi * 0.0005,
                                 wall_outer=None if fi is None else fi * 0.0005,
                                 next_road=None if other is None else other * 0.0005)
            sections.append(dict(segment_end=name, t=round(float(t), 3), **row))
    def stats(key, side):
        vals = [r[side][key] for r in sections if r[side][key] is not None]
        return dict(min=min(vals), max=max(vals), n=len(vals), missing=len(sections) - len(vals))
    checks["straight_sections"] = {f"{side}_{key}": stats(key, side) for side in ("left", "right")
                                   for key in ("road_edge", "wall_inner", "wall_outer")}
    checks["straight_sections"]["samples"] = len(sections)
    # 인접 차선 간격(헤어핀 공유 벽)
    segs = [LineString([pts[i], pts[i + 1]]) for i in range(len(pts) - 1)]
    segs = [LineString([(VERTICES[i - 1][1], VERTICES[i - 1][2]), (VERTICES[i][1], VERTICES[i][2])]) for i in range(len(VERTICES))]
    clear = []
    for i in range(len(segs)):
        for j in range(i + 2, len(segs)):
            if i == 0 and j == len(segs) - 1:
                continue
            clear.append((segs[i].distance(segs[j]), VERTICES[i][0], VERTICES[j][0]))
    clear.sort()
    checks["min_nonadjacent_centerline_clearance"] = [dict(m=round(c, 4), seg_end_a=a, seg_end_b=b) for c, a, b in clear[:3]]
    # 공식 출발 직선과의 일치(시설 정렬)
    loc = to_local(origin, theta, official_pts)
    near = (loc[:, 0] > -2.6) & (loc[:, 0] < 0.6)
    checks["official_start_straight_alignment"] = dict(
        heading_rad=theta, official_finish_yaw_rad=official_scene["start_finish"]["pose"]["yaw_rad"],
        official_centerline_lateral_offset_m_grid_to_finish=dict(min=float(loc[near, 1].min()), max=float(loc[near, 1].max())),
        official_centerline_lateral_offset_m_fit_window=dict(min=float(loc[:, 1].min()), max=float(loc[:, 1].max())))
    # 시설: 그리드 칸이 노면 안, 신호 빔이 벽 안쪽
    gslots = []
    for sl in scene["starting_grid"]["slots"]:
        u, v = to_local(origin, theta, np.array([sl["x"], sl["y"]]))
        rect = affinity.rotate(box(u - 0.125, v - 0.085, u + 0.125, v + 0.085), sl["yaw_rad"] - theta, origin=(u, v), use_radians=True)
        gslots.append(dict(index=sl["index"], u=float(u), v=float(v), inside_road=road.contains(rect), s_m=sl["s_m"]))
    checks["grid_slots"] = gslots
    tlu, tlv = to_local(origin, theta, np.array([tl["pose"]["x"], tl["pose"]["y"]]))
    checks["traffic_light"] = dict(u=float(tlu), v=float(tlv), s_m=tl["s_m"], beam_half_m=0.375,
                                   beam_inside_wall_inner=0.375 < WALL_IN)
    checks["markers_kept"] = [dict(id=m["id"], local=[float(x) for x in to_local(origin, theta, np.array([m["pose"]["x"], m["pose"]["y"]]))])
                              for m in scene["aruco_markers"]["markers"]]
    wall_g = affinity.translate(affinity.rotate(wall_outer, theta, origin=(0, 0), use_radians=True), *origin)
    b = wall_g.bounds
    hall = (scene["hall"]["width_m"], scene["hall"]["height_m"])
    checks["hall_fit"] = dict(bounds=b, hall=hall, margin=[b[0], b[1], hall[0] - b[2], hall[1] - b[3]],
                              margin_ge_0p5=min(b[0], b[1], hall[0] - b[2], hall[1] - b[3]) >= 0.5)
    # 평가기 진행 위치 비연속(>1 m) 위험: 노면+잔디 2 cm 격자의 최근접 s가 이웃끼리 1 m 이상 튀는지
    gx = np.arange(b[0] - origin[0], b[2] - origin[0], 0.02)
    jumps = 0
    xs = np.arange(wall_outer.bounds[0], wall_outer.bounds[2], 0.02)
    ys = np.arange(wall_outer.bounds[1], wall_outer.bounds[3], 0.02)
    drivable = grass_outer
    grid_s = np.full((len(xs), len(ys)), np.nan)
    from shapely import prepared
    pd = prepared.prep(drivable)
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            if pd.contains(Point(x, y)):
                d2 = (xy_local[:, 0] - x) ** 2 + (xy_local[:, 1] - y) ** 2
                grid_s[i, j] = s[int(np.argmin(d2))]
    def jump(a, c):
        dd = (a - c + total / 2) % total - total / 2
        return np.abs(dd) > 1.0
    with np.errstate(invalid="ignore"):
        jx = jump(grid_s[1:, :], grid_s[:-1, :]) & ~np.isnan(grid_s[1:, :]) & ~np.isnan(grid_s[:-1, :])
        jy = jump(grid_s[:, 1:], grid_s[:, :-1]) & ~np.isnan(grid_s[:, 1:]) & ~np.isnan(grid_s[:, :-1])
    locs = [to_global(origin, theta, xs[i], ys[j]).round(3).tolist() for i, j in zip(*np.nonzero(jx))][:10]
    checks["progress_jump_risk"] = dict(neighbor_pairs_over_1m=int(jx.sum() + jy.sum()), examples=locs,
                                        scope="2 cm grid inside wall inner face; validator raises if a pose step jumps >1 m")
    checks["lap_length_m"] = total
    checks["frame"] = dict(origin=origin.tolist(), theta_rad=theta)
    ok = (max(checks["box_decomposition"][k] for k in ("road_symdiff_m2", "grass_symdiff_m2", "wall_symdiff_m2")) < 1e-9
          and max(checks["overlap_m2"].values()) < 1e-9 and checks["coverage"]["gap_inside_wall_outer_m2"] < 1e-9
          and all(abs(abs(c["turn_deg"]) - 90) < 1e-6 for c in corners) and abs(checks["turn_sum_deg"] - 360) < 1e-6
          and all(g["inside_road"] for g in gslots) and checks["traffic_light"]["beam_inside_wall_inner"]
          and checks["hall_fit"]["margin_ge_0p5"])
    ss = checks["straight_sections"]
    ok &= all(abs(ss[f"{side}_road_edge"][k] - ROAD_HALF) <= 0.0006 and abs(ss[f"{side}_wall_inner"][k] - WALL_IN) <= 0.0006
              and abs(ss[f"{side}_wall_outer"][k] - WALL_OUT) <= 0.0006 and ss[f"{side}_road_edge"]["missing"] == 0
              for side in ("left", "right") for k in ("min", "max"))
    checks["all_geometry_checks_passed"] = bool(ok)
    scene["verification"] = dict(team_test_geometry_checks="geometry_checks.json", all_passed=bool(ok),
                                 lap_length_m=round(total, 4), note="official verification block replaced; not an official check")
    (out / "scene.json").write_text(json.dumps(scene, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (out / "geometry_checks.json").write_text(json.dumps(checks, ensure_ascii=False, indent=2, default=float) + "\n", encoding="utf-8")
    (out / "sections.json").write_text(json.dumps(sections, indent=1) + "\n", encoding="utf-8")
    prov = dict(kind="team_test_right_angle_course_v1", official=False,
                basis="2026-11-05 organizer proposal image (exact dimensions unknown): representative assumption, not a reproduction",
                source_world=str(official), source_sha256={p: sha(official / p) for p in ("world.sdf", "scene.json", "centerline.csv")},
                generator_sha256=sha(__file__), vertices_local_uv=VERTICES, cross_section_m=dict(
                    road_half=ROAD_HALF, grass=GRASS_W, wall_inner=WALL_IN, wall_thickness=WALL_T, wall_height=WALL_H),
                outputs={p: sha(out / p) for p in ("world.sdf", "scene.json", "centerline.csv", "geometry_checks.json")})
    (out / "provenance.json").write_text(json.dumps(prov, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    # ---- 미리보기 ----
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 9), dpi=130)
    for rects, color in ((wr, "0.45"), (gr, "#2f8f2f"), (rr, "0.85")):
        for r in rects:
            g = to_global(origin, theta, np.array([r[0], r[2], r[2], r[0]]), np.array([r[1], r[1], r[3], r[3]]))
            ax.fill(g[:, 0], g[:, 1], color=color, lw=0)
    ax.plot(xy[:, 0], xy[:, 1], "k--", lw=0.6)
    for c in corners:
        ax.annotate(f"{c['name']} {c['role']}", c["xy"], fontsize=6, color="navy")
    for sl in scene["starting_grid"]["slots"]:
        ax.plot(sl["x"], sl["y"], "ms", ms=3)
    ax.plot(tl["pose"]["x"], tl["pose"]["y"], "r^", ms=5)
    ax.plot(*origin, "bx")
    ax.set_xlim(0, hall[0]); ax.set_ylim(0, hall[1]); ax.set_aspect("equal")
    ax.set_title("TEAM TEST right-angle course v1 (assumption; not official)", fontsize=8)
    fig.tight_layout(); fig.savefig(out / "preview.png"); plt.close(fig)
    print(json.dumps(dict(all_geometry_checks_passed=bool(ok), lap_length_m=total, boxes=[len(rr), len(gr), len(wr)],
                          jump_pairs=checks["progress_jump_risk"]["neighbor_pairs_over_1m"]), ensure_ascii=False))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
