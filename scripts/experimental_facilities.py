"""실험본에만 적용하는 도로 시설 형상·메타데이터. 원본 생성기는 수정하지 않습니다."""

from __future__ import annotations

import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np


ROAD_TOP_M = .003
PAINT_CENTER_M = .0038
PAINT_THICKNESS_M = .0004
WHITE = "0.96 0.96 0.94 1"
BLACK = "0.008 0.010 0.014 1"
YELLOW = "0.98 0.70 0.025 1"
STEEL = "0.22 0.26 0.30 1"


def value(parent, tag, text):
    child = ET.SubElement(parent, tag)
    child.text = str(text)
    return child


def number_list(values):
    return " ".join(f"{float(item):.10f}" for item in values)


def pose(parent, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    value(parent, "pose", number_list((*xyz, *rpy)))


def material(visual, rgba, emissive=None):
    mat = ET.SubElement(visual, "material")
    value(mat, "ambient", rgba)
    value(mat, "diffuse", rgba)
    value(mat, "specular", "0 0 0 1")
    if emissive is not None:
        value(mat, "emissive", emissive)


def box(link, name, xyz, size, rgba, *, rpy=(0, 0, 0), collision=False):
    visual = ET.SubElement(link, "visual", name=name)
    pose(visual, xyz, rpy)
    geometry = ET.SubElement(visual, "geometry")
    value(ET.SubElement(geometry, "box"), "size", number_list(size))
    material(visual, rgba)
    value(visual, "cast_shadows", "false")
    if collision:
        physical = ET.SubElement(link, "collision", name=name + "_collision")
        pose(physical, xyz, rpy)
        physical.append(copy.deepcopy(geometry))
    return visual


def link_at(model, name, x, y, z=0, yaw=0):
    link = ET.SubElement(model, "link", name=name)
    pose(link, (x, y, z), (0, 0, yaw))
    return link


def local_to_world(x, y, yaw, forward, left):
    c, s = math.cos(yaw), math.sin(yaw)
    return x + forward * c - left * s, y + forward * s + left * c


def configure(res, profile, generator):
    """도로·출발 위치·ID·s는 보존하고 별도 시설 설정을 적용합니다."""
    config = copy.deepcopy(profile["facilities"])
    for group in config.values():
        for key, item in group.items():
            if key.endswith("_m") and (not math.isfinite(float(item)) or float(item) <= 0):
                raise ValueError(f"시설 치수는 유한한 양수여야 합니다: {key}")
    bump_config = config["speed_bump"]
    if bump_config["profile"] != "raised_cosine" or not 8 <= bump_config["subdivisions"] <= 200:
        raise ValueError("방지턱은 8~200 분할의 raised_cosine 형상만 지원합니다.")
    if not 2 <= bump_config["lateral_stripes"] <= 32:
        raise ValueError("방지턱 색 띠는 2~32개여야 합니다.")
    if bump_config["height_m"] >= .02 or bump_config["length_m"] <= 4 * bump_config["height_m"]:
        raise ValueError("이 기초 차량용 방지턱은 2 cm 미만 높이와 충분한 진입 길이가 필요합니다.")
    if config["traffic_light"]["initial_state"] != "red":
        raise ValueError("아직 동적 신호 제어가 없으므로 초기 상태는 red만 지원합니다.")
    if not math.isclose(config["markers"]["black_square_size_m"], .10):
        raise ValueError("현재 실험에서는 ArUco 검은 정사각형 크기 10 cm를 유지합니다.")
    if len(res["grid_slots"]) != 6:
        raise ValueError("이번 실험 시설은 보존된 6개 출발 위치를 요구합니다.")

    for bump in res["bumps"]:
        bump.update(length=bump_config["length_m"], height=bump_config["height_m"])
    res["meta"]["bump_height"] = bump_config["height_m"]
    light = res["traffic_light"]
    light["gantry_height"] = config["traffic_light"]["gantry_height_m"]
    light["lamp_center_height"] = config["traffic_light"]["lamp_center_height_m"]
    light["initial_state"] = "red"
    markers = config["markers"]
    for marker in res["markers"]:
        target_x, target_y, _ = generator.sample_at_s(
            res["arr"], (marker["s"] - markers["face_target_distance_m"]) % res["meta"]["Ltot"],
            res["meta"]["Ltot"])
        marker["yaw"] = math.atan2(target_y - marker["y"], target_x - marker["x"])
        marker["center_height"] = markers["center_height_m"]
        marker["z"] = marker["center_height"] - markers["black_square_size_m"] / 2
        marker["approach_target"] = [float(target_x), float(target_y)]
    res["experimental_facilities"] = config
    return config


def bump_mesh(length, width, height, divisions, y0=None, y1=None):
    """노면 위 높이가 양 끝에서 기울기 0인 코사인 융기. 바닥은 1 mm 묻습니다."""
    y0 = -width / 2 if y0 is None else y0
    y1 = width / 2 if y1 is None else y1
    xs = np.linspace(-length / 2, length / 2, divisions + 1)
    heights = height * .5 * (1 + np.cos(2 * np.pi * xs / length))
    vertices = []
    for x, z in zip(xs, heights):
        vertices.extend([(x, y0, z), (x, y1, z), (x, y0, -.001), (x, y1, -.001)])
    faces = []
    for index in range(divisions):
        a, b = index * 4, (index + 1) * 4
        faces.extend([(a, b, b + 1), (a, b + 1, a + 1),  # 윗면 법선: +Z
                      (a + 2, a + 3, b + 3), (a + 2, b + 3, b + 2),
                      (a + 2, b + 2, b), (a + 2, b, a),
                      (a + 1, b + 1, b + 3), (a + 1, b + 3, a + 3)])
    end = divisions * 4
    faces.extend([(0, 1, 3), (0, 3, 2),
                  (end, end + 2, end + 3), (end, end + 3, end + 1)])
    return np.asarray(vertices, dtype=float), faces


def write_obj(path, vertices, faces, material_name="surface"):
    lines = ["# Generated experimental raised-cosine road bump; metres.",
             "mtllib bump_surface.mtl", "o curved_bump", "usemtl " + material_name]
    lines += ["v " + number_list(vertex) for vertex in vertices]
    for a, b, c in faces:
        normal = np.cross(vertices[b] - vertices[a], vertices[c] - vertices[a])
        normal /= np.linalg.norm(normal)
        lines.append("vn " + number_list(normal))
    lines += ["f " + " ".join(f"{index + 1}//{normal_index + 1}" for index in face)
              for normal_index, face in enumerate(faces)]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def add_bumps(model, res, output):
    config = res["experimental_facilities"]["speed_bump"]
    mesh_dir = output / "meshes"
    mesh_dir.mkdir(exist_ok=True)
    # OBJ의 내장 재질을 사용하는 렌더러에서도 SDF와 같은 색이 나오게 합니다.
    materials = []
    for name, rgba in (("surface", WHITE), ("yellow", YELLOW), ("black", BLACK)):
        rgb = " ".join(rgba.split()[:3])
        materials.append(f"newmtl {name}\nKa {rgb}\nKd {rgb}\nKs 0 0 0\nd 1\nillum 1\n")
    (mesh_dir / "bump_surface.mtl").write_text("\n".join(materials), encoding="utf-8")
    for index, bump in enumerate(res["bumps"]):
        link = link_at(model, f"safety_bump_{index}", bump["x"], bump["y"], ROAD_TOP_M, bump["yaw"])
        filename = f"bump_{index}_profile.obj"
        write_obj(mesh_dir / filename, *bump_mesh(bump["length"], bump["width"], bump["height"], config["subdivisions"]))
        xs = np.linspace(-bump["length"] / 2, bump["length"] / 2, config["subdivisions"] + 1)
        heights = bump["height"] * .5 * (1 + np.cos(2 * np.pi * xs / bump["length"]))
        thickness = bump["height"] + .002
        for segment, (x0, x1, z0, z1) in enumerate(zip(xs, xs[1:], heights, heights[1:])):
            angle = math.atan2(z1 - z0, x1 - x0)
            collision = ET.SubElement(link, "collision", name=f"curved_strip_{segment}")
            # 각 상자의 윗면 두 끝을 코사인 표본에 맞춥니다. 하부는 노면에 묻습니다.
            pose(collision, ((x0 + x1) / 2 + math.sin(angle) * thickness / 2, 0,
                             (z0 + z1) / 2 - math.cos(angle) * thickness / 2), (0, -angle, 0))
            value(ET.SubElement(ET.SubElement(collision, "geometry"), "box"), "size",
                  number_list((math.hypot(x1 - x0, z1 - z0), bump["width"], thickness)))
            friction = ET.SubElement(ET.SubElement(ET.SubElement(collision, "surface"), "friction"), "ode")
            value(friction, "mu", ".8")
            value(friction, "mu2", ".8")
        for stripe in range(config["lateral_stripes"]):
            y0 = -bump["width"] / 2 + bump["width"] * stripe / config["lateral_stripes"]
            y1 = -bump["width"] / 2 + bump["width"] * (stripe + 1) / config["lateral_stripes"]
            filename = f"bump_{index}_stripe_{stripe}.obj"
            write_obj(mesh_dir / filename, *bump_mesh(bump["length"], bump["width"], bump["height"],
                                                     config["subdivisions"], y0, y1),
                      material_name="yellow" if stripe % 2 == 0 else "black")
            visual = ET.SubElement(link, "visual", name=f"stripe_{stripe}")
            value(ET.SubElement(ET.SubElement(visual, "geometry"), "mesh"), "uri", "meshes/" + filename)
            material(visual, YELLOW if stripe % 2 == 0 else BLACK)


# 노면 숫자는 외부 폰트·텍스처 없이 생성하여 화면과 슬롯 메타데이터를 일치시킵니다.
DIGITS = {"1": ["010", "110", "010", "010", "111"], "2": ["110", "001", "010", "100", "111"],
          "3": ["110", "001", "010", "001", "110"], "4": ["101", "101", "111", "001", "001"],
          "5": ["111", "100", "110", "001", "110"], "6": ["011", "100", "111", "101", "111"]}


def add_grid_and_finish(model, res, generator):
    config = res["experimental_facilities"]["starting_grid"]
    length, width, line = config["length_m"], config["width_m"], config["line_width_m"]
    if width <= .15 or length <= .20 or line >= .02:
        raise ValueError("출발 표시 외곽은 차량보다 커야 하고 선 두께는 2 cm 미만이어야 합니다.")
    ranking = sorted(res["grid_slots"], key=lambda slot: (res["meta"]["startfinish_s"] - slot["s"]) % res["meta"]["Ltot"])
    paint = link_at(model, "start_grid_paint", 0, 0)
    for rank, slot in enumerate(ranking, 1):
        slot["painted_number"] = rank
        specs = [("front", length / 2, 0, line, width),
                 ("left", 0, width / 2, length, line), ("right", 0, -width / 2, length, line)]
        for suffix, forward, left, sx, sy in specs:
            x, y = local_to_world(slot["x"], slot["y"], slot["yaw"], forward, left)
            box(paint, f"slot_{slot['index']}_{suffix}", (x, y, PAINT_CENTER_M), (sx, sy, PAINT_THICKNESS_M),
                WHITE, rpy=(0, 0, slot["yaw"]))
        cell = config["number_height_m"] / 5
        for row, cells in enumerate(DIGITS[str(rank)]):
            for col, filled in enumerate(cells):
                if filled == "0":
                    continue
                forward = -length / 2 - .025 - (row + .5) * cell
                left = (1 - col) * cell
                x, y = local_to_world(slot["x"], slot["y"], slot["yaw"], forward, left)
                box(paint, f"slot_{slot['index']}_number_{row}_{col}", (x, y, PAINT_CENTER_M),
                    (cell, cell, PAINT_THICKNESS_M), WHITE, rpy=(0, 0, slot["yaw"]))

    config = res["experimental_facilities"]["finish_line"]
    x, y, yaw = generator.sample_at_s(res["arr"], res["meta"]["startfinish_s"], res["meta"]["Ltot"])
    width = float(res["track_width_at"](res["meta"]["startfinish_s"]))
    finish = link_at(model, "finish_line_paint", x, y, PAINT_CENTER_M, yaw)
    across, rows = int(config["cells_across"]), int(config["rows"])
    if not 2 <= across <= 30 or not 1 <= rows <= 8:
        raise ValueError("피니시 체크무늬 개수가 올바르지 않습니다.")
    sx, sy = config["depth_m"] / rows, width / across
    for row in range(rows):
        for col in range(across):
            box(finish, f"checker_{row}_{col}", (-config["depth_m"] / 2 + (row + .5) * sx,
                -width / 2 + (col + .5) * sy, 0), (sx, sy, PAINT_THICKNESS_M),
                WHITE if (row + col) % 2 == 0 else BLACK)
    res["finish_line"] = {"s_m": res["meta"]["startfinish_s"], "x": float(x), "y": float(y),
                          "yaw_rad": float(yaw), "width_m": width, **config,
                          "collision": False, "paint_center_height_m": PAINT_CENTER_M}


def add_traffic_light(model, res):
    light = res["traffic_light"]
    config = res["experimental_facilities"]["traffic_light"]
    # 로컬 X는 진행 방향, Y는 횡단 방향입니다. 가로대는 Y축으로 뻗습니다.
    yaw, width = light["yaw"], light["width"]
    post_offset = width / 2 + .11
    px, py = local_to_world(light["x"], light["y"], yaw, 0, post_offset)
    post = link_at(model, "tl_post", px, py, yaw=yaw)
    box(post, "post", (0, 0, config["gantry_height_m"] / 2), (.025, .025, config["gantry_height_m"]),
        STEEL, collision=True)
    box(post, "foot", (0, 0, .008), (.07, .07, .016), STEEL, collision=True)
    beam = link_at(model, "tl_beam", light["x"], light["y"], yaw=yaw)
    box(beam, "cross_track_beam", (0, 0, config["gantry_height_m"]),
        (.035, 2 * post_offset + .04, .035), STEEL, collision=True)
    box(beam, "housing", (0, 0, config["lamp_center_height_m"]), (.06, .36, .09), BLACK, collision=True)
    box(beam, "hanger", (.015, 0, (config["gantry_height_m"] + config["lamp_center_height_m"]) / 2),
        (.025, .018, config["gantry_height_m"] - config["lamp_center_height_m"]), STEEL, collision=True)
    light["lamp_poses"] = {}
    for name, left, color in (("red", .105, "1 0.015 0.005 1"),
                              ("yellow", 0, "0.12 0.085 0.004 1"),
                              ("green", -.105, "0.005 0.10 0.016 1")):
        x, y = local_to_world(light["x"], light["y"], yaw, -.036, left)
        lamp = link_at(model, "lamp_" + name, x, y, config["lamp_center_height_m"], yaw)
        visual = ET.SubElement(lamp, "visual", name="lamp_" + name + "_vis")
        pose(visual, rpy=(0, math.pi / 2, 0))
        cylinder = ET.SubElement(ET.SubElement(visual, "geometry"), "cylinder")
        value(cylinder, "radius", config["lamp_radius_m"])
        value(cylinder, "length", ".012")
        material(visual, color, color if name == "red" else "0 0 0 1")
        value(visual, "cast_shadows", "false")
        light["lamp_poses"][name] = {"x": x, "y": y, "z": config["lamp_center_height_m"],
                                     "facing_yaw_rad": math.atan2(-math.sin(yaw), -math.cos(yaw))}


def marker_cells(marker_id):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, 6)
    return cv2.aruco.drawMarker(dictionary, marker_id, 6)


def add_markers(model, res):
    config = res["experimental_facilities"]["markers"]
    size = config["black_square_size_m"]
    board_size = size + 2 * config["quiet_zone_m"]
    for marker in res["markers"]:
        center = marker["center_height"]
        link = link_at(model, f"aruco_{marker['id']}", marker["x"], marker["y"], yaw=marker["yaw"])
        post_height = center - board_size / 2
        box(link, "stand", (0, 0, post_height / 2), (config["post_width_m"], config["post_width_m"], post_height),
            STEEL, collision=True)
        box(link, "base", (0, 0, .008), (.05, .05, .016), STEEL, collision=True)
        # 검은 장식 테두리는 ArUco 후보 중복 제거 단계에서 실제 경계를 밀어낼 수 있습니다.
        # 지지판까지 흰색으로 하여 코드 바깥에 두 번째 검은 사각형을 만들지 않습니다.
        box(link, "backing", (-.004, 0, center), (.009, board_size, board_size), WHITE, collision=True)
        box(link, "white_quiet_zone", (.001, 0, center), (.001, board_size, board_size), WHITE)
        cells = marker_cells(marker["id"])
        cell = size / 6
        for row in range(6):
            for col in range(6):
                if cells[row, col] != 0:
                    continue
                # +X 면에서 보았을 때 영상의 오른쪽은 로컬 +Y, 위쪽은 +Z입니다.
                box(link, f"ink_{row}_{col}", (.0018, -size / 2 + (col + .5) * cell,
                    center + size / 2 - (row + .5) * cell), (.0004, cell, cell), BLACK)
        marker["face_pose"] = {"x": marker["x"] + .002 * math.cos(marker["yaw"]),
                               "y": marker["y"] + .002 * math.sin(marker["yaw"]),
                               "z": center, "yaw_rad": marker["yaw"]}


def replace_facilities(world_path, res, generator):
    """생성기의 시설 링크만 교체합니다. 노면·벽·그리드 위치는 변경하지 않습니다."""
    tree = ET.parse(world_path)
    model = tree.find("./world/model[@name='it_arena_track_static']")
    for link in list(model.findall("link")):
        name = link.attrib["name"]
        if name.startswith(("bump_", "grid_slot_", "aruco_", "lamp_")) or name in ("tl_post", "tl_beam"):
            model.remove(link)
    add_bumps(model, res, world_path.parent)
    add_grid_and_finish(model, res, generator)
    add_traffic_light(model, res)
    add_markers(model, res)
    ET.indent(tree, space="  ")
    tree.write(world_path, encoding="utf-8", xml_declaration=True)


def update_scene(scene, res, generator):
    config = res["experimental_facilities"]
    scene["speed_bumps"].update({"profile": "raised_cosine", "bump_length_m": config["speed_bump"]["length_m"],
                                "bump_height_m": config["speed_bump"]["height_m"], "height_reference": "road_top",
                                "longitudinal_axis": "local_x_along_route",
                                "collision": f"{config['speed_bump']['subdivisions']} inclined box strips; top endpoints match visual profile samples"})
    scene["traffic_light"].update({"initial_state": "red", "state_control": "static_initial_state_only",
                                  "lamp_center_height_m": config["traffic_light"]["lamp_center_height_m"],
                                  "lamp_radius_m": config["traffic_light"]["lamp_radius_m"],
                                  "lamp_poses": res["traffic_light"]["lamp_poses"],
                                  "beam_axis": "local_y_across_route", "udp_visual_controller_connected": False})
    scene["traffic_light"].pop("udp_port", None)
    scene["aruco_markers"].update({"marker_size_m": config["markers"]["black_square_size_m"],
                                    "size_reference": "outer_edge_of_black_border",
                                    "quiet_zone_each_side_m": config["markers"]["quiet_zone_m"],
                                    "mount_bottom_height_m": config["markers"]["center_height_m"] - .05,
                                    "mount_type": "freestanding_upstream_facing_sign",
                                    "rendering": "DICT_4X4_50 cells as untextured SDF geometry; preserved PNG files unchanged"})
    by_id = {marker["id"]: marker for marker in res["markers"]}
    for marker in scene["aruco_markers"]["markers"]:
        source = by_id[marker["id"]]
        marker.update(pose=source["face_pose"], pose_reference="black_square_front_center",
                      normal_note=f"local +X faces a point {config['markers']['face_target_distance_m']:g} m upstream along the main route",
                      approach_target_xy_m=source["approach_target"])
    scene["starting_grid"]["paint"] = {**config["starting_grid"], "paint_center_height_m": PAINT_CENTER_M,
                                        "collision": False, "numbering": "front_to_back_1_to_6",
                                        "slot_number_map": {str(slot["index"]): slot["painted_number"] for slot in res["grid_slots"]}}
    scene["finish_line"] = res["finish_line"]
    cases = []
    for slot in scene["starting_grid"]["slots"]:
        cases.append({"name": f"signal_grid_{slot['index']}", "x": slot["x"], "y": slot["y"],
                      "yaw_rad": slot["yaw_rad"], "expected_signal": "red"})
    for marker in res["markers"]:
        for distance in (1.5, 1.0, .75):
            x, y, yaw = generator.sample_at_s(res["arr"], (marker["s"] - distance) % res["meta"]["Ltot"], res["meta"]["Ltot"])
            cases.append({"name": f"marker_{marker['id']}_{int(distance * 100)}cm", "x": float(x), "y": float(y),
                          "yaw_rad": float(yaw), "expected_marker_id": marker["id"], "approach_distance_m": distance})
    scene["facility_inspection"] = {"status": "experimental_not_official", "config": config,
                                    "camera_cases": cases, "test_ground_truth_only": True}
