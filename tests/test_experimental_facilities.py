"""새 시설의 축 방향·노면 높이·마커 비트·곡면/충돌면 일치 회귀 검사."""

from collections import Counter
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
from experimental_facilities import ROAD_TOP_M, bump_mesh, marker_cells  # noqa: E402
from validate_facility_visibility import detect_markers  # noqa: E402

WORLD = REPO / "src/arena_gazebo/worlds/it_arena_experimental"


@pytest.fixture(scope="module")
def data():
    return (ET.parse(WORLD / "world.sdf").find("./world/model[@name='it_arena_track_static']"),
            json.loads((WORLD / "scene.json").read_text(encoding="utf-8")))


def test_bump_is_a_closed_smooth_profile_with_correct_axes():
    vertices, faces = bump_mesh(.20, .45, .01, 40)
    np.testing.assert_allclose(np.ptp(vertices, axis=0), [.20, .45, .011], atol=1e-12)
    top = vertices[::4]
    assert top[0, 2] == top[-1, 2] == 0
    assert top[20, 2] == pytest.approx(.01)
    assert abs((top[1, 2] - top[0, 2]) / (top[1, 0] - top[0, 0])) < .013
    counts = Counter(tuple(sorted(edge)) for a, b, c in faces for edge in ((a, b), (b, c), (c, a)))
    assert set(counts.values()) == {2}
    signed_volume = sum(np.dot(vertices[a], np.cross(vertices[b], vertices[c])) / 6 for a, b, c in faces)
    assert signed_volume == pytest.approx(.20 * .45 * (.01 / 2 + .001), rel=1e-10)


def test_bump_collision_top_edges_follow_rendered_curve(data):
    model, scene = data
    bump = model.find("link[@name='safety_bump_0']")
    assert float(bump.findtext("pose").split()[2]) == ROAD_TOP_M
    collisions = bump.findall("collision")
    assert len(collisions) == 40
    assert not bump.findall("collision/geometry/mesh")  # 설치된 DART/ODE의 메시 충돌 오류를 피합니다.
    endpoints = []
    for collision in collisions:
        x, y, z, roll, pitch, yaw = map(float, collision.findtext("pose").split())
        sx, sy, sz = map(float, collision.findtext("geometry/box/size").split())
        assert y == roll == yaw == 0
        assert sy == .45
        c, s = math.cos(pitch), math.sin(pitch)
        points = [(x + c * lx + s * sz / 2, z - s * lx + c * sz / 2) for lx in (-sx / 2, sx / 2)]
        for px, pz in points:
            assert pz == pytest.approx(.01 * .5 * (1 + math.cos(2 * math.pi * px / .20)), abs=1e-9)
        endpoints.append(points)
    np.testing.assert_allclose([pair[1] for pair in endpoints[:-1]], [pair[0] for pair in endpoints[1:]], atol=1e-9)
    assert scene["speed_bumps"]["bump_length_m"] == .20
    assert scene["speed_bumps"]["bump_height_m"] == .01
    # 이상적인 곡선과 5 mm 선분 사이 최대 높이 오차는 약 0.016 mm 이하입니다.
    for left, right in endpoints:
        xs = np.linspace(left[0], right[0], 31)
        chord = left[1] + (xs - left[0]) * (right[1] - left[1]) / (right[0] - left[0])
        curve = .005 * (1 + np.cos(2 * math.pi * xs / .20))
        assert max(abs(chord - curve)) < .000016
    for uri in bump.findall("visual/geometry/mesh/uri"):
        assert (WORLD / uri.text).is_file()


def test_signal_spans_road_and_only_red_is_lit(data):
    model, scene = data
    beam = model.find("link[@name='tl_beam']/visual[@name='cross_track_beam']")
    sx, sy, _ = map(float, beam.findtext("geometry/box/size").split())
    assert sx < .05 and sy > scene["track"]["width_m"]
    for color in ("red", "yellow", "green"):
        emissive = model.findtext(f"link[@name='lamp_{color}']/visual/material/emissive")
        assert emissive != "0 0 0 1" if color == "red" else emissive == "0 0 0 1"
    assert scene["traffic_light"]["state_control"] == "static_initial_state_only"
    assert not scene["traffic_light"]["udp_visual_controller_connected"]


def test_bump_obj_has_explicit_face_normals_and_colored_materials():
    # OGRE2에서는 법선 없는 OBJ가 단색 흰색으로 보이는 것을 실제 RGB에서 확인했습니다.
    for stripe in range(10):
        lines = (WORLD / "meshes" / f"bump_0_stripe_{stripe}.obj").read_text(encoding="utf-8").splitlines()
        normals = [list(map(float, line.split()[1:])) for line in lines if line.startswith("vn ")]
        faces = [line.split()[1:] for line in lines if line.startswith("f ")]
        assert len(normals) == len(faces) > 0
        np.testing.assert_allclose(np.linalg.norm(normals, axis=1), 1, atol=1e-9)
        for index, face in enumerate(faces, 1):
            assert len(face) == 3
            assert all(vertex.endswith(f"//{index}") for vertex in face)
        assert "usemtl " + ("yellow" if stripe % 2 == 0 else "black") in lines
        assert "mtllib bump_surface.mtl" in lines
    materials = (WORLD / "meshes/bump_surface.mtl").read_text(encoding="utf-8")
    assert "newmtl yellow" in materials and "newmtl black" in materials


def test_all_six_grid_marks_and_finish_are_above_road_and_not_collisions(data):
    model, scene = data
    grid = model.find("link[@name='start_grid_paint']")
    assert not grid.findall("collision")
    fronts = [visual for visual in grid.findall("visual") if visual.attrib["name"].endswith("_front")]
    assert len(fronts) == 6
    for visual in grid.findall("visual"):
        z = float(visual.findtext("pose").split()[2])
        thickness = float(visual.findtext("geometry/box/size").split()[2])
        assert z - thickness / 2 > ROAD_TOP_M
    assert scene["starting_grid"]["paint"]["slot_number_map"] == {"0": 5, "1": 3, "2": 1, "3": 6, "4": 4, "5": 2}
    finish = model.find("link[@name='finish_line_paint']")
    assert not finish.findall("collision")
    assert len(finish.findall("visual")) == 18
    assert scene["finish_line"]["width_m"] == .45
    assert float(finish.findtext("pose").split()[2]) > ROAD_TOP_M


@pytest.mark.parametrize("marker_id", [0, 20, 30, 45])
def test_freestanding_marker_cells_are_correct_and_not_mirrored(data, marker_id):
    model, scene = data
    sign = model.find(f"link[@name='aruco_{marker_id}']")
    assert sign.find("collision[@name='stand_collision']") is not None
    assert sign.find("collision[@name='backing_collision']") is not None
    backing = sign.find("visual[@name='backing']")
    assert backing.findtext("material/diffuse") == "0.96 0.96 0.94 1"
    np.testing.assert_allclose(list(map(float, backing.findtext("geometry/box/size").split()))[1:], [.13, .13])
    expected = marker_cells(marker_id)
    actual = np.full((6, 6), 255, dtype=np.uint8)
    for visual in sign.findall("visual"):
        if visual.attrib["name"].startswith("ink_"):
            _, row, col = visual.attrib["name"].split("_")
            actual[int(row), int(col)] = 0
            _, y, z, *_ = map(float, visual.findtext("pose").split())
            assert y == pytest.approx(-.05 + (int(col) + .5) * .1 / 6, abs=1e-9)
            assert z == pytest.approx(.20 + .05 - (int(row) + .5) * .1 / 6, abs=1e-9)
    np.testing.assert_array_equal(actual, expected)
    rgb = np.repeat(np.pad(np.kron(actual, np.ones((40, 40), dtype=np.uint8)), 40, constant_values=255)[:, :, None], 3, axis=2)
    assert marker_id in detect_markers(rgb)
    metadata = next(marker for marker in scene["aruco_markers"]["markers"] if marker["id"] == marker_id)
    target = metadata["approach_target_xy_m"]
    pose = metadata["pose"]
    desired = math.atan2(target[1] - pose["y"], target[0] - pose["x"])
    assert abs(math.atan2(math.sin(desired - pose["yaw_rad"]), math.cos(desired - pose["yaw_rad"]))) < .002
    assert scene["aruco_markers"]["marker_size_m"] == .1


def test_light_lenses_fit_nominal_rgb_frustum_from_all_grid_slots(data):
    _, scene = data
    config = yaml.safe_load((REPO / "src/arena_description/config/vehicle.yaml").read_text(encoding="utf-8"))
    camera = config["vehicle"]["sensors"]["d435i"]
    color = camera["color"]
    tx = math.tan(math.radians(color["horizontal_fov_deg"]) / 2)
    ty = math.tan(math.radians(color["vertical_fov_deg"]) / 2)
    radius = scene["traffic_light"]["lamp_radius_m"]
    for slot in scene["starting_grid"]["slots"]:
        c, s = math.cos(slot["yaw_rad"]), math.sin(slot["yaw_rad"])
        cx = slot["x"] + c * camera["xyz_m"][0] - s * camera["xyz_m"][1]
        cy = slot["y"] + s * camera["xyz_m"][0] + c * camera["xyz_m"][1]
        cz = ROAD_TOP_M + camera["xyz_m"][2]
        for lamp in scene["traffic_light"]["lamp_poses"].values():
            dx, dy, dz = lamp["x"] - cx, lamp["y"] - cy, lamp["z"] - cz
            forward, left = c * dx + s * dy, -s * dx + c * dy
            assert forward > radius
            assert abs(left) + radius < (forward - radius) * tx
            assert abs(dz) + radius < (forward - radius) * ty


def test_visibility_cases_cover_all_slots_and_marker_approaches(data):
    _, scene = data
    cases = scene["facility_inspection"]["camera_cases"]
    assert len(cases) == 18
    assert len([case for case in cases if "expected_signal" in case]) == 6
    for marker_id in [0, 20, 30, 45]:
        assert {case["approach_distance_m"] for case in cases if case.get("expected_marker_id") == marker_id} == {1.5, 1., .75}
