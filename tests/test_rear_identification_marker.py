"""팀 시험용 차량 후면 5 cm ArUco ID 10 형상 검사."""

from pathlib import Path
import re
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import xacro
import yaml


REPO = Path(__file__).resolve().parents[1]
MODEL = REPO / "src/arena_description/models/arena_car/model.sdf.xacro"
CONFIG = REPO / "src/arena_description/config/vehicle.yaml"


def load():
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))["vehicle"]
    marker = config["identification_marker"]
    xyz, rpy = marker["xyz_m"], marker["rpy_rad"]
    model = ET.fromstring(xacro.process_file(str(MODEL), mappings={
        "rear_marker_enabled": str(marker["enabled"]).lower(),
        "rear_marker_x": str(xyz[0]),
        "rear_marker_y": str(xyz[1]),
        "rear_marker_z": str(xyz[2]),
        "rear_marker_yaw": str(rpy[2]),
        "rear_marker_board_size": str(marker["printed_board_size_m"]),
        "rear_marker_code_size": str(marker["black_code_size_m"]),
    }).toxml()).find("model")
    return config, marker, model


def test_rear_marker_configuration_is_explicit_and_does_not_reuse_course_ids():
    _, marker, _ = load()
    assert marker["dictionary"] == "DICT_4X4_50"
    assert marker["id"] == 10
    assert marker["id"] not in {0, 20, 30, 45}
    assert marker["printed_board_size_m"] == .05
    assert marker["black_code_size_m"] + 2 * marker["quiet_zone_each_side_m"] == .05
    assert marker["status"].startswith("team_provisional_")


def test_rear_marker_visual_encodes_dictionary_id_10_without_changing_collision_shape():
    config, marker, model = load()
    chassis = model.find("link[@name='chassis']")
    board = chassis.find("visual[@name='rear_marker_id10_board']")
    np.testing.assert_allclose(
        list(map(float, board.findtext("geometry/box/size").split())),
        [.001, .05, .05],
    )
    ink = [visual for visual in chassis.findall("visual") if visual.attrib["name"].startswith("rear_marker_id10_ink_")]
    rendered = np.full((6, 6), 255, dtype=np.uint8)
    for visual in ink:
        match = re.fullmatch(r"rear_marker_id10_ink_(\d)_(\d)", visual.attrib["name"])
        assert match
        rendered[int(match.group(1)), int(match.group(2))] = 0
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    expected = cv2.aruco.drawMarker(dictionary, 10, 6)
    np.testing.assert_array_equal(rendered, expected)
    assert not [
        collision for collision in chassis.findall("collision")
        if collision.attrib["name"].startswith("rear_marker_")
    ]

    board_pose = list(map(float, board.findtext("pose").split()))
    chassis_z = float(chassis.findtext("pose").split()[2])
    np.testing.assert_allclose(board_pose[:2], marker["xyz_m"][:2], atol=1e-9)
    assert abs(chassis_z + board_pose[2] - marker["xyz_m"][2]) < 1e-9

    marker_bottom = marker["xyz_m"][2] - marker["printed_board_size_m"] / 2
    marker_top = marker["xyz_m"][2] + marker["printed_board_size_m"] / 2
    rear_tof = next(item for item in config["sensors"]["tof_ring"]["modules"] if item["name"] == "rear")
    lidar_z = config["sensors"]["lidar_2d"]["xyz_m"][2]
    assert marker_bottom > rear_tof["xyz_m"][2]
    assert marker_top < lidar_z
