"""팀 등록 차량 후면 5 cm ArUco ID 49와 등록 PNG 일치 검사."""

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
    assert marker["id"] == 49
    assert marker["id"] not in {0, 20, 30, 45}
    assert marker["printed_board_size_m"] == .05
    np.testing.assert_allclose(marker["black_code_size_m"] + 2 * marker["quiet_zone_each_side_m"],
                               .05, rtol=0, atol=1e-12)
    assert marker["status"] == "team_selected_registered_mount_provisional"
    assert marker["black_code_size_m"] == .0348


def test_rear_marker_visual_encodes_dictionary_id_49_without_changing_collision_shape():
    config, marker, model = load()
    chassis = model.find("link[@name='chassis']")
    board = chassis.find("visual[@name='rear_marker_id49_board']")
    np.testing.assert_allclose(
        list(map(float, board.findtext("geometry/box/size").split())),
        [.001, .05, .05],
    )
    ink = [visual for visual in chassis.findall("visual") if visual.attrib["name"].startswith("rear_marker_id49_ink_")]
    rendered = np.full((6, 6), 255, dtype=np.uint8)
    for visual in ink:
        match = re.fullmatch(r"rear_marker_id49_ink_(\d)_(\d)", visual.attrib["name"])
        assert match
        rendered[int(match.group(1)), int(match.group(2))] = 0
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    make = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    expected = make(dictionary, 49, 6)
    np.testing.assert_array_equal(rendered, expected)
    # 1000 px 등록 PNG와 물리 비율/셀 배치가 같고 좌우 반전되지 않는지 대조한다.
    source = cv2.imread(str(REPO / marker['source_image']), cv2.IMREAD_GRAYSCALE)
    reproduced = np.full((1000, 1000), 255, dtype=np.uint8)
    reproduced[152:848, 152:848] = np.repeat(np.repeat(rendered, 116, axis=0), 116, axis=1)
    np.testing.assert_array_equal(reproduced, source)
    assert marker['black_code_size_m'] / marker['printed_board_size_m'] == 696 / 1000
    for visual in ink:
        row, col = map(int, visual.attrib['name'].rsplit('_', 2)[1:])
        pose = list(map(float, visual.findtext('pose').split()))
        step = marker['black_code_size_m'] / 6
        # 후방(-X)에서 볼 때 화면 오른쪽은 차량 -Y. 등록 PNG의 열과 일치해야 한다.
        assert abs(pose[1] - (-marker['black_code_size_m'] / 2 + (col + .5) * step) * -1) < 1e-9
        assert pose[0] < marker['xyz_m'][0]
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
