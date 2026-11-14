import copy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from prepare_uno_q_quick_setup import create_parameters


MASK = dict(schema="arena_quick_self_mask_v1", enabled=True, frame_id="base_link", status="temporary_user_approximation_unmeasured",
            center_xy_m=[.00365, 0.], body_length_m=.20, body_width_m=.15, margin_each_side_m=.03)
CALIBRATION = {"/**": {"ros__parameters": {"motion_quick_calibration": True}}}


def test_global_and_node_overlay_preserves_motion_policy():
    base = {"local_pursuit": {"ros__parameters": {"max_speed_mps": .3, "use_sim_time": False,
                                                  "motion_quick_calibration": False}},
            "lidar_safety": {"ros__parameters": {"guard_policy": "existing"}}}
    before = copy.deepcopy(base)
    result = create_parameters(CALIBRATION, MASK, base)
    assert base == before
    assert result["local_pursuit"]["ros__parameters"]["max_speed_mps"] == .3
    assert result["lidar_safety"]["ros__parameters"]["guard_policy"] == "existing"
    assert all(node["ros__parameters"]["motion_quick_calibration"] for node in result.values())
    assert len({node["ros__parameters"]["lidar_self_mask_json"] for node in result.values()}) == 1
    assert all(node["ros__parameters"]["motion_imu_topic"] == "/hardware_runtime/imu" for node in result.values())
    assert all(node["ros__parameters"]["rear_blind_half_angle_deg"] == 0. for node in result.values())
    assert all(node["ros__parameters"]["lidar_blind_sectors_json"] == "" for node in result.values())
    assert all(node["ros__parameters"]["known_structure_policy"] == "off" for node in result.values())
    assert not any(node["ros__parameters"]["g1_g2_rear_wall_witness"] for node in result.values())


@pytest.mark.parametrize("key,value", [("body_length_m", .26), ("body_width_m", .20),
                                      ("margin_each_side_m", .30), ("center_xy_m", [0., 0.]),
                                      ("enabled", False)])
def test_incorrect_dimensions_or_center_rejected(key, value):
    mask = dict(MASK, **{key: value})
    with pytest.raises(ValueError):
        create_parameters(CALIBRATION, mask)


def test_non_ros_base_rejected():
    with pytest.raises(ValueError):
        create_parameters(CALIBRATION, MASK, {"random": {"value": 1}})


def test_selected_overlay_retains_rear_return_outside_box():
    import math
    from types import SimpleNamespace
    import numpy as np
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root / "src/arena_autonomy"))
    from arena_autonomy.quick_self_mask import QuickSelfMask
    from arena_autonomy.local_path import mask_scan
    parameters = create_parameters(CALIBRATION, MASK)["/**"]["ros__parameters"]
    transform = SimpleNamespace(lidar_x_m=0., lidar_y_m=0., lidar_yaw_rad=0., lidar_frame_id="laser_frame")
    source = SimpleNamespace(header=SimpleNamespace(frame_id="laser_frame"), ranges=[.03, .30],
                             angle_min=math.pi-.002, angle_increment=.001, range_min=.01, range_max=8.)
    masked = QuickSelfMask(parameters["lidar_self_mask_json"], transform).apply(source)
    result = mask_scan(masked, parameters["rear_blind_half_angle_deg"])
    assert np.isnan(result.ranges[0])
    assert result.ranges[1] == .30
    assert source.ranges == [.03, .30]
