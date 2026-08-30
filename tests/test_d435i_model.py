"""D435i 명목 사양과 시뮬레이션 센서 분리를 검증합니다."""

import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml


REPO = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPO / "src/arena_description/config/vehicle.yaml"
MODEL_PATH = REPO / "src/arena_description/models/arena_car/model.sdf.xacro"
LAUNCH_PATH = REPO / "src/arena_bringup/launch/simulation.launch.py"


def d435i_config():
    return yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))["vehicle"]["sensors"]["d435i"]


def default_model():
    return ET.fromstring(xacro.process_file(str(MODEL_PATH)).toxml()).find("model")


def test_official_nominal_stream_profiles_are_explicit():
    config = d435i_config()
    assert config["active_stream_profile"] == "high_speed_async"
    assert config["stream_profiles"] == {
        "high_speed_async": {
            "color_rate_hz": 60.0,
            "depth_rate_hz": 90.0,
            "hardware_sync_compatible": False,
        },
        "synchronized_60": {
            "color_rate_hz": 60.0,
            "depth_rate_hz": 60.0,
            "hardware_sync_compatible": True,
        },
        "low_load_30": {
            "color_rate_hz": 30.0,
            "depth_rate_hz": 30.0,
            "hardware_sync_compatible": True,
        },
    }
    assert config["color"]["horizontal_fov_deg"] == 69.4
    assert config["color"]["vertical_fov_deg"] == 42.5
    assert config["depth"]["horizontal_fov_deg"] == 87.0
    assert config["depth"]["vertical_fov_deg"] == 58.0
    assert config["depth"]["minimum_depth_m"] == 0.195
    assert config["depth"]["qualified_quality_range_max_m"] == 2.0
    assert config["depth"]["nominal_range_max_m"] == 3.0
    assert config["depth"]["simulation_far_clip_m"] == 3.0
    assert config["color"]["render_far_clip_m"] > 10.3


def test_default_model_uses_separate_color_and_depth_sensors():
    model = default_model()
    sensors = {sensor.attrib["name"]: sensor for sensor in model.findall("./link/sensor")}
    assert "d435i_rgbd" not in sensors
    assert sensors["d435i_color"].attrib["type"] == "camera"
    assert sensors["d435i_depth"].attrib["type"] == "depth_camera"
    assert sensors["d435i_color"].findtext("topic") == "camera/color/image_raw"
    assert sensors["d435i_depth"].findtext("topic") == "camera/depth/image_rect_raw"
    assert float(sensors["d435i_color"].findtext("update_rate")) == 60.0
    assert float(sensors["d435i_depth"].findtext("update_rate")) == 90.0

    color_camera = sensors["d435i_color"].find("camera")
    depth_camera = sensors["d435i_depth"].find("camera")
    assert float(color_camera.findtext("horizontal_fov")) == pytest.approx(math.radians(69.4))
    assert float(depth_camera.findtext("horizontal_fov")) == pytest.approx(math.radians(87.0))
    assert color_camera.findtext("optical_frame_id") == "camera_color_optical_frame"
    assert depth_camera.findtext("optical_frame_id") == "camera_depth_optical_frame"

    for camera, vertical_fov_deg in ((color_camera, 42.5), (depth_camera, 58.0)):
        width = float(camera.findtext("image/width"))
        height = float(camera.findtext("image/height"))
        fx = float(camera.findtext("lens/intrinsics/fx"))
        fy = float(camera.findtext("lens/intrinsics/fy"))
        assert fx == pytest.approx((width / 2.0) / math.tan(float(camera.findtext("horizontal_fov")) / 2.0))
        assert 2.0 * math.atan((height / 2.0) / fy) == pytest.approx(math.radians(vertical_fov_deg))

    assert float(color_camera.findtext("clip/far")) == 20.0
    assert float(depth_camera.findtext("clip/near")) == 0.195
    assert float(depth_camera.findtext("clip/far")) == 3.0
    assert float(depth_camera.findtext("depth_camera/clip/near")) == 0.195
    assert float(depth_camera.findtext("depth_camera/clip/far")) == 3.0


def test_launch_profile_names_and_intrinsic_calculation_match_config():
    spec = importlib.util.spec_from_file_location("arena_simulation_launch_d435i", LAUNCH_PATH)
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    config = d435i_config()
    assert set(launch.D435I_STREAM_PROFILES) == set(config["stream_profiles"])
    color = config["color"]
    intrinsics = launch._nominal_intrinsics(color)
    assert intrinsics["horizontal_fov_rad"] == pytest.approx(math.radians(69.4))
    assert intrinsics["fx_px"] == pytest.approx(612.3337640816779)
    assert intrinsics["fy_px"] == pytest.approx(617.1589761579678)
    assert launch._resolve_d435i_profile(config, "configured")[0] == "high_speed_async"
    config["active_stream_profile"] = "low_load_30"
    assert launch._resolve_d435i_profile(config, "configured")[1]["depth_rate_hz"] == 30.0
    assert launch._resolve_d435i_profile(config, "synchronized_60")[1]["depth_rate_hz"] == 60.0
    with pytest.raises(RuntimeError, match="d435i_profile"):
        launch._resolve_d435i_profile(config, "missing")
    config["stream_profiles"]["synchronized_60"]["depth_rate_hz"] = 90.0
    with pytest.raises(RuntimeError, match="equal"):
        launch._resolve_d435i_profile(config, "synchronized_60")
    config["stream_profiles"]["low_load_30"]["depth_rate_hz"] = 0.0
    with pytest.raises(RuntimeError, match="positive"):
        launch._resolve_d435i_profile(config, "low_load_30")


def test_sensor_bridges_use_separate_info_topics_and_lazy_pointcloud():
    bridge_dir = REPO / "src/arena_bringup/config"
    entries = yaml.safe_load((bridge_dir / "d435i_sensor_bridge.yaml").read_text(encoding="utf-8"))
    assert {entry["topic_name"] for entry in entries} == {
        "/camera/color/image_raw", "/camera/color/camera_info", "/camera/depth/image_rect_raw",
        "/camera/depth/camera_info", "/camera/imu",
    }
    assert all(entry["direction"] == "GZ_TO_ROS" for entry in entries)
    assert all(entry["publisher_queue"] == 5 for entry in entries if entry["ros_type_name"] == "sensor_msgs/msg/Image")
    pointcloud, = yaml.safe_load((bridge_dir / "d435i_pointcloud_bridge.yaml").read_text(encoding="utf-8"))
    assert pointcloud["gz_topic_name"] == "/camera/depth/image_rect_raw/points"
    assert pointcloud["ros_topic_name"] == "/camera/depth/color/points"
    assert pointcloud["lazy"] is True
