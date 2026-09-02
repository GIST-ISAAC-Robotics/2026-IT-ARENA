"""VL53L7CX급 하부 ToF 링의 설정과 생성 모델을 검증합니다."""

import importlib.util
import math
from pathlib import Path
import struct
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml


REPO = Path(__file__).resolve().parents[1]
CONFIG_PATH = REPO / "src/arena_description/config/vehicle.yaml"
MODEL_PATH = REPO / "src/arena_description/models/arena_car/model.sdf.xacro"
LAUNCH_PATH = REPO / "src/arena_bringup/launch/simulation.launch.py"


def vehicle_config():
    return yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))["vehicle"]


def model_with(**mappings):
    root = ET.fromstring(xacro.process_file(str(MODEL_PATH), mappings=mappings).toxml())
    return root.find("model")


def load_launch_module():
    spec = importlib.util.spec_from_file_location("arena_simulation_launch_tof", LAUNCH_PATH)
    launch = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch)
    return launch


def test_tof_ring_uses_six_multizone_modules_and_two_profiles():
    tof = vehicle_config()["sensors"]["tof_ring"]
    assert tof["enabled"] is True
    assert tof["model_reference"] == "VL53L7CX"
    assert tof["active_profile"] == "tracking_8x8_15"
    assert tof["profiles"]["low_latency_4x4_60"]["horizontal_zones"] == 4
    assert tof["profiles"]["low_latency_4x4_60"]["vertical_zones"] == 4
    assert tof["profiles"]["low_latency_4x4_60"]["update_rate_hz"] == 60.0
    assert tof["profiles"]["tracking_8x8_15"]["horizontal_zones"] == 8
    assert tof["profiles"]["tracking_8x8_15"]["vertical_zones"] == 8
    assert tof["profiles"]["tracking_8x8_15"]["update_rate_hz"] == 15.0
    assert len(tof["modules"]) == 6
    assert len({module["name"] for module in tof["modules"]}) == 6


def test_lidar_rate_override_is_explicit_and_bounded():
    launch = load_launch_module()
    lidar = vehicle_config()["sensors"]["lidar_2d"]
    assert launch._resolve_lidar_rate(lidar, "configured") == 10.0
    assert launch._resolve_lidar_rate(lidar, "30") == 30.0
    for value in ("0.1", "101", "nan", "inf"):
        with pytest.raises(RuntimeError, match="lidar_rate_hz"):
            launch._resolve_lidar_rate(lidar, value)


def test_nominal_bearing_intervals_tile_360_but_do_not_prove_spatial_coverage():
    tof = vehicle_config()["sensors"]["tof_ring"]
    yaws = sorted(module["rpy_rad"][2] % (2.0 * math.pi) for module in tof["modules"])
    gaps = [
        (yaws[(index + 1) % len(yaws)] - yaw) % (2.0 * math.pi)
        for index, yaw in enumerate(yaws)
    ]
    assert gaps == pytest.approx([math.radians(60.0)] * 6)
    assert math.radians(tof["horizontal_fov_deg"]) == pytest.approx(max(gaps))


def test_optical_centers_and_carrier_proxy_fit_the_confirmed_planar_envelope():
    config = vehicle_config()
    tof = config["sensors"]["tof_ring"]
    half_length = config["footprint"]["length_m"] / 2.0
    half_width = config["footprint"]["width_m"] / 2.0
    thickness, width, _ = tof["carrier_size_m"]
    for module in tof["modules"]:
        x, y, _ = module["xyz_m"]
        yaw = module["rpy_rad"][2]
        # 캐리어는 광학면 뒤쪽 local x=[-thickness, 0]에 놓입니다.
        for local_x in (-thickness, 0.0):
            for local_y in (-width / 2.0, width / 2.0):
                world_x = x + math.cos(yaw) * local_x - math.sin(yaw) * local_y
                world_y = y + math.sin(yaw) * local_x + math.cos(yaw) * local_y
                assert abs(world_x) <= half_length + 1e-9
                assert abs(world_y) <= half_width + 1e-9


def test_default_sdf_has_six_3d_tof_grids_and_separate_point_topics():
    config = vehicle_config()
    tof = config["sensors"]["tof_ring"]
    model = model_with()
    sensors = {
        sensor.attrib["name"]: sensor
        for sensor in model.findall("./link/sensor")
        if sensor.attrib["name"].startswith("tof_")
    }
    assert set(sensors) == {f"tof_{module['name']}" for module in tof["modules"]}
    for module in tof["modules"]:
        link = model.find(f"link[@name='tof_{module['name']}_link']")
        pose = [float(value) for value in link.findtext("pose").split()]
        assert pose == pytest.approx([*module["xyz_m"], *module["rpy_rad"]])
        sensor = sensors[f"tof_{module['name']}"]
        assert sensor.attrib["type"] == "gpu_lidar"
        assert sensor.findtext("topic") == module["topic"]
        assert float(sensor.findtext("update_rate")) == 15.0
        assert int(sensor.findtext("lidar/scan/horizontal/samples")) == 8
        assert int(sensor.findtext("lidar/scan/vertical/samples")) == 8
        assert float(sensor.findtext("lidar/scan/horizontal/max_angle")) == pytest.approx(math.radians(26.25))
        assert float(sensor.findtext("lidar/scan/vertical/min_angle")) == pytest.approx(math.radians(-26.25))
        assert float(sensor.findtext("lidar/range/min")) == 0.020
        assert float(sensor.findtext("lidar/range/max")) == 3.50


def test_4x4_profile_and_disabled_model_are_selectable():
    model = model_with(
        tof_rate="60.0",
        tof_horizontal_zones="4",
        tof_vertical_zones="4",
    )
    tof_sensors = [
        sensor for sensor in model.findall("./link/sensor")
        if sensor.attrib["name"].startswith("tof_")
    ]
    assert len(tof_sensors) == 6
    assert all(sensor.findtext("update_rate") == "60.0" for sensor in tof_sensors)
    assert all(sensor.findtext("lidar/scan/horizontal/samples") == "4" for sensor in tof_sensors)
    assert all(sensor.findtext("lidar/scan/vertical/samples") == "4" for sensor in tof_sensors)
    assert all(float(sensor.findtext("lidar/scan/vertical/min_angle")) == pytest.approx(math.radians(-22.5))
               for sensor in tof_sensors)

    disabled = model_with(tof_enabled="false")
    assert not [
        sensor for sensor in disabled.findall("./link/sensor")
        if sensor.attrib["name"].startswith("tof_")
    ]


def test_launch_validates_profiles_coverage_and_vl53l7cx_rate_limits():
    launch = load_launch_module()
    config = vehicle_config()
    tof = config["sensors"]["tof_ring"]
    assert set(launch.TOF_PROFILES) == set(tof["profiles"])
    assert launch._resolve_tof_profile(tof, "configured")[0] == "tracking_8x8_15"
    assert launch._resolve_tof_profile(tof, "tracking_8x8_15")[1]["vertical_zones"] == 8
    launch._validate_tof_ring(tof, config["body"])

    with pytest.raises(RuntimeError, match="tof_profile"):
        launch._resolve_tof_profile(tof, "missing")
    tof["profiles"]["tracking_8x8_15"]["update_rate_hz"] = 16.0
    with pytest.raises(RuntimeError, match="15.0"):
        launch._resolve_tof_profile(tof, "tracking_8x8_15")
    tof["profiles"]["tracking_8x8_15"]["update_rate_hz"] = 15.0
    tof["modules"][0]["rpy_rad"][2] = math.radians(10.0)
    with pytest.raises(RuntimeError, match="coverage gap"):
        launch._validate_tof_ring(tof, config["body"])


def test_launch_bridges_only_the_multizone_pointcloud_representation():
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    assert "sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked" in source
    assert "f\"{topic}/points" in source
    assert "override_frame_id" in source
    tof_bridge_section = source.split(
        "if tof[\"enabled\"] and render_sensors:\n        for module in tof[\"modules\"]:", 1
    )[1].split("    autonomy =", 1)[0]
    assert "sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" not in tof_bridge_section


def test_chassis_visual_inset_does_not_shrink_collision_or_mass():
    model = model_with()
    chassis = model.find("link[@name='chassis']")
    collision_size = [float(value) for value in chassis.findtext("collision/geometry/box/size").split()]
    visual_size = [float(value) for value in chassis.findtext("visual/geometry/box/size").split()]
    assert collision_size == pytest.approx([.20, .15, .06])
    assert visual_size == pytest.approx([.188, .130, .06])
    assert float(chassis.findtext("inertial/mass")) == vehicle_config()["body"]["mass_kg"]
    assert sum(float(link.findtext("inertial/mass")) for link in model.findall("link")) == pytest.approx(2.0)
    for module in vehicle_config()["sensors"]["tof_ring"]["modules"]:
        x, y, _ = module["xyz_m"]
        assert abs(x) > visual_size[0] / 2 or abs(y) > visual_size[1] / 2


@pytest.mark.parametrize("invalid_change,match", [
    (lambda t: t["modules"][0].update(name="unexpected"), "module names"),
    (lambda t: t["modules"][0].update(topic=t["modules"][1]["topic"]), "unique topic"),
    (lambda t: t.update(range_min_m=float("nan")), "range limits"),
    (lambda t: t.update(horizontal_fov_deg=-60), "horizontal_fov"),
])
def test_invalid_tof_config_is_rejected_instead_of_silently_using_xacro_defaults(invalid_change, match):
    launch = load_launch_module()
    config = vehicle_config()
    invalid_change(config["sensors"]["tof_ring"])
    with pytest.raises(RuntimeError, match=match):
        launch._validate_tof_ring(config["sensors"]["tof_ring"], config["body"])


def test_fractional_zone_counts_are_not_silently_truncated():
    config = vehicle_config()["sensors"]["tof_ring"]
    config["profiles"]["low_latency_4x4_60"]["horizontal_zones"] = 4.5
    with pytest.raises(RuntimeError, match="4 or 8"):
        load_launch_module()._resolve_tof_profile(config, "low_latency_4x4_60")


def test_low_target_checker_rejects_ground_and_preserves_pointcloud_row_padding():
    path = REPO / "scripts/validate_tof_ring.py"
    spec = importlib.util.spec_from_file_location("arena_tof_validation", path)
    validator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(validator)
    fields = [SimpleNamespace(name=name, offset=offset) for name, offset in (("x", 0), ("y", 4), ("z", 8))]
    points = [(.18, 0., -.02), (.18, 0., -.04), (.5, 0., -.01), (math.inf, 0., 0.)]
    raw = b"".join(struct.pack(">fff", *point) + b"\x00" * 4 + b"\x00" * 8 for point in points)
    message = SimpleNamespace(fields=fields, is_bigendian=True, height=4, width=1,
                              point_step=16, row_step=24, data=raw)
    decoded = validator.xyz_points(message)
    assert decoded[0] == pytest.approx(points[0])
    assert decoded[2] == pytest.approx(points[2])
    hits = validator.target_hits(decoded, .18, .05, .04)
    assert len(hits) == 1
    assert hits[0] == pytest.approx(points[0])
