import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import pytest
import xacro
import yaml

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src/arena_autonomy"))
from arena_autonomy.core import StartSignal, estimate_wall, follow_command, marker_ids, scan_points
from arena_autonomy import wall_follow


def model(**mappings):
    return ET.fromstring(xacro.process_file(str(REPO / "src/arena_description/models/arena_car/model.sdf.xacro"),
                                           mappings=mappings).toxml())


def test_depth_can_be_disabled_without_removing_rgb_lidar_or_imu():
    assert model().find(".//sensor[@name='d435i_depth']") is not None
    rgb_only = model(d435i_depth_enabled="false")
    assert rgb_only.find(".//sensor[@name='d435i_depth']") is None
    for name in ("d435i_color", "d435i_imu", "rplidar_c1"):
        assert rgb_only.find(f".//sensor[@name='{name}']") is not None
    bridge = yaml.safe_load((REPO / "src/arena_bringup/config/d435i_rgb_imu_bridge.yaml").read_text())
    assert {entry['topic_name'] for entry in bridge} == {
        "/camera/color/image_raw", "/camera/color/camera_info", "/camera/imu"}


def test_c1_nominal_scan_and_provisional_mount():
    config = yaml.safe_load((REPO / "src/arena_description/config/vehicle.yaml").read_text())["vehicle"]
    lidar = config["sensors"]["lidar_2d"]
    assert lidar["enabled"] and "provisional" in lidar["status"]
    root = model()
    link = root.find(".//link[@name='lidar_link']")
    sensor = link.find("sensor")
    horizontal = sensor.find("lidar/scan/horizontal")
    samples = int(horizontal.findtext("samples"))
    assert samples == lidar["samples_per_scan"] == 500
    assert float(sensor.findtext("update_rate")) == lidar["scan_rate_hz"] == 10
    step = (float(horizontal.findtext("max_angle")) - float(horizontal.findtext("min_angle"))) / (samples - 1)
    assert math.degrees(step) == pytest.approx(.72)
    assert step * samples == pytest.approx(2 * math.pi)
    assert float(sensor.findtext("lidar/range/min")) == .05
    assert float(sensor.findtext("lidar/range/max")) == 12
    assert float(sensor.findtext("lidar/range/resolution")) == .015
    z = float(link.findtext("pose").split()[2])
    assert z > config["drivetrain"]["wheel_radius_m"] + config["body"]["height_m"]
    for visual in link.findall("visual"):
        offset = float(visual.findtext("pose").split()[2])
        height = float(visual.findtext("geometry/cylinder/length"))
        assert offset + height / 2 < 0  # 자기 몸체가 스캔 평면을 막지 않음


def test_speed_limits_remain_without_oscillating_jerk_clamp():
    plugin = model(drive_mode="legacy_velocity", max_speed="2").find(".//plugin[@name='gz::sim::systems::AckermannSteering']")
    assert plugin.find("max_jerk") is None and plugin.find("min_jerk") is None
    assert float(plugin.findtext("max_velocity")) == 2
    assert float(plugin.findtext("max_acceleration")) == 4
    assert float(plugin.findtext("min_acceleration")) == -4


def signal_image(state):
    rgb = np.full((480, 848, 3), 100, np.uint8)
    cv2.rectangle(rgb, (250, 130), (390, 180), (3, 3, 3), -1)
    for name, x, color in [("red", 280, (255, 4, 1)), ("yellow", 320, (255, 180, 1)),
                           ("green", 360, (1, 255, 10))]:
        cv2.circle(rgb, (x, 155), 10, color if name == state else tuple(int(v * .08) for v in color), -1)
    return rgb


def test_green_without_observed_red_or_green_grass_does_not_start():
    signal = StartSignal()
    for _ in range(8):
        assert not signal.update(signal_image("green"))
    for _ in range(2):
        signal.update(signal_image("red"))
    grass = signal_image("yellow")
    grass[280:] = (0, 255, 0)
    for _ in range(8):
        assert not signal.update(grass)


def test_red_then_consecutive_green_latches_continuous_running():
    signal = StartSignal()
    for _ in range(2):
        assert not signal.update(signal_image("red"))
    assert not signal.update(signal_image("yellow"))
    assert not signal.update(signal_image("green"))
    assert not signal.update(signal_image("green"))
    assert signal.update(signal_image("green"))
    assert signal.update(np.zeros((480, 848, 3), np.uint8))  # 신호등을 지난 뒤에도 유지


def test_wall_fit_mirrors_and_recovers_from_small_pole_outlier():
    x = np.linspace(-.2, .65, 80)
    for side, sign in [("left", 1), ("right", -1)]:
        points = np.column_stack((x, sign * np.full(len(x), .425)))
        points = np.vstack((points, [[.1, sign * .25], [.11, sign * .25]]))
        estimate = estimate_wall(points, side)
        assert estimate.distance == pytest.approx(.425, abs=.01)
        speed, steering, _ = follow_command(points, side)
        assert speed > 0 and abs(steering) < .02
        points[:, 1] *= .7
        _, steering, _ = follow_command(points, side)
        assert steering * sign < 0  # 가까운 벽에서 떨어짐


def test_missing_wall_and_close_obstacle_stop():
    assert follow_command(np.empty((0, 2)), "left")[0] == 0
    x = np.linspace(-.2, .65, 80)
    points = np.vstack((np.column_stack((x, np.full(len(x), .425))), [[.12, 0]]))
    speed, _, detail = follow_command(points, "left")
    assert speed == 0 and detail["reason"] == "obstacle_stop"


def test_scan_invalid_values_are_not_obstacles_or_free_space_samples():
    points, count = scan_points([math.nan, math.inf, 0, .01, .5, 20], -1, .2, .05, 12)
    assert count == 1 and np.all(np.isfinite(points))


def test_runtime_autonomy_has_no_ground_truth_or_map_input():
    source = (REPO / "src/arena_autonomy/arena_autonomy/wall_follow.py").read_text()
    for forbidden in ("Odometry", '"/odom"', '"/tf"', "centerline.csv", "scene.json", "dynamic_pose", "traffic_light/state"):
        assert forbidden not in source


def test_aruco_gate_id_is_read_from_pixels():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    make = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
    gray = np.full((200, 200), 255, np.uint8)
    gray[40:160, 40:160] = make(dictionary, 20, 120)
    assert marker_ids(cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)) == [20]


def controller_double(monkeypatch):
    """ROS 네트워크에 명령을 보내지 않고 타이머의 정지 판단만 검사합니다."""
    from builtin_interfaces.msg import Time
    from sensor_msgs.msg import LaserScan
    commands, statuses = [], []
    stamp = Time(sec=10)
    scan = LaserScan(angle_min=-math.pi, angle_increment=2 * math.pi / 500,
                     range_min=.05, range_max=12., ranges=[.5] * 500)
    scan.header.stamp = stamp
    signal = StartSignal()
    signal.started = True
    signal.observed = "green"
    controller = SimpleNamespace(
        settings={"control_rate_hz": 20., "scan_timeout_s": .45, "image_timeout_s": 1.,
                  "lidar_x_m": -.03, "target_wall_distance_m": .425, "wheelbase_m": .145,
                  "max_steering_angle_rad": .45, "max_speed_mps": .35, "min_speed_mps": .14,
                  "lateral_acceleration_limit_mps2": 3.0, "acceleration_mps2": .5},
        signal=signal, scan=scan, rgb_stamp=10., scan_wall_time=time.monotonic(),
        image_wall_time=time.monotonic(), enabled=True, side="left", ids=[],
        last_steering=0., last_speed=.2, last_status="", last_status_time=-math.inf,
        last_control_time=-math.inf, now_s=lambda: 10.,
        publisher=SimpleNamespace(publish=commands.append),
        status_publisher=SimpleNamespace(publish=statuses.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: stamp)),
        get_logger=lambda: SimpleNamespace(info=lambda *_: None))
    monkeypatch.setattr(wall_follow, "follow_command", lambda *_: (.3, .1, {"reason": "following"}))
    return controller, commands, statuses


@pytest.mark.parametrize("failure", ["stale_scan", "stale_image", "stalled_transport", "disabled", "no_green"])
def test_controller_stops_without_fresh_inputs_and_permission(monkeypatch, failure):
    import json
    controller, commands, statuses = controller_double(monkeypatch)
    wall_follow.WallFollow.control(controller)
    assert commands[-1].drive.speed > 0
    if failure == "stale_scan":
        controller.scan.header.stamp.sec = 8
    elif failure == "stale_image":
        controller.rgb_stamp = 8.
    elif failure == "stalled_transport":
        controller.scan_wall_time = time.monotonic() - 4
    elif failure == "disabled":
        controller.enabled = False
    else:
        controller.signal.started = False
    wall_follow.WallFollow.control(controller)
    assert commands[-1].drive.speed == 0
    expected = "DISABLED" if failure == "disabled" else "WAIT_GREEN" if failure == "no_green" else "SENSOR_STOP"
    assert json.loads(statuses[-1].data)["state"] == expected


def test_late_image_is_ignored_without_clearing_start_permission():
    from sensor_msgs.msg import Image
    from builtin_interfaces.msg import Time
    signal = StartSignal()
    signal.started = True
    controller = SimpleNamespace(last_image_processed=10., signal=signal, side="right")
    late = Image()
    late.header.stamp = Time(sec=9)
    wall_follow.WallFollow.on_image(controller, late)
    assert signal.started and controller.side == "right" and controller.last_image_processed == 10.


def test_validator_forwards_shutdown_without_inherited_terminal(monkeypatch):
    monkeypatch.syspath_prepend(str(REPO / "scripts"))
    import validate_basic_autonomy as validator
    calls = []
    monkeypatch.setattr(validator.subprocess, "Popen", lambda *args, **kwargs: calls.append((args, kwargs)))
    log = object()
    validator.start_demo_process(log)
    args, kwargs = calls[0]
    assert args[0][:3] == ["ros2", "launch", "--noninteractive"]
    assert kwargs["stdin"] == validator.subprocess.DEVNULL
    assert kwargs["start_new_session"] is True
    assert kwargs["stdout"] is log and kwargs["stderr"] is log


@pytest.mark.parametrize(
    ("light", "drive_speed", "autonomy", "expected"),
    [
        ("red", 0.0, {"started": True, "speed_command_mps": 0.0}, True),
        ("yellow", 0.02, {"started": False, "speed_command_mps": 0.0}, True),
        ("unknown", 0.0, {"started": False, "speed_command_mps": 0.03}, True),
        ("red", 0.0, {"started": False, "speed_command_mps": 0.0}, False),
        ("green", 0.20, {"started": True, "speed_command_mps": 0.20}, False),
    ],
)
def test_validator_detects_early_permission_and_speed_independent_of_run_state(
        monkeypatch, light, drive_speed, autonomy, expected):
    monkeypatch.syspath_prepend(str(REPO / "scripts"))
    import validate_basic_autonomy as validator
    assert validator.early_start_detected(light, drive_speed, autonomy) is expected
