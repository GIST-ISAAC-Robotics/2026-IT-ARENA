import json
import math
from pathlib import Path
import struct
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src/arena_autonomy"))
from arena_autonomy.tof_safety_core import cloud_xyz, safe_speed, swept_clearance, to_body
from arena_autonomy.tof_safety import TofSafety


@pytest.mark.parametrize("big", [False, True])
def test_cloud_fields_endian_row_padding_and_invalid(big):
    order = ">" if big else "<"
    data = b"".join(struct.pack(order + "ffff", 99, x, 2, 3) + b"PAD!" for x in (1, math.nan))
    fields = [SimpleNamespace(name=n, datatype=7, count=1, offset=o) for n, o in (("x", 4), ("y", 8), ("z", 12))]
    cloud = SimpleNamespace(fields=fields, is_bigendian=big, width=1, height=2, point_step=16, row_step=20, data=data)
    assert cloud_xyz(cloud) == pytest.approx(np.array([[1, 2, 3]]))
    cloud.data = cloud.data[:-1]
    with pytest.raises(ValueError, match="잘린"):
        cloud_xyz(cloud)


def test_nominal_sensor_transform_and_ground_rejection():
    points = to_body(np.array([[.3, 0, -.04], [.4, 0, 0]]), [.098, 0, .04], [0, 0, 0])
    assert points[0, 2] == pytest.approx(0)
    assert .35 < swept_clearance(points, 0, 1) < .4
    assert math.isinf(swept_clearance(points[:1], 0, 1))
    rear = to_body(np.array([[.3, 0, 0]]), [-.098, 0, .04], [0, 0, math.pi])
    assert math.isinf(swept_clearance(rear, 0, 1))
    assert swept_clearance(rear, 0, -1) < .3


def test_swept_turn_sees_side_obstacle_without_stopping_for_distant_parallel_wall():
    point = [[.35, .22, .04]]
    assert math.isinf(swept_clearance(point, 0, 1))
    assert swept_clearance(point, .3, 1) < .5
    assert math.isinf(swept_clearance([[.2, .425, .05]], 0, 1))


def test_stopping_speed_bound_has_latency_and_braking():
    v = safe_speed(.45, .2, 2)
    assert v * .2 + v * v / 4 == pytest.approx(.45)
    assert v < 1.1
    assert safe_speed(-.1, .2, 2) == 0
    assert safe_speed(.45, .4, 2) < v


def gate():
    from ackermann_msgs.msg import AckermannDriveStamped
    from builtin_interfaces.msg import Time
    from arena_autonomy.tof_safety_core import SafetyGeometry
    request = AckermannDriveStamped()
    request.drive.speed = 5.5555556
    commands, statuses = [], []
    wall = time.monotonic()
    controller = SimpleNamespace(
        p={"sensor_timeout_s": .3, "encoder_timeout_s": .15, "command_timeout_s": .3,
           "reaction_time_s": .15, "assumed_braking_deceleration_mps2": 2.,
           "assumed_detection_distance_m": .5, "stop_margin_m": .05, "clear_hold_s": .8},
        modules=[{"name": n} for n in ("front", "fl", "rl", "rear", "rr", "fr")],
        clouds={n: (9.98, wall, np.array([[.3, 0, 0.]])) for n in ("front", "fl", "rl", "rear", "rr", "fr")},
        request=request, request_at=10., request_wall=wall, encoder_at=9.99, encoder_wall=wall,
        measured_speed=.35, steering_limit=.375, geometry=SafetyGeometry(),
        last_time=10., last_cloud_stamps={}, latched=False, clear_since=None, last_status="",
        now_s=lambda: 10., publisher=SimpleNamespace(publish=commands.append),
        status_publisher=SimpleNamespace(publish=statuses.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=10))),
        get_logger=lambda: SimpleNamespace(info=lambda *_: None))
    return controller, commands, statuses


def test_clear_space_still_respects_limited_low_target_detection_range():
    controller, commands, statuses = gate()
    TofSafety.control(controller)
    assert 0 < commands[-1].drive.speed < 1.1
    assert json.loads(statuses[-1].data)["state"] == "VISIBILITY_SPEED_LIMIT"


@pytest.mark.parametrize("failure", ["missing", "stale", "encoder", "command", "future", "transport"])
def test_fail_closed_without_fresh_inputs(failure):
    controller, commands, _ = gate()
    if failure == "missing": controller.clouds.pop("rear")
    elif failure == "stale": controller.clouds["front"] = (9., time.monotonic(), np.empty((0, 3)))
    elif failure == "future": controller.clouds["front"] = (11., time.monotonic(), np.empty((0, 3)))
    elif failure == "transport": controller.clouds["front"] = (9.99, time.monotonic() - 4, np.empty((0, 3)))
    elif failure == "encoder": controller.encoder_at = 9.
    else: controller.request_at = 9.
    TofSafety.control(controller)
    assert commands[-1].drive.speed == 0


def test_close_obstacle_latches_and_does_not_resume_on_one_clear_frame():
    controller, commands, statuses = gate()
    controller.clouds["front"] = (9.98, time.monotonic(), np.array([[.2, 0, .04]]))
    TofSafety.control(controller)
    assert commands[-1].drive.speed == 0 and controller.latched
    controller.clouds["front"] = (9.98, time.monotonic(), np.array([[.3, 0, 0.]]))
    controller.measured_speed = 0
    TofSafety.control(controller)
    assert commands[-1].drive.speed == 0
    assert json.loads(statuses[-1].data)["state"] == "OBSTACLE_STOP"


def test_direction_reversal_brakes_current_motion_first():
    controller, commands, statuses = gate()
    controller.request.drive.speed = -.5
    TofSafety.control(controller)
    assert commands[-1].drive.speed == 0
    assert json.loads(statuses[-1].data)["state"] == "REVERSAL_BRAKE"


def test_safety_node_never_uses_simulator_truth_or_map():
    text = (REPO / "src/arena_autonomy/arena_autonomy/tof_safety.py").read_text()
    for forbidden in ("/sim/", '"/odom"', "Odometry", "centerline.csv", "scene.json", "dynamic_pose"):
        assert forbidden not in text
