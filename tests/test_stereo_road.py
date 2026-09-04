from pathlib import Path
import sys
import json
import math
import time
from types import SimpleNamespace

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src/arena_autonomy"))
from arena_autonomy.stereo_road import road_target, road_steering


def synthetic_road(offset=0):
    width, height = 848, 480
    k = (612.33, 617.16, 424., 240.)
    fx, fy, cx, cy = k
    v, u = np.meshgrid(np.arange(height), np.arange(width), indexing="ij")
    with np.errstate(divide="ignore"):
        depth = .075 * fy / (v-cy)
    depth[depth <= 0] = np.inf
    with np.errstate(invalid="ignore"):
        lateral = (cx-u) * depth / fx
    road = (lateral >= offset-.225) & (lateral <= offset+.225)
    rgb = np.full((height, width, 3), (60, 150, 60), dtype=np.uint8)
    rgb[road] = (88, 88, 90)
    return rgb, depth.astype(np.float32), k


@pytest.mark.parametrize("offset", [-.10, 0, .10])
def test_ground_road_target_and_steering_direction(offset):
    rgb, depth, k = synthetic_road(offset)
    target, detail = road_target(rgb, depth, k, k)
    assert target is not None
    assert target[1] == pytest.approx(offset, abs=.012)
    assert detail["road_near_samples"] >= 2
    steering = road_steering(target)
    assert abs(steering) < .015 if offset == 0 else steering * offset > 0


def test_gray_wall_is_not_ground_and_missing_depth_does_not_invent_road():
    rgb, depth, k = synthetic_road()
    depth[:] = .13
    assert road_target(rgb, depth, k, k)[0] is None
    depth[:] = np.nan
    assert road_target(rgb, depth, k, k)[0] is None


def test_colored_ground_is_not_automatically_a_road():
    rgb, depth, k = synthetic_road()
    rgb[:] = (60, 150, 60)
    target, detail = road_target(rgb, depth, k, k)
    assert target is None
    assert detail["road_reason"] == "insufficient_near_ground_road"


@pytest.mark.parametrize("offset", [-.18, .18])
def test_one_visible_boundary_uses_nominal_road_width(offset):
    rgb, depth, k = synthetic_road(offset)
    target, detail = road_target(rgb, depth, k, k)
    assert target is not None
    assert target[1] == pytest.approx(offset, abs=.012)
    assert detail["inferred_boundary_samples"] > 0


@pytest.mark.parametrize("target", [(0., .1), (math.nan, .1), (.3, math.inf)])
def test_invalid_target_is_rejected(target):
    with pytest.raises(ValueError):
        road_steering(target)


@pytest.mark.parametrize("failure", [None, "depth_stale", "rgb_stale", "transport_stale", "disabled", "no_green"])
def test_stereo_controller_uses_target_or_stops_on_bad_inputs(monkeypatch, failure):
    from builtin_interfaces.msg import Time
    from arena_autonomy import stereo_wall_follow as module
    commands, statuses = [], []
    settings = {
        "control_rate_hz": 20., "depth_timeout_s": .25, "image_timeout_s": 1.,
        "max_steering_angle_rad": .45, "max_speed_mps": .7, "min_speed_mps": .18,
        "wall_maximum_forward_m": 1.45, "gap_free_clearance_m": .55,
        "gap_row_half_height_px": 14, "depth_stride_px": 4,
        "lateral_clearance_hold_s": .8, "color_fx_px": 612., "color_fy_px": 617.,
        "color_cx_px": 424., "color_cy_px": 240., "camera_x_m": .055,
        "camera_y_m": 0., "camera_z_m": .075, "road_width_m": .45,
        "road_target_x_m": .34, "wheelbase_m": .145,
        "lateral_acceleration_limit_mps2": 3., "acceleration_mps2": .8,
    }
    stamp = Time(sec=10)
    control = SimpleNamespace(
        settings=settings, now_s=lambda:10., last_control_time=9., enabled=True,
        signal=SimpleNamespace(observed="green", started=True), side="left", ids=[],
        depth_valid_count=0, depth=np.ones((2,2)), depth_stamp=10., depth_wall_time=time.monotonic(),
        rgb_stamp=10., image_wall_time=time.monotonic(), rgb=np.zeros((2,2,3),np.uint8),
        depth_info=(446.,432.,424.,240.), points=np.empty((0,2)), last_speed=.2,
        last_steering=0., last_lateral_clearance_time=-math.inf,
        last_status_time=-math.inf, last_status="",
        publisher=SimpleNamespace(publish=commands.append),
        status_publisher=SimpleNamespace(publish=statuses.append),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:stamp)),
        get_logger=lambda:SimpleNamespace(info=lambda *_:None),
    )
    monkeypatch.setattr(module,"depth_gap_command",lambda *_a,**_k:(.5,-.4,{"reason":"depth_gap_following"}))
    monkeypatch.setattr(module,"depth_lateral_clearance",lambda *_a:None)
    monkeypatch.setattr(module,"road_target",lambda *_a:((.34,.05),{}))
    if failure == "depth_stale": control.depth_stamp=9.
    elif failure == "rgb_stale": control.rgb_stamp=8.
    elif failure == "transport_stale": control.depth_wall_time=time.monotonic()-4
    elif failure == "disabled": control.enabled=False
    elif failure == "no_green": control.signal.started=False
    module.StereoWallFollow.control(control)
    if failure is None:
        assert commands[-1].drive.speed > 0
        assert commands[-1].drive.steering_angle > 0  # 반대 방향 gap을 더하지 않음
        assert json.loads(statuses[-1].data)["steering_source"] == "rgbd_road_pure_pursuit"
    else:
        assert commands[-1].drive.speed == 0
