"""실제 ROS 메시지 타입/CDR와 NodeDouble만 사용. init/spin/장치/발행 없음."""
from dataclasses import replace
import json
import math

import numpy as np
import pytest
from rclpy.serialization import serialize_message, deserialize_message
from std_msgs.msg import String

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.lidar_motion_ros import MotionInput
from arena_autonomy.native_lidar_input import decode_measurements
from arena_autonomy.scene_motion import motion_scene
from arena_vehicle_interface.c1_native_preview import C1NativePreview
from test_lidar_motion_ros import NodeDouble
from test_native_measurement_motion import scan
from test_runtime_lidar_frame import FRAME, POLICY


def node(**overrides):
    parameters = dict(motion_lidar_frame_id=FRAME, motion_scan_time_policy=POLICY,
        lidar_compensation='both', use_sim_time=False, motion_scan_timing_verified=False,
        motion_lidar_x_m=.06, motion_wheel_radius_m=.0325)
    parameters.update(overrides)
    result = NodeDouble(parameters)
    result.execution_environment = 'hardware_runtime_candidate_v1'
    result.t = 1.12
    return result


def fill(adapter):
    # 입력값을 만든 합성 이력이다. 실제 엔코더/IMU 또는 정지 측정이라고 하지 않는다.
    for index in range(81):
        stamp = .9+index*.005
        adapter.history.add_mean_wheel(stamp, 1./adapter.history.config.wheel_radius_m)
        adapter.history.add_gyro(stamp, .8)


def test_motion_input_declares_frame_as_string_and_default_still_rejects_c1():
    default = MotionInput(NodeDouble())
    assert default.history.config.lidar_frame_id == 'laser_frame'
    assert default.node.parameters['motion_lidar_frame_id'] == 'laser_frame'
    with pytest.raises(MotionUnavailable, match='scan_frame_mismatch'):
        default.project(scan(source_frame_id=FRAME, time_basis='unverified_estimate'))
    selected = MotionInput(node())
    assert selected.history.config.lidar_frame_id == FRAME
    assert selected.node.parameters['motion_lidar_frame_id'] == FRAME
    assert not selected.verified and selected.timing_policy == POLICY
    assert set(selected.node.subscriptions) == {'/wheel_states', '/camera/imu'}


@pytest.mark.parametrize('value', [None, False, 123, '', 'laser frame', 'other', ['laser_frame']])
def test_invalid_frame_parameter_fails_before_subscription(value):
    configured = node(motion_lidar_frame_id=value)
    with pytest.raises(ValueError, match='registered lidar source frame'):
        MotionInput(configured)
    assert configured.subscriptions == []


@pytest.mark.parametrize('fault', ['verified', 'sim_clock', 'default_environment', 'default_policy', 'mode'])
def test_c1_name_cannot_bypass_runtime_time_policy(fault):
    configured = node()
    if fault == 'verified': configured.parameters['motion_scan_timing_verified'] = True
    elif fault == 'sim_clock': configured.parameters['use_sim_time'] = True
    elif fault == 'default_environment': configured.execution_environment = 'default'
    elif fault == 'default_policy': configured.parameters['motion_scan_time_policy'] = 'verified_only'
    elif fault == 'mode': configured.parameters['lidar_compensation'] = 'none'
    with pytest.raises(ValueError): MotionInput(configured)
    assert not configured.subscriptions


def test_correct_frame_does_not_invent_missing_motion_history():
    adapter = MotionInput(node())
    raw = scan(source_frame_id=FRAME, time_basis='unverified_estimate')
    with pytest.raises(MotionUnavailable): adapter.project(raw)
    assert adapter.history.wheels.times == adapter.history.gyro.times == []
    assert adapter.counts['drive_motor'] == adapter.counts['wheels'] == adapter.counts['gyro'] == 0


def test_selected_frame_rejects_other_registered_source_without_rewriting_input():
    adapter = MotionInput(node()); fill(adapter)
    raw = scan(source_frame_id='laser_frame', time_basis='unverified_estimate')
    with pytest.raises(MotionUnavailable, match='scan_frame_mismatch'): adapter.project(raw)
    assert raw.source_frame_id == 'laser_frame' and raw.time_basis == 'unverified_estimate'
    assert adapter.last_meta['reason'] == 'scan_frame_mismatch'
    good = replace(raw, source_frame_id=FRAME)
    points, count = adapter.project(good)
    assert len(points) == count == len(good.ranges_m)
    assert adapter.last_meta['hardware_timing_verified'] is False


def test_actual_producer_codec_and_cdr_keep_frame_source_time_and_unverified_state():
    nodes = [[round(i*65536/500), 8000, 255, 0] for i in range(500)]
    nodes[20][1] = 0
    batch = dict(schema='c1_sdk_native_preview_v1', scan=1, begin_monotonic_ns=1_000_000_000,
        end_monotonic_ns=1_100_000_000, sdk_estimated_first_us=1_000_000,
        us_per_sample=200., nodes=nodes)
    producer = C1NativePreview(source_epoch=11)
    payload, diagnostic = producer.convert(batch, receipt_monotonic_ns=1_120_000_000,
                                           receipt_ros_ns=1_120_000_000)
    wire = json.dumps(payload, allow_nan=False)
    message = deserialize_message(serialize_message(String(data=wire)), String)
    assert message.data == wire
    raw = decode_measurements(message.data, expected_time_basis='unverified_estimate')
    assert raw.source_frame_id == payload['source_frame_id'] == FRAME
    assert raw.source_epoch == payload['source_epoch'] == 11
    assert raw.acquired_at_s == tuple(payload['acquired_at_s'])
    assert raw.angles_rad == tuple(payload['angles_rad'])
    assert raw.time_basis == payload['time_basis'] == 'unverified_estimate'
    assert diagnostic['time_contract_verified'] is False and diagnostic['sensor_acquisition_timestamp'] is False
    assert diagnostic['mounting_calibrated'] is False and diagnostic['drive_output'] is False
    adapter = MotionInput(node()); fill(adapter)
    points, count = adapter.project(raw)
    scene = motion_scene(raw, adapter.history, 1.12, timing_verified=adapter.verified, rear_half_angle_deg=0.)
    assert count == len(points) == 499
    assert scene.source_frame_id == FRAME and scene.source_epoch == 66
    assert tuple(ray.acquired_at_s for ray in scene.rays) == raw.acquired_at_s
    assert scene.rays[20].status == 'unknown' and scene.rays[20].hit_xy_m is None
    assert math.isinf(raw.ranges_m[20])
    assert adapter.verified is False and adapter.last_meta['hardware_timing_verified'] is False
    assert adapter.last_meta['time_basis'] == 'unverified_estimate'
    assert payload['time_basis'] == 'unverified_estimate' and json.dumps(payload, allow_nan=False) == wire
