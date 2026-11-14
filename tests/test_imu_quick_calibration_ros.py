"""실제 ROS 메시지 + fake node. rclpy init/네트워크/장치 실행은 하지 않는다."""
import json
from types import SimpleNamespace

import numpy as np
import pytest
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String

from arena_autonomy.lidar_motion_ros import MotionInput


class Node:
    def __init__(self, **parameters):
        self.parameters = {'motion_quick_calibration': True, 'motion_imu_frame_id': 'lsm6dsox_sensor', **parameters}
        self.t = 0.
        self.subscriptions, self.publications = {}, {}

    def declare_parameter(self, key, default):
        self.parameters.setdefault(key, default)
        return self.get_parameter(key)

    def get_parameter(self, key):
        return SimpleNamespace(value=self.parameters[key])

    def has_parameter(self, key):
        return key in self.parameters

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(self.t*1e9)))

    def create_subscription(self, kind, topic, callback, qos):
        self.subscriptions[topic] = callback
        return SimpleNamespace(callback=callback)

    def create_publisher(self, kind, topic, qos):
        self.publications[topic] = []
        return SimpleNamespace(publish=self.publications[topic].append)


def stamp(message, t):
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(round(t*1e9), 1_000_000_000)
    message.header.frame_id = 'lsm6dsox_sensor'
    return message


def sample(node, adapter, index, *, state='WAIT_GREEN', speed=0., accel=True, frame='lsm6dsox_sensor'):
    node.t = index*.02
    adapter.on_calibration_state(String(data=json.dumps(dict(state=state, sim_time_s=node.t))))
    adapter.on_calibration_command(stamp(AckermannDriveStamped(), node.t))
    adapter.on_wheels(stamp(JointState(name=['rear_left_wheel_joint', 'rear_right_wheel_joint'],
                                     velocity=[speed/.025]*2), node.t))
    if accel:
        message = stamp(Imu(), node.t)
        message.header.frame_id = frame
        message.angular_velocity_covariance[0] = -1.
        message.linear_acceleration.x = 9.80665  # 센서 x가 차량 위쪽인 장착
        adapter.on_calibration_accel(message)
    message = stamp(Imu(), node.t)
    message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = .04, .015, -.01
    message.linear_acceleration_covariance[0] = -1.
    adapter.on_imu(message)


def test_opt_in_subscriptions_publish_diagnostics_but_never_commands():
    node = Node()
    MotionInput(node)
    assert set(node.subscriptions) == {'/wheel_states', '/camera/imu', '/autonomy/status', '/drive/safe',
                                       '/hardware_runtime/encoder_observation',
                                       '/hardware_runtime/accel', '/hardware_runtime/steering_observation'}
    assert set(node.publications) == {'~/imu_calibration/status'}


def test_stable_wait_commits_bias_and_projection_to_real_history_once():
    node = Node(motion_gyro_bias_rad_s=.5)
    adapter = MotionInput(node)
    for i in range(1, 92):
        sample(node, adapter, i)
    assert adapter.calibration.version == 1
    np.testing.assert_allclose(adapter.axis, [1., 0., 0.])
    assert adapter.history.config.gyro_bias_rad_s == pytest.approx(.04)
    assert abs(adapter.history.gyro.values[-1]) < 1e-12
    assert adapter.history.gyro.times[0] >= 1.5
    assert adapter.history.wheels.times[0] >= 1.5
    node.t += .02
    message = stamp(Imu(), node.t)
    message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = .74, .015, -.01
    adapter.on_calibration_state(String(data=json.dumps(dict(state='FOLLOW', sim_time_s=node.t))))
    adapter.on_imu(message)
    assert adapter.history.gyro.values[-1] == pytest.approx(.7)
    report = json.loads(node.publications['~/imu_calibration/status'][-1].data)
    assert report['version'] == 1
    assert not report['physical_calibration_verified']


@pytest.mark.parametrize('kwargs', [{'accel': False}, {'frame': 'other_sensor'}, {'speed': .2}, {'state': 'FOLLOW'}])
def test_bad_or_missing_calibration_inputs_keep_legacy_mapping(kwargs):
    node = Node()
    adapter = MotionInput(node)
    for i in range(1, 92):
        sample(node, adapter, i, **kwargs)
    assert adapter.calibration.version == 0
    assert adapter.axis == (0., 0., 1.)
    assert adapter.history.gyro.values[-1] == -.01


def test_boundary_resets_candidate_and_rejects_preboundary_accel_and_command():
    node = Node()
    adapter = MotionInput(node)
    for i in range(1, 40):
        sample(node, adapter, i)
    adapter.begin_input_boundary(node.t)
    adapter.on_calibration_accel(stamp(Imu(), .1))
    adapter.on_calibration_command(stamp(AckermannDriveStamped(), .1))
    assert adapter.calibration.accel is None and adapter.calibration.command is None
    assert adapter.calibration.version == 0


def test_ros_clock_rewind_cannot_reuse_stale_candidate_or_state():
    node = Node()
    adapter = MotionInput(node)
    for i in range(1, 40):
        sample(node, adapter, i)
    node.t = .1
    adapter.clock()
    assert adapter.calibration.state is None and not adapter.calibration.window
    assert not adapter.history.gyro.times


def test_verified_steering_metadata_freshness_and_sign_reference():
    node = Node()
    adapter = MotionInput(node)
    node.t = 1.
    value = dict(schema='uno_q_steering_observation_v1', valid=True, central_angle_rad=.2,
                 receipt_realtime_ns=1_000_000_000, age_ns=0, steering_sign_verified=True,
                 sign_reference='measured_count_increase_is_left_2026_11_14', hardware_angle_measured=False)
    adapter.on_calibration_steering(String(data=json.dumps(value)))
    assert adapter.calibration.steering == (1., .2, True)
    value['age_ns'] = 900_000_000
    adapter.on_calibration_steering(String(data=json.dumps(value)))
    assert adapter.calibration.steering is None


def test_combined_imu_topic_uses_valid_acceleration_covariance():
    node = Node(motion_calibration_accel_topic='/camera/imu')
    adapter = MotionInput(node)
    assert '/hardware_runtime/accel' not in node.subscriptions
    for i in range(1, 92):
        node.t = i*.02
        adapter.on_calibration_state(String(data=json.dumps(dict(state='WAIT_GREEN', sim_time_s=node.t))))
        adapter.on_calibration_command(stamp(AckermannDriveStamped(), node.t))
        adapter.on_wheels(stamp(JointState(name=['rear_left_wheel_joint', 'rear_right_wheel_joint'],
                                         velocity=[0., 0.]), node.t))
        message = stamp(Imu(), node.t)
        message.linear_acceleration.y = 9.80665
        message.angular_velocity.y = .03
        adapter.on_imu(message)
    np.testing.assert_allclose(adapter.axis, [0., 1., 0.])
    assert adapter.history.config.gyro_bias_rad_s == pytest.approx(.03)


def test_motor_feedback_sign_applies_only_to_history_not_motor_controller():
    node = Node(motion_feedback_mode='drive_motor_shaft', motion_gear_ratio=10.)
    adapter = MotionInput(node)
    adapter.calibration.encoder_sign = -1
    node.t = 1.
    message = stamp(JointState(name=['drive_motor_shaft'], position=[10.], velocity=[-100.]), node.t)
    adapter.on_drive_motor(message)
    assert adapter.history.wheels.values == [.25]
    assert adapter.calibration.encoder[1] == -.25
    assert adapter.history.config.wheel_scale == 1.
    assert not adapter.calibration.snapshot()['motor_controller_polarity_updated']


def test_raw_count_observation_can_calibrate_without_inventing_motion_history():
    node, adapter = Node(), None
    adapter = MotionInput(node)
    for i in range(1, 105):
        node.t = i*.02
        adapter.on_calibration_state(String(data=json.dumps(dict(state='WAIT_GREEN', sim_time_s=node.t))))
        adapter.on_calibration_command(stamp(AckermannDriveStamped(), node.t))
        adapter.on_calibration_raw_encoder(String(data=json.dumps(dict(
            schema='uno_q_encoder_observation_v1', valid=True, motor_counts=50, boot='abcd1234',
            observation_seq=i, capture_us=i*20_000, receipt_realtime_ns=round(node.t*1e9), age_ns=0))))
        message = stamp(Imu(), node.t)
        message.linear_acceleration.z = 9.80665
        adapter.on_calibration_accel(message)
        message.angular_velocity.z = .02
        adapter.on_imu(message)
    assert adapter.calibration.version == 1
    assert not adapter.history.wheels.times
    assert adapter.history.config.gyro_bias_rad_s == pytest.approx(.02)


def test_old_raw_capture_cannot_be_refreshed_by_new_host_receipt():
    node = Node()
    adapter = MotionInput(node)
    node.t = 10.
    adapter.on_calibration_raw_encoder(String(data=json.dumps(dict(
        schema='uno_q_encoder_observation_v1', valid=True, motor_counts=50, boot='abcd1234',
        observation_seq=1, capture_us=1_000_000, receipt_realtime_ns=10_000_000_000,
        stamp_ns=1_000_000_000, age_ns=0))))
    assert adapter.calibration.raw_encoder is None
    assert adapter.calibration.reason == 'raw_encoder_invalid'
