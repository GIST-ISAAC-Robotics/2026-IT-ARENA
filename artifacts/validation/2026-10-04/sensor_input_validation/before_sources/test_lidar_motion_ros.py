"""ROS 메시지 경계 검사. 네트워크나 실제 구동 명령을 사용하지 않는다."""
import json
import math
from types import SimpleNamespace

import pytest
from sensor_msgs.msg import Imu, JointState, LaserScan

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.lidar_motion_ros import MotionInput


class NodeDouble:
    def __init__(self, parameters=None):
        self.parameters = parameters or {}
        self.t = 0.
        self.subscriptions = []

    def declare_parameter(self, name, default):
        self.parameters.setdefault(name, default)
        return self.get_parameter(name)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def has_parameter(self, name):
        return name in self.parameters

    def get_clock(self):
        return SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=round(self.t*1e9)))

    def create_subscription(self, kind, topic, callback, qos):
        self.subscriptions.append(topic)


def stamp(msg, t):
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(round(t*1e9), 1_000_000_000)
    return msg


def wheel(t=0.):
    return stamp(JointState(name=['car::rear_right_wheel_joint', 'car::rear_left_wheel_joint'],
                            velocity=[60., 20.]), t)


def imu(t=0., z=1.):
    msg = stamp(Imu(), t)
    msg.angular_velocity.z = z
    return msg


def test_only_wheel_and_gyro_subscriptions_and_named_wheels():
    node = NodeDouble()
    adapter = MotionInput(node)
    adapter.on_wheels(wheel())
    assert set(node.subscriptions)=={'/wheel_states', '/camera/imu'}
    assert adapter.history.wheels.values==[1.]


def test_orientation_and_acceleration_not_used():
    adapter = MotionInput(NodeDouble())
    msg = imu()
    msg.orientation.w = float('nan')
    msg.linear_acceleration.x = float('nan')
    adapter.on_imu(msg)
    assert adapter.history.gyro.values==[1.]


def test_nonfinite_and_missing_angular_velocity_are_rejected_and_recordable():
    rows = []
    adapter = MotionInput(NodeDouble(), record=rows.append)
    adapter.on_imu(imu(z=float('nan')))
    msg = imu()
    msg.angular_velocity_covariance[0] = -1.
    adapter.on_imu(msg)
    assert adapter.counts['rejected']==2
    assert not adapter.history.gyro.times
    json.dumps(rows, allow_nan=False)


def test_clock_reset_clears_both_streams():
    node = NodeDouble()
    node.t = 1.
    adapter = MotionInput(node)
    adapter.on_wheels(wheel(1.))
    adapter.on_imu(imu(1.))
    node.t = .1
    adapter.clock()
    assert adapter.counts['resets']==1
    assert not adapter.history.wheels.times and not adapter.history.gyro.times


def test_independent_imu_topic_can_replace_camera_imu():
    node = NodeDouble({'motion_imu_topic': '/imu/data'})
    MotionInput(node)
    assert '/imu/data' in node.subscriptions
    assert '/camera/imu' not in node.subscriptions


def test_future_time_and_duplicate_rejected():
    adapter = MotionInput(NodeDouble())
    adapter.on_imu(imu(1.))
    adapter.on_imu(imu())
    adapter.on_imu(imu())
    assert adapter.counts['gyro']==1
    assert adapter.counts['rejected']==2


def test_mounting_roll_projects_y_axis_to_yaw():
    adapter = MotionInput(NodeDouble({'motion_imu_roll_rad':math.pi/2}))
    msg = imu(z=0.)
    msg.angular_velocity.y = 2.
    adapter.on_imu(msg)
    assert adapter.history.gyro.values==[2.]


def test_unverified_physical_scan_cannot_enable_deskew():
    node = NodeDouble({'lidar_compensation':'both'})
    node.t = .2
    adapter = MotionInput(node)
    msg = stamp(LaserScan(ranges=[1., 2.], range_min=.05, range_max=12., time_increment=.01), .1)
    msg.header.frame_id = 'laser_frame'
    with pytest.raises(MotionUnavailable, match='unverified'):
        adapter.project(msg)
    msg.header.frame_id = 'wrong'
    with pytest.raises(MotionUnavailable, match='frame'):
        adapter.project(msg)


def test_invalid_mounting_rejected():
    with pytest.raises(ValueError):
        MotionInput(NodeDouble({'motion_imu_roll_rad':float('nan')}))


MOTOR = {'motion_feedback_mode': 'drive_motor_shaft', 'motion_gear_ratio': 15., 'motion_wheel_radius_m': .0325}


def motor(t=0., position=0., velocity=150., name='drive_motor_shaft'):
    return stamp(JointState(name=[name], position=[position], velocity=[velocity]), t)


def test_legacy_default_is_explicit_rear_pair_for_old_parameters():
    adapter = MotionInput(NodeDouble())
    assert adapter.feedback_mode == 'rear_wheel_pair_legacy' and adapter.feedback_topic == '/wheel_states'


def test_motor_shaft_mode_subscribes_only_single_channel_and_divides_by_ratio():
    node = NodeDouble(dict(MOTOR))
    adapter = MotionInput(node)
    assert set(node.subscriptions) == {'/drive_motor/encoder', '/camera/imu'}
    adapter.on_drive_motor(motor(velocity=150.))
    # 150 rad/s 모터 / 15 = 평균 바퀴 10 rad/s, 반지름 0.0325 m.
    assert adapter.history.wheels.values == pytest.approx([.325])
    assert adapter.counts['drive_motor'] == 1 and adapter.counts['wheels'] == 0


def test_motor_shaft_mode_rejects_wheel_pair_names_and_bad_values():
    rows = []
    adapter = MotionInput(NodeDouble(dict(MOTOR)), record=rows.append)
    adapter.on_drive_motor(wheel())
    adapter.on_drive_motor(motor(name='rear_left_wheel_joint'))
    adapter.on_drive_motor(motor(t=.01, velocity=float('nan')))   # 첫 표본: 속도 미관측
    adapter.on_drive_motor(motor(t=.02, position=float('nan')))   # 누적 위치 손상
    adapter.on_drive_motor(motor(t=.03, velocity=float('inf')))
    assert adapter.counts['rejected'] == 5 and not adapter.history.wheels.times
    assert [row['kind'] for row in rows] == ['drive_motor'] * 3
    json.dumps(rows, allow_nan=False)


def test_motor_shaft_stamp_contract_keeps_capture_time_and_rejects_future_or_duplicate():
    node = NodeDouble(dict(MOTOR))
    node.t = 1.
    adapter = MotionInput(node)
    adapter.on_drive_motor(motor(t=.98))
    adapter.on_drive_motor(motor(t=.98))
    adapter.on_drive_motor(motor(t=1.2))
    assert adapter.history.wheels.times == pytest.approx([.98])
    assert adapter.counts['drive_motor'] == 1 and adapter.counts['rejected'] == 2
    node.t = .5
    adapter.clock()
    assert not adapter.history.wheels.times and adapter.counts['resets'] == 1


@pytest.mark.parametrize('speed_mps', [5.01, -5.01, 1000.])
def test_motor_finite_speed_spike_is_rejected_without_refreshing_history(speed_mps):
    node = NodeDouble(dict(MOTOR))
    adapter = MotionInput(node)
    adapter.on_drive_motor(motor(t=0., velocity=150.))
    node.t = .01
    adapter.on_drive_motor(motor(t=.01, velocity=speed_mps / .0325 * 15.))
    assert adapter.history.wheels.times == [0.]
    assert adapter.counts['rejected'] == 1


@pytest.mark.parametrize('value', [0., -1., float('nan'), float('inf')])
def test_invalid_motor_speed_sanity_bound_is_rejected(value):
    with pytest.raises(ValueError):
        MotionInput(NodeDouble({**MOTOR, 'motion_max_motor_feedback_mps': value}))


def test_legacy_mode_does_not_read_motor_channel():
    adapter = MotionInput(NodeDouble())
    adapter.on_wheels(motor())
    assert adapter.counts['rejected'] == 1 and not adapter.history.wheels.times


@pytest.mark.parametrize('overrides', [{'motion_feedback_mode': 'configured'},
                                       {'motion_feedback_mode': 'drive_motor_shaft'},
                                       {'motion_feedback_mode': 'drive_motor_shaft', 'motion_gear_ratio': float('nan')}])
def test_invalid_feedback_mode_or_missing_ratio_rejected(overrides):
    with pytest.raises(ValueError):
        MotionInput(NodeDouble(dict(overrides)))
