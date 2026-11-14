"""gyro/accel 시각 분리와 실측 servo count 관측. 센서 물리 교정 시험 아님."""
import copy
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_vehicle_interface'), str(ROOT/'tests')]
from arena_vehicle_interface.imu_fifo_clock import ImuFifoClock
from test_imu_fifo_clock import stream, convert
from test_uno_q_host import make_owner, ticks, profile
from uno_q_observations import SeparateAccelClock, steering_observation, bind_observations
from uno_q_observations import EncoderObservationClock
from arena_vehicle_interface.serial_ingress import SerialReceipt


def test_accel_uses_own_fifo_rows_and_does_not_change_gyro_result_or_input():
    rows = stream()
    for row in rows:
        if row.get('sensor_tag') == 2:row.update(accel_mps2=[1., 2., 9.8], raw_xyz=[418, 836, 4095])
    before = copy.deepcopy(rows)
    expected = convert(ImuFifoClock(), rows)
    accel = []
    clock = SeparateAccelClock(ImuFifoClock(), lambda stamp, meta: accel.append((stamp, meta)))
    actual = convert(clock, rows)
    assert actual == expected and rows == before and len(accel) == len(expected) == 500
    assert all(stamp.sample['sensor_tag'] == 2 and 'gyro_rad_s' not in stamp.sample for stamp, _ in accel)
    assert all(meta['boot'] == stamp.sample['boot'] and meta['sequence'] == stamp.sample['sequence']
               and meta['sensor_tick'] == stamp.sensor_tick and not meta['time_contract_verified']
               for stamp, meta in accel)
    assert accel[0][0].sample['sequence'] != expected[0].sample['sequence']


def test_accel_clock_rejects_unknown_tag_and_rewarms_after_boot_change():
    for tag in (0, 3, True):
        with pytest.raises(ValueError):ImuFifoClock(sensor_tag=tag)
    clock = ImuFifoClock(sensor_tag=2)
    rows = stream()
    for row in rows:
        if row.get('sensor_tag') == 2:row['accel_mps2'] = [0., 0., 9.8]
    assert convert(clock, rows)
    assert convert(clock, stream(100, boot=8)) == [] and clock.fit is None


def test_steering_count_sign_and_receipt_age_not_invented_wheel_angle_measurement():
    owner, t, m, u, _ = make_owner();ticks(owner, t, 6)
    result = steering_observation(owner.servo, profile().zero, t.now)
    assert result['valid'] and result['position_count'] == 2824
    assert result['central_angle_rad'] > 0 and result['steering_sign_verified']
    assert result['sign_reference'] == 'measured_count_increase_is_left_2026_11_14'
    assert result['receipt_realtime_ns'] == owner.servo.observed_realtime_ns
    assert result['age_ns'] == t.now-owner.servo.observed_ns
    assert not result['hardware_angle_measured'] and not result['time_contract_verified']
    assert not steering_observation(owner.servo, profile().zero, t.now+200_000_000)['valid']
    assert not steering_observation(owner.servo, None, t.now)['steering_sign_verified']


def test_ros_accel_message_marks_other_channels_unavailable():
    class Imu:
        def __init__(self):
            self.header = NS(frame_id='', stamp=NS(sec=0, nanosec=0))
            self.orientation_covariance = [0.]*9;self.angular_velocity_covariance = [0.]*9
            self.linear_acceleration = NS(x=0., y=0., z=0.)
    class Node:
        def __init__(self):self.pubs = {}
        def create_publisher(self, cls, topic, qos):
            values = [];self.pubs[topic] = values
            return NS(publish=values.append)
    node = Node();owner = NS()
    bind_observations(node, owner, Imu=Imu, String=lambda **kw:NS(**kw), sensor_qos=1)
    owner.publish_accel(NS(stamp_ns=1234567890, sample=dict(accel_mps2=[1., 2., 3.])), dict(boot=1, sequence=2))
    msg = node.pubs['/hardware_runtime/accel'][0]
    assert msg.header.stamp.sec == 1 and msg.header.stamp.nanosec == 234567890
    assert msg.orientation_covariance[0] == msg.angular_velocity_covariance[0] == -1
    assert (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z) == (1., 2., 3.)


def test_raw_encoder_stationarity_does_not_fabricate_speed_or_refresh_duplicates():
    clock = EncoderObservationClock()
    row = dict(boot='1234abcd', generation=2, observation_seq=10, capture_us=100000,
               motor_counts=-42, raw_count=65500, hal_fault=False)
    receipt = SerialReceipt('rx', b'', 1_000_000_000, 2_000_000_000)
    first = clock.convert(row, receipt)
    assert first['valid'] and first['motor_counts'] == -42 and first['observation_seq'] == 10
    assert first['capture_us'] == 100000 and not first['encoder_sign_verified']
    assert first['stationary_only'] and not first['wheel_speed_inferred'] and 'velocity' not in first
    later = SerialReceipt('rx', b'', 1_020_000_000, 2_020_000_000)
    assert clock.convert(row, later) is None
    assert clock.convert(row|dict(capture_us=120000), later) is None
    next_row = clock.convert(row|dict(capture_us=120000, observation_seq=11), later)
    assert next_row['valid'] and next_row['stamp_ns'] > first['stamp_ns']
    assert not clock.convert(row|dict(hal_fault=True), later)['valid']
    assert not clock.convert(row|dict(observation_seq=-1, capture_us=0), later)['valid']
