"""UNO Q 원시 accel/실측 서보 count 관측. 제어·영점 이동·자동 교정을 하지 않는다."""
import math
from arena_vehicle_interface.imu_fifo_clock import ImuFifoClock
from arena_vehicle_interface.imu_read_clock import ImuReadClock
from arena_vehicle_interface.steering_calibration import CadMapping, check_profile

ACCEL_TOPIC = '/hardware_runtime/accel'
ACCEL_META_TOPIC = '/hardware_runtime/accel_meta'
STEERING_TOPIC = '/hardware_runtime/steering_observation'
ENCODER_TOPIC = '/hardware_runtime/encoder_observation'


class EncoderObservationClock:
    """원 count의 정지후보 관측. 설치 부호/바퀴속도를 만들지 않는다."""
    def __init__(self):
        self.clock = ImuReadClock(warmup_ns=0)
        self.last_boot = self.last_seq = None

    def convert(self, row, receipt):
        seq, capture, count = (row.get(k) for k in ('observation_seq', 'capture_us', 'motor_counts'))
        valid = (type(seq) is int and seq >= 0 and type(capture) is int and capture > 0
                 and type(count) is int and row.get('hal_fault') is False)
        result = dict(schema='uno_q_encoder_observation_v1', valid=False, boot=row['boot'],
            generation=row['generation'], observation_seq=seq, capture_us=capture, motor_counts=count,
            raw_count=row.get('raw_count'), receipt_realtime_ns=receipt.realtime_ns,
            receipt_monotonic_ns=receipt.monotonic_ns, age_ns=None, stamp_ns=None,
            time_basis='minimum_receipt_offset_mcu_read_estimate', time_contract_verified=False,
            encoder_sign_verified=False, wheel_speed_inferred=False, stationary_only=True)
        if not valid:return result
        if self.last_boot == row['boot'] and seq <= self.last_seq:return None
        sample = dict(boot=int(row['boot'], 16), sequence=seq, read_begin_us=capture, read_end_us=capture)
        stamps = self.clock.convert_batch([sample], host_monotonic_ns=receipt.monotonic_ns,
                                          ros_receipt_ns=receipt.realtime_ns)
        if not stamps:return None  # duplicate/old sample never receives a new freshness timestamp
        stamp = stamps[0]
        self.last_boot, self.last_seq = row['boot'], seq
        result.update(valid=True, age_ns=stamp.receipt_age_ns, stamp_ns=stamp.stamp_ns,
                      clock_epoch=stamp.generation)
        return result


class SeparateAccelClock:
    """기존 gyro clock 결과를 그대로 반환하고 tag2만 별도 인스턴스에서 대응한다."""
    def __init__(self, gyro, publish):
        self.gyro, self.accel, self.publish = gyro, ImuFifoClock(sensor_tag=2), publish

    def __getattr__(self, name):return getattr(self.gyro, name)

    def convert_batch(self, frames, *, host_monotonic_ns, ros_receipt_ns):
        frames = tuple(frames)
        stamps = self.accel.convert_batch(frames, host_monotonic_ns=host_monotonic_ns,
                                         ros_receipt_ns=ros_receipt_ns)
        for stamp in stamps:
            row = stamp.sample
            meta = dict(schema='uno_q_accel_observation_v1', boot=row['boot'], sequence=row['sequence'],
                sensor_tick=stamp.sensor_tick, read_begin_us=row['read_begin_us'], read_end_us=row['read_end_us'],
                stamp_ns=stamp.stamp_ns, generation=stamp.generation, receipt_monotonic_ns=host_monotonic_ns,
                receipt_realtime_ns=ros_receipt_ns, receipt_age_ns=stamp.receipt_age_ns,
                mcu_lower_ns=stamp.mcu_lower_ns, mcu_upper_ns=stamp.mcu_upper_ns,
                register_anchor_sequence=stamp.register_anchor_sequence,
                accel_mps2=list(row['accel_mps2']), raw_xyz=list(row['raw_xyz']),
                time_basis=self.accel.time_basis, time_contract_verified=False,
                sensor_acquisition_timestamp_verified=False, gyro_mixed_into_accel=False)
            self.publish(stamp, meta)
        return self.gyro.convert_batch(frames, host_monotonic_ns=host_monotonic_ns, ros_receipt_ns=ros_receipt_ns)


def steering_observation(servo, profile, now):
    row = dict(schema='uno_q_steering_observation_v1', valid=False, position_count=None,
        central_angle_rad=None, observed_monotonic_ns=servo.observed_ns,
        receipt_realtime_ns=servo.observed_realtime_ns,
        age_ns=None if servo.observed_ns is None else now-servo.observed_ns,
        steering_sign_verified=False, sign_reference=None, hardware_angle_measured=False,
        time_basis='host_USB_read_receipt_realtime', time_contract_verified=False,
        direction_profile_assembly_id=None, physical_output_verified=False)
    if profile is None or servo.observation is None or not servo.ready(now):return row
    try:
        check_profile(profile, ready=True)
        count = servo.observation['position_counts']
        if not profile['safe_min_count'] <= count <= profile['safe_max_count']:return row
        relative = (count-profile['zero_count'])*2*math.pi/(profile['count_direction_per_cad_positive']*4096)
        goal = CadMapping.default().relative(relative)
        if goal.clamped:return row
        # 이 sign은 장착 count 방향 관측이다. 실제 바퀴각/곡률 크기 검증과 구별한다.
        sign = profile['count_direction_per_cad_positive'] == -1
        current_evidence = profile.get('selection_basis', {}).get('selected_on') == '2026-11-14'
        row.update(valid=True, position_count=count, central_angle_rad=goal.equivalent_rad,
            steering_sign_verified=profile['direction_confirmed'],
            sign_reference=('measured_count_increase_is_left_2026_11_14' if sign and current_evidence
                            else 'selected_profile_count_increase_is_left' if sign
                            else 'selected_profile_count_increase_is_right'),
            direction_profile_assembly_id=profile['assembly_id'])
    except (KeyError, TypeError, ValueError, ArithmeticError):return row
    return row


def bind_observations(node, owner, *, Imu, String, sensor_qos):
    import json
    accel_pub = node.create_publisher(Imu, ACCEL_TOPIC, sensor_qos)
    accel_meta = node.create_publisher(String, ACCEL_META_TOPIC, sensor_qos)
    steering_pub = node.create_publisher(String, STEERING_TOPIC, sensor_qos)
    encoder_pub = node.create_publisher(String, ENCODER_TOPIC, sensor_qos)
    def publish_accel(stamp, meta):
        message = Imu();message.header.frame_id = 'lsm6dsox_sensor'
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp.stamp_ns, 1_000_000_000)
        message.orientation_covariance[0] = message.angular_velocity_covariance[0] = -1.
        message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = stamp.sample['accel_mps2']
        accel_pub.publish(message)
        accel_meta.publish(String(data=json.dumps(meta, allow_nan=False, separators=(',', ':'))))
    owner.publish_accel = publish_accel
    owner.publish_steering = lambda row: steering_pub.publish(String(data=json.dumps(row, allow_nan=False, separators=(',', ':'))))
    owner.publish_encoder = lambda row: encoder_pub.publish(String(data=json.dumps(row, allow_nan=False, separators=(',', ':'))))
    return accel_pub, accel_meta, steering_pub, encoder_pub
