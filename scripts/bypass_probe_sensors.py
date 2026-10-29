"""G2 이상 기구 평가기의 가상 센서 생산자. 실제 센서/MCU 드라이버가 아니다."""
import math
import numpy as np
from arena_vehicle_interface.drive_feedback import MotorEncoderSpec

SPEC=MotorEncoderSpec(28,15.,.0325)


class ProbeSensors:
    def __init__(self,profile='ideal',seed=17):
        if profile not in ('ideal','range','motion','combined'):
            raise ValueError('unknown registered probe sensor profile')
        self.profile=profile
        self.range_enabled=profile in ('range','combined')
        self.motion_enabled=profile in ('motion','combined')
        self.range_rng=np.random.default_rng(seed)
        self.gyro_rng=np.random.default_rng(seed+1000)
        self.last_time=None;self.last_count=0

    @property
    def actual_lidar_yaw(self):
        return math.radians(-.5) if self.motion_enabled else 0.

    def range_value(self,distance):
        if not math.isfinite(distance):return distance
        return distance+(float(self.range_rng.uniform(-.03,.03)) if self.range_enabled else 0.)

    def measure_motion(self,now,travel,speed,steering):
        """생산자만 실제 이동을 카운트로 만든다. 제어기에는 평균 속도/gyro만 전달."""
        count=round(travel/SPEC.mean_wheel_travel_m_per_count)
        if self.last_time is None:mean=0.
        else:
            if now<=self.last_time:raise ValueError('capture time must advance')
            rate=SPEC.motor_rad_s_from_counts(count-self.last_count,now-self.last_time)
            mean=SPEC.mean_wheel_rad_s(rate)
        self.last_time,self.last_count=now,count
        gyro=speed*math.tan(steering)/.135
        if self.motion_enabled:
            gyro+=math.radians(1.)+float(self.gyro_rng.normal(0.,.02))
        return (mean if self.motion_enabled else speed/.0325),gyro,count

    def description(self):
        return dict(profile=self.profile,
            range_uniform_half_width_m=.03 if self.range_enabled else 0.,
            physical_lidar_yaw_bias_rad=self.actual_lidar_yaw,
            gyro_bias_rad_s=math.radians(1.) if self.motion_enabled else 0.,
            gyro_gaussian_std_rad_s=.02 if self.motion_enabled else 0.,
            motor_count_quantization=self.motion_enabled,
            counts_per_motor_revolution=28,gear_ratio=15.,wheel_radius_m=.0325,
            capture_interval_s=.02,measured_distribution=False,
            sensor_latency_transport_and_slip_verified=False)
