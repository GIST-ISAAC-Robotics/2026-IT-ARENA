"""WAIT 정지 후보에서 시작하는 임시 IMU 보정. 장치/ROS/구동 명령을 소유하지 않는다.

중력은 가속도 영점으로 제거하지 않고 센서 좌표의 위쪽 단위 벡터로 보존한다.
평면 yaw에는 전후축이 필요 없다. 전후축과 엔코더 부호는 별도 주행 근거가 있어야 한다.
"""
from dataclasses import dataclass
import math

import numpy as np


WAIT_STATES = frozenset(('WAIT', 'WAIT_GREEN', 'DISABLED', 'RACE_LATCHED', 'ACTUATOR_WAIT'))


@dataclass(frozen=True)
class QuickCalibrationConfig:
    stationary_duration_s: float = 1.5
    min_samples: int = 80
    input_max_age_s: float = .15
    state_max_age_s: float = .7
    sample_gap_s: float = .08
    stationary_speed_mps: float = .025
    zero_command_mps: float = .005
    gravity_mps2: float = 9.80665
    gravity_tolerance_mps2: float = .5
    accel_std_limit_mps2: float = .12
    gyro_std_limit_rad_s: float = .008
    stationary_gyro_limit_rad_s: float = .12
    wheelbase_m: float = .145
    steering_sign_verified: bool = False
    encoder_sign_verified: bool = False
    initial_encoder_sign: int = 1
    turn_min_duration_s: float = .25
    forward_min_duration_s: float = .25
    raw_encoder_candidates: bool = False
    nominal_counts_per_motor_rev: float = 28.
    nominal_gear_ratio: float = 11.9118
    nominal_wheel_radius_m: float = .0325

    def __post_init__(self):
        for name, value in vars(self).items():
            if name in ('steering_sign_verified', 'encoder_sign_verified', 'raw_encoder_candidates'):
                if type(value) is not bool:
                    raise ValueError(name+' must be bool')
            elif name == 'initial_encoder_sign':
                if type(value) is not int or value not in (-1, 1):
                    raise ValueError('initial_encoder_sign must be -1 or 1')
            elif name == 'min_samples':
                if type(value) is not int or value < 3:
                    raise ValueError('min_samples must be integer >= 3')
            elif not math.isfinite(value) or value <= 0:
                raise ValueError(name+' must be finite and positive')


def _vector(value):
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    return result if result.shape == (3,) and np.all(np.isfinite(result)) else None


class QuickImuCalibration:
    """단일 executor용. 활성 보정은 검증된 WAIT 창 끝에서만 원자적으로 교체한다.

    시간/정지/저진동 조건은 정지 후보의 근거이며 차체 정지 증명은 아니다.
    운반 중 등속 이동, 완만한 지속 회전, 일정 가속도/경사는 이 입력만으로 완전히 분리 못 한다.
    """
    def __init__(self, config=None):
        self.config = config or QuickCalibrationConfig()
        self.bias = self.up = self.forward = None
        self.encoder_sign = self.config.initial_encoder_sign
        self.encoder_sign_source = 'prior_verified_profile' if self.config.encoder_sign_verified else 'configured_unverified'
        self.version = 0
        self.state = self.command = self.encoder = self.accel = self.steering = None
        self.raw_encoder = None
        self.raw_speed = self.raw_previous_speed = None
        self.raw_encoder_sign = None
        self.raw_stationary_since = None
        self.retired_encoder_boots = set()
        self.last_imu = None
        self.reason = 'waiting_for_inputs'
        self.reset_count = 0
        self._reset_wait()
        self._reset_dynamic()

    def _reset_wait(self):
        self.window = []
        self.wait_done = False

    def _reset_dynamic(self):
        self.turn_votes = {-1: {-1: 0., 1: 0.}, 1: {-1: 0., 1: 0.}}
        self.turn_samples = 0
        self.forward_sum = np.zeros(3)
        self.forward_weight = 0.
        self.forward_samples = 0
        self.last_dynamic_encoder = None
        self.last_dynamic_accel = None
        self.previous_encoder = None
        self.dynamic_encoder_basis = None

    def reset_inputs(self):
        """재시작/시각 경계에서 활성 보정은 보존하고 오래된 증거만 버린다."""
        self.state = self.command = self.encoder = self.accel = self.steering = None
        self.raw_encoder = None
        self.raw_speed = self.raw_previous_speed = None
        self.raw_stationary_since = None
        self.last_imu = None
        self._reset_wait()
        self._reset_dynamic()
        self.reason = 'input_boundary'

    def reject(self, reason):
        if self.window:
            self.reset_count += 1
        self.window = []
        self.wait_done = False
        self.reason = reason

    def _fresh(self, item, now, age=None):
        return item is not None and -1e-6 <= now-item[0] <= (age or self.config.input_max_age_s)

    def _stamp(self, stamp, now, previous):
        return (math.isfinite(stamp) and math.isfinite(now) and 0 <= stamp <= now+1e-6
                and now-stamp <= self.config.input_max_age_s
                and (previous is None or stamp > previous[0]))

    def observe_state(self, stamp, state, now):
        if (not isinstance(state, str) or not math.isfinite(stamp) or not math.isfinite(now)
                or not -1e-6 <= now-stamp <= self.config.state_max_age_s
                or (self.state is not None and stamp <= self.state[0])):
            return False
        was_wait = self.state is not None and self.state[1] in WAIT_STATES
        if (state in WAIT_STATES) != was_wait:
            self._reset_wait()
        self.state = (stamp, state)
        if state not in WAIT_STATES:
            self.window = []
        return True

    def observe_command(self, stamp, speed, steering, now):
        if not self._stamp(stamp, now, self.command) or not all(map(math.isfinite, (speed, steering))):
            self.reject('command_invalid')
            return False
        self.command = (stamp, speed, steering)
        if abs(speed) > self.config.zero_command_mps:
            self.reject('command_not_zero')
        return True

    def observe_encoder(self, stamp, speed, now):
        if not self._stamp(stamp, now, self.encoder) or not math.isfinite(speed):
            self.reject('encoder_invalid')
            return False
        self.previous_encoder, self.encoder = self.encoder, (stamp, speed)
        if abs(speed) > self.config.stationary_speed_mps:
            self.reject('encoder_moving')
        return True

    def observe_accel(self, stamp, value, now):
        value = _vector(value)
        if value is None or not self._stamp(stamp, now, self.accel):
            self.reject('accel_invalid')
            return False
        self.accel = (stamp, value)
        return True

    def observe_encoder_count(self, stamp, count, boot, sequence, capture_us, now):
        """부호/기어비/PI가 없어도 raw count 불변으로 정지 후보만 판단한다.

        이동 이력에 속도 0을 넣지 않는다. MCU의 독립 관측 순서/시각이 전진해야 한다.
        """
        prior = self.raw_encoder
        if (not self._stamp(stamp, now, prior) or type(count) is not int
                or not isinstance(boot, str) or not boot or boot in self.retired_encoder_boots
                or type(sequence) is not int or sequence < 0
                or type(capture_us) is not int or capture_us < 0):
            self.raw_stationary_since = None
            self.reject('raw_encoder_invalid')
            return False
        if prior is not None and boot == prior[2] and (sequence <= prior[3] or capture_us <= prior[4]):
            self.raw_stationary_since = None
            self.reject('raw_encoder_not_advancing')
            return False
        restart = (prior is None or self.raw_stationary_since is None or boot != prior[2] or count != prior[1]
                   or stamp-prior[0] > self.config.input_max_age_s)
        if prior is not None and boot != prior[2]:
            self.retired_encoder_boots.add(prior[2])
            self._reset_dynamic()
        if (prior is not None and boot == prior[2]
                and 0 < capture_us-prior[4] <= self.config.input_max_age_s*1e6):
            c = self.config
            delta_s = (capture_us-prior[4])*1e-6
            nominal_speed = ((count-prior[1])/delta_s * 2*math.pi*c.nominal_wheel_radius_m
                             / (c.nominal_counts_per_motor_rev*c.nominal_gear_ratio))
            self.raw_previous_speed, self.raw_speed = self.raw_speed, (capture_us*1e-6, nominal_speed)
        else:
            self.raw_speed = self.raw_previous_speed = None
        if restart:
            self.raw_stationary_since = stamp
            self.reject('raw_encoder_window_restart')
        self.raw_encoder = (stamp, count, boot, sequence, capture_us)
        return True

    def observe_steering(self, stamp, angle, now, *, sign_verified=False):
        if (not self._stamp(stamp, now, self.steering) or not math.isfinite(angle)
                or abs(angle) > .8):
            self.steering = None
            return False
        self.steering = (stamp, angle, bool(sign_verified))
        return True

    def observe_imu(self, stamp, gyro, now):
        gyro = _vector(gyro)
        if (gyro is None or not self._stamp(stamp, now, None)
                or (self.last_imu is not None and stamp <= self.last_imu)):
            self.reject('gyro_invalid')
            return False
        if self.last_imu is not None and stamp-self.last_imu > self.config.sample_gap_s:
            self.reject('imu_gap')
        self.last_imu = stamp
        c = self.config
        if not self._fresh(self.state, now, c.state_max_age_s):
            self.reject('state_stale')
            return False
        if self.state[1] not in WAIT_STATES:
            self.reason = 'driving_bias_frozen' if self.up is not None else 'awaiting_stable_wait'
            self._observe_dynamic(stamp, gyro, now)
            return False
        for name, item in (('command', self.command), ('accel', self.accel)):
            if not self._fresh(item, now) or abs(stamp-item[0]) > c.input_max_age_s:
                self.reject(name+'_stale')
                return False
        if self._fresh(self.encoder, now):
            if abs(self.encoder[1]) > c.stationary_speed_mps:
                self.reject('encoder_moving')
                self._observe_dynamic(stamp, gyro, now)
                return False
        elif not (self._fresh(self.raw_encoder, now) and self.raw_stationary_since is not None
                  and stamp-self.raw_stationary_since >= .1):
            self.reject('encoder_stale')
            self._observe_dynamic(stamp, gyro, now)
            return False
        if (self._fresh(self.raw_encoder, now) and (self.raw_stationary_since is None
                or stamp-self.raw_stationary_since < .1)):
            self.reject('raw_encoder_not_stationary')
            self._observe_dynamic(stamp, gyro, now)
            return False
        if abs(self.command[1]) > c.zero_command_mps:
            self.reject('command_not_zero')
            self._observe_dynamic(stamp, gyro, now)
            return False
        acceleration = self.accel[1]
        if abs(np.linalg.norm(acceleration)-c.gravity_mps2) > c.gravity_tolerance_mps2:
            self.reject('accel_not_gravity')
            return False
        if np.linalg.norm(gyro) > c.stationary_gyro_limit_rad_s:
            self.reject('gyro_motion')
            return False
        if self.wait_done:
            self.reason = 'ready_wait'
            return False
        self.window.append((stamp, gyro, acceleration, self.accel[0]))
        gyros = np.asarray([row[1] for row in self.window])
        accels = np.asarray([row[2] for row in self.window])
        if (np.max(np.std(gyros, axis=0)) > c.gyro_std_limit_rad_s
                or np.max(np.std(accels, axis=0)) > c.accel_std_limit_mps2):
            self.reject('window_not_stable')
            return False
        self.reason = 'collecting_stationary'
        if (len(self.window) < c.min_samples or stamp-self.window[0][0] < c.stationary_duration_s
                or len({row[3] for row in self.window}) < max(3, c.min_samples//4)):
            return False
        new_bias, gravity = np.mean(gyros, axis=0), np.mean(accels, axis=0)
        new_up = gravity/np.linalg.norm(gravity)
        if self.up is not None and np.dot(new_up, self.up) < math.cos(math.radians(5)):
            self.forward = None
            self._reset_dynamic()
        self.bias, self.up = new_bias, new_up
        candidate = self.encoder_candidate()
        if candidate is not None:
            if self.dynamic_encoder_basis == 'raw_count_nominal':
                self.raw_encoder_sign = candidate
            else:
                self.encoder_sign = candidate
                self.encoder_sign_source = 'measured_steering_kinematic_candidate'
        forward = self.forward_candidate()
        forward_sign = (self.raw_encoder_sign if self.dynamic_encoder_basis == 'raw_count_nominal' else
                        self.encoder_sign if self.encoder_sign_source != 'configured_unverified' else None)
        if forward is not None and forward_sign is not None:
            forward = forward*forward_sign
            forward -= np.dot(forward, self.up)*self.up
            self.forward = forward/np.linalg.norm(forward)
        self.version += 1
        self.wait_done = True
        self.window = []
        self.reason = 'ready_wait'
        return True

    def _observe_dynamic(self, stamp, gyro, now):
        if self.up is None or not self._fresh(self.accel, now):
            return
        if self._fresh(self.encoder, now):
            encoder, prior, basis = self.encoder, self.previous_encoder, 'motion_feedback'
        elif (self.config.raw_encoder_candidates and self._fresh(self.raw_encoder, now)
              and self.raw_stationary_since is not None and self.raw_speed is not None):
            encoder, prior, basis = self.raw_speed, self.raw_previous_speed, 'raw_count_nominal'
        else:
            return
        if self.dynamic_encoder_basis is not None and basis != self.dynamic_encoder_basis:
            self._reset_dynamic()  # raw와 이미 부호변환된 정규속도의 증거를 섞지 않는다.
        self.dynamic_encoder_basis = basis
        if encoder[0] == self.last_dynamic_encoder:
            return
        self.last_dynamic_encoder = encoder[0]
        if prior is None or not 0 < encoder[0]-prior[0] <= self.config.input_max_age_s:
            return
        dt = encoder[0]-prior[0]
        speed = encoder[1]
        yaw = float(np.dot(gyro-self.bias, self.up))
        steer = self.steering[1] if self._fresh(self.steering, now) else None
        # 실제 조향 피드백 + 별도로 검증된 좌/우 부호만 사용하며 명령 부호를 참값으로 쓰지 않는다.
        if ((self.config.steering_sign_verified or (steer is not None and self.steering[2]))
                and steer is not None and abs(steer) >= .08 and abs(speed) >= .10 and abs(yaw) >= .08):
            expected = speed*math.tan(steer)/self.config.wheelbase_m
            ratio = yaw/expected
            if .4 <= abs(ratio) <= 2.5:
                direction = 1 if steer > 0 else -1
                sign = 1 if ratio > 0 else -1
                self.turn_votes[direction][sign] += dt
                self.turn_samples += 1
        # 직선 가감속의 수평 specific force와 encoder dv/dt 상관. 아직 전방이라는 뜻은 아니다.
        acceleration = (speed-prior[1])/dt
        if (self.accel[0] != self.last_dynamic_accel and abs(yaw) < .15
                and (steer is None or abs(steer) < .06) and .2 <= abs(acceleration) <= 5.):
            self.last_dynamic_accel = self.accel[0]
            horizontal = self.accel[1]-np.dot(self.accel[1], self.up)*self.up
            magnitude = np.linalg.norm(horizontal)
            if .15 <= magnitude <= 5. and .35 <= magnitude/abs(acceleration) <= 2.5:
                self.forward_sum += horizontal/magnitude * math.copysign(dt, acceleration)
                self.forward_weight += dt
                self.forward_samples += 1

    def encoder_candidate(self):
        if self.turn_samples < 12:
            return None
        result = []
        for side in (-1, 1):
            votes = self.turn_votes[side]
            total = sum(votes.values())
            winner = max(votes, key=votes.get)
            if total < self.config.turn_min_duration_s or votes[winner]/total < .9:
                return None
            result.append(winner)
        return result[0] if result[0] == result[1] else None

    def forward_candidate(self):
        if self.forward_samples < 12 or self.forward_weight < self.config.forward_min_duration_s:
            return None
        magnitude = np.linalg.norm(self.forward_sum)
        if magnitude/self.forward_weight < .9:
            return None
        return self.forward_sum/magnitude

    def snapshot(self):
        vector = lambda value: None if value is None else [float(x) for x in value]
        forward_candidate, sign_candidate = self.forward_candidate(), self.encoder_candidate()
        return dict(enabled=True, status=self.reason, version=self.version,
                    gyro_bias_rad_s=vector(self.bias), up_axis_sensor=vector(self.up),
                    forward_axis_sensor=vector(self.forward),
                    forward_axis_encoder_positive_candidate=vector(forward_candidate),
                    forward_axis_status=('active_approximate' if self.forward is not None else
                                         'candidate_awaiting_reference_or_stop' if forward_candidate is not None
                                         else 'pending_longitudinal_excitation'),
                    encoder_sign=self.encoder_sign, encoder_sign_source=self.encoder_sign_source,
                    encoder_sign_candidate=sign_candidate,
                    encoder_sign_candidate_basis=self.dynamic_encoder_basis,
                    raw_encoder_sign=self.raw_encoder_sign,
                    raw_candidate_speed_published=False,
                    encoder_sign_candidate_conflicts_configured=(None if self.dynamic_encoder_basis == 'raw_count_nominal'
                        else sign_candidate is not None and sign_candidate != self.config.initial_encoder_sign),
                    raw_sign_requires_installed_profile_reconciliation=(self.raw_encoder_sign is not None),
                    encoder_sign_conflicts_configured=self.encoder_sign != self.config.initial_encoder_sign,
                    motor_controller_polarity_updated=False,
                    stationary_samples=len(self.window), candidate_resets=self.reset_count,
                    raw_count_stationarity_available=self.raw_encoder is not None,
                    turn_samples=self.turn_samples, turn_evidence_s=self.turn_votes,
                    forward_samples=self.forward_samples,
                    confidence='stationary_gravity_approximation' if self.up is not None else 'pending',
                    physical_calibration_verified=False)
