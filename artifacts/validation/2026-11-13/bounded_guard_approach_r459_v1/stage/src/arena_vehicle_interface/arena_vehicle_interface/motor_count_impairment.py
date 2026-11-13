"""시험용 누적 카운트 이득. 정수 격자·카운트 차분 속도를 함께 유지한다.

아날로그 각도에 이득을 곱해 분수 카운트를 발행하지 않는다. 관측 간 카운트
증분을 이득으로 누적한 뒤 반올림하며, 구간 경계를 걸치는 증분은 시간 비율로
나눈다(구간 안 일정 회전 가정). 실제 펄스 누락 확률/기어 오차 모델은 아니다.
"""
import math

from arena_vehicle_interface.drive_feedback import MotorEncoderSpec


MODEL = 'integer_count_gain_v1'


class MotorCountTransform:
    def __init__(self, counts_per_revolution):
        # 이 변환은 모터 각만 다룬다. 감속비·바퀴 환산은 기존 소비자의 몫이다.
        self.spec = MotorEncoderSpec(counts_per_revolution, 1., 1.)
        self.reset()

    def reset(self):
        self.previous = None
        self.accumulated = 0.

    def apply(self, position, velocity, stamp, fault):
        if not math.isfinite(stamp) or fault.bias != 0:
            raise ValueError('finite source time and zero motor additive bias required')
        raw = self.spec.counts_from_position(position)
        previous = self.previous
        rate = math.nan
        if previous is None:
            # 누적 카운터 원점은 이동량이 아니므로 초기값을 그대로 둔다.
            self.accumulated = float(raw)
        else:
            old_t, old_raw, old_output = previous
            dt = stamp-old_t
            if dt <= 0:
                raise ValueError('strictly increasing motor capture time required')
            active_dt = max(0., min(stamp, fault.end_s if fault.end_s is not None else stamp)
                            - max(old_t, fault.start_s))
            self.accumulated += (raw-old_raw) * (1.+(fault.scale-1.)*active_dt/dt)
        output = round(self.accumulated)
        angle = self.spec.motor_angle_from_counts(output)
        # 소비자와 같은 signed 64-bit 격자를 벗어나는 입력도 거절한다.
        self.spec.counts_from_position(angle)
        if previous is not None and math.isfinite(velocity):
            rate = self.spec.motor_rad_s_from_counts(output-previous[2], stamp-previous[0])
        self.previous = (stamp, raw, output)
        return angle, rate
