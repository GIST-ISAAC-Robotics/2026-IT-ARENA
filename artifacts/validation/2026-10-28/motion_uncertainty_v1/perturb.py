"""이미 수신된 입력의 잔여 오차 민감도. 실물 사양/물리 회전 속도 모델이 아니다."""
from dataclasses import dataclass
import hashlib
import math
from types import SimpleNamespace

from arena_autonomy.follow_trace import thaw
from arena_autonomy.local_path import mask_scan


@dataclass(frozen=True)
class ErrorCase:
    name: str
    gyro_sigma: float = 0.
    gyro_bias: float = 0.
    angle_bias: float = 0.
    ray_time_scale: float = 1.
    range_uniform: float = 0.
    seed: int = 28

    def __post_init__(self):
        values = (self.gyro_sigma, self.gyro_bias, self.angle_bias,
                  self.ray_time_scale, self.range_uniform)
        if not all(math.isfinite(x) for x in values):
            raise ValueError('finite error parameters required')
        if self.gyro_sigma < 0 or self.range_uniform < 0 or self.ray_time_scale <= 0:
            raise ValueError('invalid error magnitude')


CASES = (
    ErrorCase('baseline'),
    ErrorCase('gyro', gyro_sigma=.02, gyro_bias=math.radians(1)),
    ErrorCase('angle', angle_bias=math.radians(.5)),
    ErrorCase('ray_clock', ray_time_scale=1.05),
    ErrorCase('combined', gyro_sigma=.02, gyro_bias=math.radians(1),
              angle_bias=math.radians(.5), ray_time_scale=1.05, range_uniform=.03),
)


def uniform(seed, *key):
    # 호출 순서·이력 창과 무관하게 같은 취득 표본에는 같은 오차를 준다.
    blob = repr((seed, *key)).encode('ascii')
    value = int.from_bytes(hashlib.blake2b(blob, digest_size=8).digest(), 'big')
    return (value + .5) / 2**64


def normal(seed, *key):
    u = max(1e-16, uniform(seed, *key, 0))
    return math.sqrt(-2 * math.log(u)) * math.cos(2 * math.pi * uniform(seed, *key, 1))


def transform(recorded, case):
    data = thaw(recorded)
    gyro = data['history']['gyro']
    gyro['values'] = [v + case.gyro_bias + case.gyro_sigma * normal(case.seed, 'gyro', float(t).hex())
                      for t, v in zip(gyro['times'], gyro['values'])]
    scan = data['scan']
    # 회전 끝 시각은 유지한다. 물리적 가변 RPM 대신 보고된 광선 시각의 오차다.
    source_stamp = (scan['sec'], scan['nanosec'])
    old_increment = scan['time_increment']
    if case.ray_time_scale != 1.:
        old_start_ns = scan['sec'] * 10**9 + scan['nanosec']
        new_increment = old_increment * case.ray_time_scale
        new_start_ns = old_start_ns + round((len(scan['ranges']) - 1) * (old_increment - new_increment) * 1e9)
        scan['sec'], scan['nanosec'] = divmod(new_start_ns, 10**9)
        scan['time_increment'] = new_increment
    scan['angle_min'] += case.angle_bias
    if case.range_uniform:
        scan['ranges'] = [r + case.range_uniform * (2 * uniform(case.seed, 'range', source_stamp, i) - 1)
                          if math.isfinite(r) else r for i, r in enumerate(scan['ranges'])]
    if case.angle_bias:
        # 기록은 on_scan의 마스크 이후 값이다. 각도 메타데이터를 바꿨으면
        # 실제 수신기와 같은 마스크를 다시 적용해야 scene과 일치한다.
        # 과거 마스크로 잃은 점을 복원하지 않으므로 추가 미관측은 그대로다.
        scan['ranges'] = mask_scan(SimpleNamespace(**scan), data['rear_half_angle_deg']).ranges
    return data
