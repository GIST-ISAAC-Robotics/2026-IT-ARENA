"""조건부 시각 구간의 신선도 진단. ROS stamp 생성·구동 허가와 미연결."""
from dataclasses import dataclass

from arena_vehicle_interface.clock_bounds import MappedInterval, integer


@dataclass(frozen=True, slots=True)
class HostTimeWindow:
    """호스트 단조 시계의 결정 시각과 명시된 읽기 오차 구간(ns)."""
    host_epoch: str
    lower_ns: int
    upper_ns: int

    def __post_init__(self):
        if not isinstance(self.host_epoch, str) or not self.host_epoch.strip() or len(self.host_epoch) > 128:
            raise ValueError('explicit host clock epoch required')
        integer(self.lower_ns, 'host lower')
        integer(self.upper_ns, 'host upper')
        if self.upper_ns < self.lower_ns:
            raise ValueError('host interval reversed')


def assess_freshness(mapped, now, *, mapping_host_epoch, max_age_ns, inclusive=False):
    """모든 가능한 취득 나이가 범위 안일 때만 fresh_under_assumptions.

    MCU↔host affine 적합과 시계 세대의 실물 타당성은 호출자의 별도 의무다.
    가운데 값을 ROS stamp로 만들거나 uncertainty를 timeout에서 빼지 않는다.
    반환 구간은 상관을 이용해 줄이지 않은 보수적 차집합이다.
    """
    if not isinstance(mapped, MappedInterval) or not isinstance(now, HostTimeWindow):
        raise ValueError('typed mapped and host intervals required')
    if now.host_epoch != mapping_host_epoch:
        raise ValueError('host clock epoch mismatch')
    integer(max_age_ns, 'max age')
    if max_age_ns == 0 or type(inclusive) is not bool:
        raise ValueError('positive age limit and explicit endpoint policy required')
    # MappedInterval 자체는 공개 결과 dataclass이므로 수동 생성된 잘못된 값도 거절한다.
    for name in ('capture_us', 'host_lower_ns', 'host_upper_ns', 'width_ns', 'extrapolation_us'):
        integer(getattr(mapped, name), name)
    if (not isinstance(mapped.boot, str) or not mapped.boot.strip()
            or mapped.host_upper_ns-mapped.host_lower_ns != mapped.width_ns):
        raise ValueError('invalid mapped interval')
    lower = now.lower_ns-mapped.host_upper_ns
    upper = now.upper_ns-mapped.host_lower_ns
    within_limit = upper <= max_age_ns if inclusive else upper < max_age_ns
    beyond_limit = lower > max_age_ns if inclusive else lower >= max_age_ns
    if upper < 0:
        status = 'future_under_assumptions'
    elif beyond_limit:
        status = 'stale_under_assumptions'
    elif lower >= 0 and within_limit:
        status = 'fresh_under_assumptions'
    else:
        status = 'uncertain_age'
    return dict(status=status, age_lower_ns=lower, age_upper_ns=upper,
                max_age_ns=max_age_ns, limit_inclusive=inclusive,
                mapping_width_ns=mapped.width_ns, now_width_ns=now.upper_ns-now.lower_ns,
                boot=mapped.boot, host_epoch=now.host_epoch,
                timestamp_policy_satisfied=status == 'fresh_under_assumptions',
                ready_to_drive=False, ros_stamp_verified=False, clock_mapping_applied=False,
                sensor_value_validated=False, hardware_assumptions_verified=False)
