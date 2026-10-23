"""원천 입력 나이를 MCU 단위로 옮기는 오프라인 산술 후보. 명령을 만들지 않는다.

source는 원래 /drive/safe 생성 시각을 host 단조 구간으로 옮겼다는 외부 주장이다.
이 파일은 ROS→host 변환/교정이나 watermark/ARM 절차를 구현하지 않는다.
"""
from dataclasses import asdict
from fractions import Fraction
import math

from audit_actuation_offer_clock import assess_offer
from arena_vehicle_interface.clock_bounds import integer
from arena_vehicle_interface.clock_freshness import HostTimeWindow


def review_source_time(offer, fit, received, now, source, arm_barrier, *, mapping_host_epoch,
                       expected_boot, expected_generation, max_lease_us, max_issue_age_ns,
                       forward_delay_upper_ns, source_ttl_ns, policy_note):
    """단위가 다른 두 지속 시간을 보수적으로 환산한 뒤 기존 수신기 식과 대조한다.

    h_ns = offset + rate * mcu_us * 1000 이고 rate는 선언된 범위 안이다.
    age는 가장 느린 host/MCU 비율로 올림, TTL은 가장 빠른 비율로 내림한다.
    추정 중간값이나 실제 차체 상태를 사용하지 않는다.
    """
    offer_report = assess_offer(offer, fit, received, now, mapping_host_epoch=mapping_host_epoch,
                                expected_boot=expected_boot, expected_generation=expected_generation,
                                max_lease_us=max_lease_us, max_issue_age_ns=max_issue_age_ns,
                                forward_delay_upper_ns=forward_delay_upper_ns, policy_note=policy_note)
    if not isinstance(source, HostTimeWindow) or not isinstance(arm_barrier, HostTimeWindow):
        raise ValueError('typed source and ARM windows required')
    if source.host_epoch != mapping_host_epoch or arm_barrier.host_epoch != mapping_host_epoch:
        raise ValueError('source/ARM epoch mismatch')
    integer(source_ttl_ns, 'host source TTL', 10_000_000_000)
    if not source_ttl_ns:
        raise ValueError('positive source TTL required')
    source_deadline_lower = source.lower_ns + source_ttl_ns
    integer(source_deadline_lower, 'source deadline')
    rate_delta = Fraction(fit.assumptions.max_rate_error_ppm, 1_000_000)
    rate_min, rate_max = 1-rate_delta, 1+rate_delta
    age_lower_ns = now.lower_ns-source.upper_ns
    age_upper_ns = now.upper_ns-source.lower_ns
    ordered = age_lower_ns >= 0
    after_arm = source.lower_ns >= arm_barrier.upper_ns
    # 미래 입력의 음수 나이를 0으로 고치지 않는다. 후보 계산 없음으로 보고한다.
    age_us = math.ceil(Fraction(age_upper_ns, 1000)/rate_min) if ordered else None
    ttl_us = min(max_lease_us, offer['lease_expires_us']-offer['lease_issued_us'],
                 math.floor(Fraction(source_ttl_ns, 1000)/rate_max))
    representable = age_us is not None and 0 <= age_us < ttl_us
    deadline_us = None
    mapped_deadline = None
    arrival_before_deadline = deadline_within_source = False
    if representable:
        integer(age_us, 'MCU source age')
        deadline_us = min(offer['lease_expires_us'], offer['lease_issued_us']+ttl_us-age_us)
        mapped_deadline = fit.map_interval(deadline_us, boot=expected_boot)
        arrival_before_deadline = offer_report['arrival_upper_ns'] < mapped_deadline.host_lower_ns
        deadline_within_source = mapped_deadline.host_upper_ns <= source_deadline_lower
    conditions = dict(offer_time_conditions=offer_report['timestamp_policy_satisfied'],
                      source_order_certain=ordered, source_after_arm_certain=after_arm,
                      source_age_representable=representable,
                      arrival_before_source_expiry=offer_report['arrival_upper_ns'] < source_deadline_lower,
                      arrival_before_receiver_deadline=arrival_before_deadline,
                      receiver_deadline_not_extended=deadline_within_source)
    return dict(schema='arena_actuation_source_time_review_v1',
                timestamp_policy_satisfied=all(conditions.values()), conditions=conditions,
                source=asdict(source), arm_barrier=asdict(arm_barrier), now=asdict(now),
                source_age_lower_ns=age_lower_ns, source_age_upper_ns=age_upper_ns,
                candidate_source_age_us=age_us, candidate_ttl_us=ttl_us,
                receiver_deadline_us=deadline_us,
                mapped_receiver_deadline=None if mapped_deadline is None else asdict(mapped_deadline),
                source_deadline_lower_ns=source_deadline_lower, source_ttl_ns=source_ttl_ns,
                offer_review=offer_report,
                rate_min=[rate_min.numerator, rate_min.denominator],
                rate_max=[rate_max.numerator, rate_max.denominator],
                hardware_assumptions_verified=False, source_host_mapping_verified=False,
                source_watermark_checked=False, arm_ack_correlated=False,
                command_created=False, ready_to_drive=False, runtime_connected=False)
