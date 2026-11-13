"""수신기 임대가 수명을 제한하는 호스트 후보. serial/ROS/실물 출력을 열지 않는다.

절대 시계 offset과 전송 지연 상한 없이, 수신한 lease의 발급이 송신 판단보다
앞선다는 인과관계와 명시된 상대 시계 비율 범위를 쓴다. 원천 시각은 호출자가
보존한 같은 host 단조 시계 구간이어야 한다.
"""
from dataclasses import dataclass
from fractions import Fraction
import math

from actuation_host_session import CompactHostSession
from actuation_host_observer import validate_payload
from arena_vehicle_interface.clock_bounds import integer
from arena_vehicle_interface.clock_freshness import HostTimeWindow
from arena_vehicle_interface.actuation_wire import encode


@dataclass(frozen=True, slots=True)
class ReceiverLeaseTiming:
    source_ttl_ns: int
    max_receipt_age_ns: int
    max_rate_error_ppm: int
    policy_note: str

    def __post_init__(self):
        integer(self.source_ttl_ns, 'source TTL', 10_000_000_000)
        integer(self.max_receipt_age_ns, 'receipt age', 10_000_000_000)
        integer(self.max_rate_error_ppm, 'relative clock rate ppm', 999_999)
        if (not self.source_ttl_ns or not self.max_receipt_age_ns
                or type(self.policy_note) is not str or not self.policy_note.strip()):
            raise ValueError('explicit positive time bounds and policy note required')


def offer_time(offer, received, now, timing, max_lease_us):
    kind, _ = validate_payload(offer, max_lease_us)
    if kind != 'offer' or not isinstance(timing, ReceiverLeaseTiming):
        raise ValueError('offer and explicit receiver-owned timing required')
    if (not isinstance(received, HostTimeWindow) or not isinstance(now, HostTimeWindow)
            or received.host_epoch != now.host_epoch):
        raise ValueError('same host clock epoch required')
    age_lower, age_upper = now.lower_ns-received.upper_ns, now.upper_ns-received.lower_ns
    conditions = dict(receipt_before_send=age_lower >= 0,
                      host_queue_age_bounded=age_upper < timing.max_receipt_age_ns)
    return dict(policy='receiver_owned_lease_source_age',
        timestamp_policy_satisfied=all(conditions.values()), conditions=conditions,
        receipt_age_upper_ns=age_upper, absolute_clock_fit_used=False,
        arrival_before_expiry_proven=False, hardware_rate_assumption_verified=False)


def source_time(offer, received, now, source, arm_barrier, timing, max_lease_us):
    prior = offer_time(offer, received, now, timing, max_lease_us)
    if any(not isinstance(x, HostTimeWindow) or x.host_epoch != now.host_epoch
           for x in (source, arm_barrier)):
        raise ValueError('source and ARM in same host clock epoch required')
    age_lower, age_upper = now.lower_ns-source.upper_ns, now.upper_ns-source.lower_ns
    delta = Fraction(timing.max_rate_error_ppm, 1_000_000)
    # h_ns/(mcu_us*1000) in [1-delta,1+delta]. 중간값 대신 정수 상하한을 쓴다.
    age_us = math.ceil(Fraction(age_upper, 1000)/(1-delta)) if age_lower >= 0 else None
    ttl_us = min(max_lease_us, offer['lease_expires_us']-offer['lease_issued_us'],
                 math.floor(Fraction(timing.source_ttl_ns, 1000)/(1+delta)))
    conditions = dict(offer_conditions=prior['timestamp_policy_satisfied'],
        source_before_send=age_lower >= 0, source_after_arm=source.lower_ns >= arm_barrier.upper_ns,
        source_age_representable=age_us is not None and 0 <= age_us < ttl_us)
    valid = all(conditions.values())
    return dict(policy='receiver_owned_lease_source_age', timestamp_policy_satisfied=valid, bound_offer=dict(offer),
        conditions=conditions, candidate_source_age_us=age_us, candidate_ttl_us=ttl_us,
        receiver_deadline_us=(min(offer['lease_expires_us'], offer['lease_issued_us']+ttl_us-age_us)
                              if valid else None),
        source_deadline_lower_ns=source.lower_ns+timing.source_ttl_ns,
        max_rate_error_ppm=timing.max_rate_error_ppm, policy_note=timing.policy_note,
        arrival_before_expiry_proven=False, hardware_rate_assumption_verified=False,
        source_host_mapping_verified=False, offer_review=prior)


class ReceiverLeaseHostSession(CompactHostSession):
    """기존 ACK/세대/STOP 순서를 재사용한다. 반환 envelope의 실송신은 별도 책임이다.

    명령 원천 시각을 수신 시각으로 바꾸지 않는다. 제어 요청과 DRIVE는 같은
    sequence 열을 쓰고 STOP/절단/새 세대에서 기존 ARM 근거를 유지하지 않는다.
    """
    def __init__(self, *, timing, max_speed_mps, max_steering_rad, **options):
        if not isinstance(timing, ReceiverLeaseTiming):
            raise ValueError('explicit receiver-owned timing required')
        for value in (max_speed_mps, max_steering_rad):
            if type(value) not in (float, int) or not math.isfinite(value) or value <= 0:
                raise ValueError('finite positive actuation limits required')
        super().__init__(**options)
        self.timing = timing
        self.max_speed_mps, self.max_steering_rad = max_speed_mps, max_steering_rad

    def _review_offer_time(self, fit, now, **options):
        receipt = self.latest('offer')
        return offer_time(receipt.payload, receipt.received, now, self.timing, self._observer.max_lease_us)

    def _review_source_time(self, receipt, fit, now, source, **options):
        return source_time(receipt.payload, receipt.received, now, source, self._arm_barrier,
                           self.timing, self._observer.max_lease_us)

    def prepare_drive(self, *, speed_mps, steering_rad, source, now):
        # 잘못된 수치 때문에 유효 원천 watermark를 먼저 소비하지 않는다.
        if (type(speed_mps) not in (int, float) or not math.isfinite(speed_mps)
                or not 0 <= speed_mps <= self.max_speed_mps
                or type(steering_rad) not in (int, float) or not math.isfinite(steering_rad)
                or abs(steering_rad) > self.max_steering_rad):
            return self._result('invalid_drive_values', envelope=None, command_created=False)
        if self._sequence >= 2**32-1:
            self.disconnect('request_sequence_exhausted')
            return self._result('request_sequence_exhausted', envelope=None, command_created=False)
        reviewed = self.review_source(None, now, source,
            max_issue_age_ns=None, forward_delay_upper_ns=None,
            source_ttl_ns=self.timing.source_ttl_ns, policy_note=self.timing.policy_note)
        if not reviewed['session_conditions_satisfied']:
            return dict(reviewed, envelope=None)
        # age를 계산한 바로 그 lease를 사용한다. 이후 새 offer로 바꿔 끼우지 않는다.
        offer = reviewed['timing']['bound_offer']
        self._sequence += 1
        command = dict(v=1, kind='DRIVE', boot=offer['boot'], generation=offer['generation'],
            lease=offer['lease'], client=self.client_id, seq=self._sequence,
            speed_mps=float(speed_mps), steering_rad=float(steering_rad),
            source_age_us=reviewed['timing']['candidate_source_age_us'],
            ttl_us=reviewed['timing']['candidate_ttl_us'])
        envelope=dict(type='command', request='', command=command)
        return dict(reviewed, command_created=True, envelope=envelope, wire=encode(envelope))
