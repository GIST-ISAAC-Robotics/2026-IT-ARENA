"""호스트 송신 후보의 시간 정책. MCU 임대의 만료 판정을 대신하지 않는다."""
import math

from arena_vehicle_interface.actuation_contract import integer

SHARED_CLOCK = 'shared_clock'
RECEIVER_LEASE = 'receiver_owned_lease'
POLICIES = (SHARED_CLOCK, RECEIVER_LEASE)
MAX_OFFER_US = 100_000


def offer_send_eligible(offer, *, policy, host_now_us, received_wall, now_wall, wall_timeout):
    """서로 다른 시계의 절대값을 빼지 않고 송신 후보만 선택한다.

    receiver_owned_lease는 최근 수신/형식을 확인할 뿐 MCU에서 아직 유효하다고
    보장하지 않는다. 원 boot/generation/lease와 원 source_age를 그대로 보내고,
    MCU는 자신의 issued/expires로 최종 수락한다. 수신 시각부터 임대를 재시작하지 않는다.
    기간의 단위는 기존 계약의 microsecond이며 발진기 오차 교정은 별도다.
    """
    if policy not in POLICIES:
        raise ValueError('unknown offer_time_policy')
    if offer is None:
        return False
    elapsed = now_wall - received_wall
    if policy == SHARED_CLOCK:
        # 기존 시뮬레이션 판정 순서/경계를 그대로 유지한다.
        return elapsed < wall_timeout and 0 <= host_now_us - offer.get('lease_issued_us', -1) < MAX_OFFER_US
    if not (math.isfinite(elapsed) and 0 <= elapsed < min(wall_timeout, MAX_OFFER_US / 1e6)):
        return False
    if not all(integer(offer.get(key)) for key in ('generation', 'lease_issued_us', 'lease_expires_us')):
        return False
    return (integer(offer.get('lease'), 1) and type(offer.get('boot')) is str and bool(offer['boot'])
            and 0 < offer['lease_expires_us'] - offer['lease_issued_us'] <= MAX_OFFER_US)
