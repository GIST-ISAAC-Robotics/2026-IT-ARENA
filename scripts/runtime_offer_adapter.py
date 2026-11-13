"""runtime 확장 offer를 strict compact 핵심과 원 관측으로 분리하는 순수 어댑터.

actuation_wire.Decoder가 검증한 payload 객체를 받는다. ROS/serial/시계 매핑,
세션 상태, ARM/DRIVE 허가는 소유하지 않는다. 알 수 없는 필드는 버리지 않는다.
"""
from dataclasses import dataclass
import json

from actuation_host_observer import OFFER_KEYS, SCHEMAS, boolean, signed, validate_payload, wire_text
from arena_vehicle_interface.actuation_wire import MAX_PAYLOAD
from arena_vehicle_interface.clock_bounds import integer


STATUS_KEYS = frozenset({'status_now_us', 'estop_held', 'reason'})
FEEDBACK_KEY = 'motor_feedback_v1'
FEEDBACK_KEYS = frozenset({'valid', 'sequence', 'capture_us', 'counts'})


@dataclass(frozen=True, slots=True)
class RuntimePayloadParts:
    format: str
    compact: dict
    status: dict | None = None
    feedback: dict | None = None


def _feedback(raw, compact, now):
    # decode_motor_feedback의 원 관측 계약과 동일하다. 그 함수가 요구하는
    # host 추정 시각을 여기서 꾸며 만들지 않고 MCU 단위 원값만 반환한다.
    if type(raw) is not dict or set(raw) != FEEDBACK_KEYS:
        raise ValueError('motor_feedback_schema')
    boolean(raw['valid'])
    sequence, capture, counts = (raw[key] for key in ('sequence', 'capture_us', 'counts'))
    absent = sequence is None and capture is None and counts is None
    if absent:
        if raw['valid']:
            raise ValueError('motor_feedback_missing_valid_sample')
    else:
        integer(sequence, 'motor feedback sequence')
        integer(capture, 'motor feedback capture')
        signed(counts)
        if capture > now:
            raise ValueError('motor_feedback_future_capture')
    return dict(schema='runtime_motor_feedback_raw_v1', boot=compact['boot'],
                generation=compact['generation'], lease=compact['lease'],
                status_now_us=now, source_valid=raw['valid'], sequence=sequence,
                mcu_capture_us=capture, motor_counts=counts,
                time_basis='mcu_monotonic_us', time_contract_verified=False,
                scope='raw_observation_only_no_drive')


def adapt_runtime_payload(payload, *, max_lease_us):
    """정확한 compact 또는 두 runtime offer 형식만 허용한다.

    status/feedback의 boot·generation·기준 시각은 같은 한 offer에서 가져온다.
    외부 상태와 합치거나 미래 capture/unknown 필드를 정규화하지 않는다.
    반환 dict들은 입력 및 다른 반환 dict와 mutable 객체를 공유하지 않는다.
    """
    integer(max_lease_us, 'maximum lease', 10_000_000)
    if not max_lease_us:
        raise ValueError('positive maximum lease required')
    if type(payload) is not dict:
        raise ValueError('runtime payload object required')
    kind = payload.get('type')
    if type(kind) is not str or kind not in SCHEMAS:
        raise ValueError('unknown runtime payload type')
    keys = set(payload)
    if keys == SCHEMAS[kind]:
        _, canonical = validate_payload(payload, max_lease_us)
        return RuntimePayloadParts('compact_v1', json.loads(canonical))
    status_keys = OFFER_KEYS | STATUS_KEYS
    if kind != 'offer' or keys not in (status_keys, status_keys | {FEEDBACK_KEY}):
        raise ValueError('unknown or incomplete runtime offer fields')

    # 허용 목록을 먼저 고정한 뒤에만 핵심을 분리한다. compact validator는 그대로다.
    _, canonical = validate_payload({key: payload[key] for key in OFFER_KEYS}, max_lease_us)
    compact = json.loads(canonical)
    now = integer(payload['status_now_us'], 'status now')
    if now != compact['lease_issued_us']:
        raise ValueError('runtime status and lease issue time differ')
    boolean(payload['estop_held'])
    wire_text(payload['reason'])
    if len(payload['reason']) > 32:
        raise ValueError('runtime reason exceeds firmware text bound')
    status = dict(schema='runtime_offer_status_v1', boot=compact['boot'],
                  generation=compact['generation'], lease=compact['lease'],
                  state=compact['state'], status_now_us=now,
                  estop_held=payload['estop_held'], reason=payload['reason'],
                  output_ready=compact['output_ready'], scope='observation_only_no_drive')
    feedback = _feedback(payload[FEEDBACK_KEY], compact, now) if FEEDBACK_KEY in payload else None
    # 핵심만 작다고 원 확장 프레임의 1536-byte 한도를 빠뜨리지 않는다.
    full = json.dumps(payload, separators=(',', ':'), ensure_ascii=True, allow_nan=False)
    if len(full.encode('ascii')) > MAX_PAYLOAD:
        raise ValueError('runtime payload limit exceeded')
    return RuntimePayloadParts('runtime_offer_motor_v1' if feedback is not None else 'runtime_offer_status_v1',
                               compact, status, feedback)
