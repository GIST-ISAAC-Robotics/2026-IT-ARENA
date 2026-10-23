"""compact MCU 메시지의 출력 없는 호스트 보관 후보. 운영 ROS/serial과 미연결.

프레이밍/CRC를 통과한 payload만 받는다. ACK·state·diag가 offer 수신 시각이나
유효기간을 갱신하지 않는다. 외부에서 확인한 boot/host epoch/연결 ID를 요구하며
이 객체가 세션을 인증하거나 ARM/DRIVE/STOP을 송신하지는 않는다.
"""
from collections import deque
from dataclasses import dataclass
import json

from audit_actuation_offer_clock import KEYS as OFFER_KEYS, STATES, assess_offer, text_id
from arena_vehicle_interface.clock_bounds import integer
from arena_vehicle_interface.clock_freshness import HostTimeWindow

LIMIT = 2**63-1
STATE_KEYS = {'type', 'v', 'boot', 'generation', 'state', 'reason', 'changes'}
ACK_KEYS = {'type', 'request', 'accepted', 'reason'}
TX_KEYS = {'ack_dropped', 'ack_evicted', 'ack_purged', 'state_merged', 'diag_skipped',
           'aborted_frames', 'encode_failures'}
DIAG_KEYS = {'type', 'v', 'boot', 'now_us', 'generation', 'state', 'reason', 'accepted_seq',
             'target_speed_umps', 'steering_urad', 'deadline_us', 'estop_held', 'feedback_mode',
             'feedback_valid', 'feedback_seq', 'feedback_capture_us', 'motor_counts',
             'left_ticks', 'right_ticks', 'mean_wheel_umps', 'output_ready',
             'deadline_miss_count', 'lease_count', 'tx'}
SCHEMAS = {'offer': OFFER_KEYS, 'state': STATE_KEYS, 'ack': ACK_KEYS, 'diag': DIAG_KEYS}


def signed(value):
    if type(value) is not int or not -2**63 <= value <= LIMIT:
        raise ValueError('bounded signed integer required')


def boolean(value):
    if type(value) is not bool:
        raise ValueError('boolean required')


def wire_text(value):
    # StoredString의 1..64 Unicode scalar 계약. 공백도 wire에서는 문자열이다.
    if (type(value) is not str or not 1 <= len(value) <= 64
            or any(0xD800 <= ord(char) <= 0xDFFF for char in value)):
        raise ValueError('one through 64 Unicode scalars required')


def validate_payload(payload, max_lease_us):
    """후보 wire v1 형식을 고정한다. 알 수 없는 필드를 조용히 버리지 않는다."""
    if type(payload) is not dict:
        raise ValueError('object required')
    kind = payload.get('type')
    if type(kind) is not str or kind not in SCHEMAS or set(payload) != SCHEMAS[kind]:
        raise ValueError('exact known schema required')
    if kind == 'ack':
        wire_text(payload['request'])
        wire_text(payload['reason'])
        boolean(payload['accepted'])
    else:
        if type(payload['v']) is not int or payload['v'] != 1:
            raise ValueError('v1 required')
        wire_text(payload['boot'])
        integer(payload['generation'], 'generation')
        if type(payload['state']) is not str or payload['state'] not in STATES:
            raise ValueError('known state required')
        if kind == 'offer':
            for name in ('lease', 'lease_issued_us', 'lease_expires_us'):
                integer(payload[name], name)
            if not payload['lease'] or not 0 < payload['lease_expires_us']-payload['lease_issued_us'] <= max_lease_us:
                raise ValueError('positive bounded lease required')
            boolean(payload['output_ready'])
            if payload['owner'] is not None:
                wire_text(payload['owner'])
            if (payload['state'] in {'ARMED', 'ACTIVE'}) != (payload['owner'] is not None):
                raise ValueError('state/owner inconsistency')
        elif kind == 'state':
            wire_text(payload['reason'])
            integer(payload['changes'], 'changes')
        else:
            wire_text(payload['reason'])
            for name in ('now_us', 'deadline_miss_count', 'lease_count'):
                integer(payload[name], name)
            if payload['lease_count'] > 32:
                raise ValueError('lease table bound exceeded')
            signed(payload['accepted_seq'])
            for name in ('estop_held', 'feedback_valid', 'output_ready'):
                boolean(payload[name])
            if payload['feedback_mode'] not in ('drive_motor_shaft', 'rear_wheel_pair_legacy'):
                raise ValueError('known feedback mode required')
            for name in ('feedback_seq', 'feedback_capture_us', 'deadline_us'):
                if payload[name] is not None:
                    integer(payload[name], name)
            for name in ('target_speed_umps', 'steering_urad', 'mean_wheel_umps', 'motor_counts', 'left_ticks', 'right_ticks'):
                if payload[name] is not None:
                    signed(payload[name])
            if type(payload['tx']) is not dict or set(payload['tx']) != TX_KEYS:
                raise ValueError('exact TX counters required')
            for name, value in payload['tx'].items():
                integer(value, name)
    canonical = json.dumps(payload, separators=(',', ':'), ensure_ascii=True, allow_nan=False)
    if len(canonical.encode('ascii')) > 1536:
        raise ValueError('payload limit exceeded')
    return kind, canonical


@dataclass(frozen=True, slots=True)
class Receipt:
    payload_json: str
    received: HostTimeWindow
    connection_id: str

    @property
    def payload(self):
        # 호출자가 수정해도 저장본과 과거 증거는 변하지 않는다.
        return json.loads(self.payload_json)


class CompactHostObserver:
    """연결 1회용·단일 소유자. 재연결/host 재시작에는 새 ID의 새 객체를 만든다.

    ACK에는 boot/generation이 없으므로 상관 확인되지 않은 진단으로만 보관한다.
    generation/lease/MCU 시각 단조성은 인증이나 최신성의 증명이 아니다.
    """
    def __init__(self, *, boot, host_epoch, connection_id, max_lease_us, history_limit=32):
        self.boot = text_id(boot, 'boot')
        self.host_epoch = text_id(host_epoch, 'host epoch')
        self.connection_id = text_id(connection_id, 'connection')
        integer(max_lease_us, 'maximum lease', 10_000_000)
        integer(history_limit, 'history limit', 128)
        if not max_lease_us or not history_limit:
            raise ValueError('positive bounds required')
        self.max_lease_us = max_lease_us
        self.history = deque(maxlen=history_limit)
        self._latest = {}
        self._last_receive = None
        self._last_lease = 0
        self._last_issue = None
        self._last_diag = None
        self.generation = None
        self.closed = False
        self.accepted = self.rejected = self.history_dropped = 0

    def _result(self, accepted, reason, kind, received):
        if len(self.history) == self.history.maxlen:
            self.history_dropped = min(LIMIT, self.history_dropped+1)
        self.history.append(dict(accepted=accepted, reason=reason, kind=kind,
                                 received_lower_ns=received.lower_ns, received_upper_ns=received.upper_ns))
        if accepted:
            self.accepted = min(LIMIT, self.accepted+1)
        else:
            self.rejected = min(LIMIT, self.rejected+1)
        return dict(accepted=accepted, reason=reason, ready_to_drive=False,
                    runtime_connected=False, ack_correlated=False)

    def latest(self, kind):
        return self._latest.get(kind)  # Receipt는 frozen이며 payload는 매번 사본이다.

    def disconnect(self):
        self.closed = True
        self._latest.clear()

    def observe(self, payload, *, received, connection_id):
        if not isinstance(received, HostTimeWindow):
            raise ValueError('explicit receipt window required')
        if self.closed:
            return self._result(False, 'closed', None, received)
        if received.host_epoch != self.host_epoch or connection_id != self.connection_id:
            return self._result(False, 'context_mismatch', None, received)
        if self._last_receive is not None and (received.lower_ns < self._last_receive.lower_ns
                                               or received.upper_ns < self._last_receive.upper_ns):
            self.disconnect()
            return self._result(False, 'clock_regressed', None, received)
        self._last_receive = received
        try:
            kind, canonical = validate_payload(payload, self.max_lease_us)
        except (ValueError, TypeError, OverflowError, RecursionError):
            return self._result(False, 'schema', None, received)
        if kind != 'ack':
            if payload['boot'] != self.boot:
                return self._result(False, 'boot_mismatch', kind, received)
            generation = payload['generation']
            if self.generation is not None and generation < self.generation:
                return self._result(False, 'old_generation', kind, received)
            if generation != self.generation:
                # 새 세대 state/diag만 와도 옛 offer를 더 이상 내주지 않는다.
                self.generation = generation
                for previous_kind in ('offer', 'state', 'diag'):
                    self._latest.pop(previous_kind, None)
            if kind == 'offer':
                if payload['lease'] <= self._last_lease:
                    return self._result(False, 'duplicate_or_old_lease', kind, received)
                if self._last_issue is not None and payload['lease_issued_us'] < self._last_issue:
                    return self._result(False, 'issue_regressed', kind, received)
                self._last_lease, self._last_issue = payload['lease'], payload['lease_issued_us']
            elif kind == 'diag':
                if self._last_diag is not None and payload['now_us'] <= self._last_diag:
                    return self._result(False, 'duplicate_or_old_diag', kind, received)
                self._last_diag = payload['now_us']
        self._latest[kind] = Receipt(canonical, received, self.connection_id)
        return self._result(True, 'recorded', kind, received)

    def review_offer_time(self, fit, now, *, max_issue_age_ns, forward_delay_upper_ns, policy_note):
        receipt = self.latest('offer')
        if self.closed or receipt is None:
            return dict(status='no_current_offer', timestamp_policy_satisfied=False,
                        ready_to_drive=False, runtime_connected=False)
        if not isinstance(now, HostTimeWindow) or now.lower_ns < self._last_receive.upper_ns:
            raise ValueError('decision precedes observed receipt')
        return assess_offer(receipt.payload, fit, receipt.received, now,
                            mapping_host_epoch=self.host_epoch, expected_boot=self.boot,
                            expected_generation=self.generation, max_lease_us=self.max_lease_us,
                            max_issue_age_ns=max_issue_age_ns, forward_delay_upper_ns=forward_delay_upper_ns,
                            policy_note=policy_note)
