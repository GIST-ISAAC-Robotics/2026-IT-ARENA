"""compact 호스트 요청/응답의 출력 없는 후보. ROS·serial·구동 생성과 미연결.

반환하는 제어 envelope는 시험기/후속 전송 어댑터가 별도로 송신해야 한다.
ACK 상관 확인과 새 세대 offer는 시간 신선도/물리 정지의 대체가 아니다.
"""
from collections import deque
from dataclasses import dataclass
import uuid

from actuation_host_observer import CompactHostObserver, Receipt, wire_text
from actuation_source_time_review import review_source_time
from arena_vehicle_interface.clock_bounds import integer
from arena_vehicle_interface.clock_freshness import HostTimeWindow

CONTROL_REASONS = dict(ARM='explicit_arm', CLEAR='explicit_clear',
                       DISARM='explicit_disarm', STOP='software_stop')
ARMED = {'ARMED', 'ACTIVE'}


@dataclass(frozen=True, slots=True)
class PendingControl:
    request: str
    kind: str
    started: HostTimeWindow
    deadline_ns: int
    base_generation: int | None


class CompactHostSession:
    """연결 1회·단일 소유자. 미확인 제어 1개, 결과 이력 32개로 한정한다.

    STOP은 기존 요청을 선점하고 ARM 증거를 즉시 버린다. 시간/연결 변경에는
    새 객체가 필요하며, 객체별 UUID 요청 접두부를 재사용하지 않는다(인증 아님).
    """
    def __init__(self, *, boot, host_epoch, connection_id, client_id, max_lease_us,
                 request_timeout_ns):
        wire_text(client_id)
        integer(request_timeout_ns, 'request timeout', 60_000_000_000)
        if not request_timeout_ns:
            raise ValueError('positive request timeout required')
        self._observer = CompactHostObserver(boot=boot, host_epoch=host_epoch,
            connection_id=connection_id, max_lease_us=max_lease_us)
        self.client_id = client_id
        self.request_timeout_ns = request_timeout_ns
        self._prefix = uuid.uuid4().hex
        self._sequence = 0
        self._pending = None
        self._ack = self._confirmation = None
        self._arm_generation = self._arm_barrier = None
        self._last_event = self._source_watermark = None
        self._closed = False
        self._stop_requested = False
        self._outcomes = deque(maxlen=32)

    @property
    def pending(self):
        return self._pending

    @property
    def arm_barrier(self):
        return self._arm_barrier

    @property
    def closed(self):
        return self._closed

    @property
    def stop_requested(self):
        return self._stop_requested

    @property
    def outcomes(self):
        return tuple(dict(row) for row in self._outcomes)

    def latest(self, kind):
        return self._observer.latest(kind)

    @staticmethod
    def _result(reason, **details):
        return dict(reason=reason, ready_to_drive=False, runtime_connected=False, **details)

    def _revoke(self):
        self._arm_generation = self._arm_barrier = None
        # 이전 source watermark는 STOP/CLEAR/ARM으로 되돌리지 않는다.

    def _finish(self, outcome):
        if self._pending is not None:
            self._outcomes.append(dict(request=self._pending.request, kind=self._pending.kind,
                                       outcome=outcome))
        self._pending = self._ack = self._confirmation = None

    def disconnect(self, reason='disconnected_unconfirmed'):
        self._closed = True
        self._revoke()
        self._finish(reason)
        self._observer.disconnect()

    def _advance(self, now):
        if not isinstance(now, HostTimeWindow):
            raise ValueError('typed host time window required')
        if self.closed:
            return False
        if now.host_epoch != self._observer.host_epoch:
            self.disconnect('host_epoch_changed_unconfirmed')
            return False
        if self._last_event is not None and (now.lower_ns < self._last_event.lower_ns
                                            or now.upper_ns < self._last_event.upper_ns):
            self.disconnect('host_clock_regressed_unconfirmed')
            return False
        self._last_event = now
        # 불확실한 구간의 일부라도 기한에 닿으면 허가 증거로 쓰지 않는다.
        if self.pending is not None and now.upper_ns >= self.pending.deadline_ns:
            self._revoke()
            self._finish('timeout_unconfirmed')
        return True

    def poll(self, now):
        self._advance(now)
        return self._result('closed' if self.closed else 'polled', pending=self.pending is not None)

    def begin_control(self, kind, now, *, fit=None, max_issue_age_ns=None,
                      forward_delay_upper_ns=None, policy_note=None):
        """명시 요청을 기록하고 envelope를 반환한다. 송신/자동 재시도는 하지 않는다."""
        if type(kind) is not str or kind not in CONTROL_REASONS:
            raise ValueError('explicit known control kind required')
        self._revoke()
        if kind == 'STOP':
            self._stop_requested = True
            self._finish('superseded_by_stop_unconfirmed')
        if not self._advance(now):
            return self._result('closed', envelope=None)
        if self.pending is not None:
            return self._result('control_pending', envelope=None)
        if kind == 'ARM' and self._stop_requested:
            return self._result('explicit_clear_required', envelope=None)
        receipt = self.latest('offer')
        base = self._observer.generation
        if kind != 'STOP':
            if receipt is None or now.lower_ns < receipt.received.upper_ns:
                return self._result('no_certain_current_offer', envelope=None)
            offer = receipt.payload
            states = {'ARM': {'DISARMED'}, 'CLEAR': {'ESTOP_LATCHED', 'FAULT_LATCHED'}, 'DISARM': ARMED}
            if offer['state'] not in states[kind]:
                return self._result('state_incompatible', envelope=None)
            if ((kind == 'ARM' and not offer['output_ready'])
                    or (kind == 'DISARM' and offer['owner'] != self.client_id)):
                return self._result('owner_or_output_not_ready', envelope=None)
            try:
                timing = self._observer.review_offer_time(fit, now,
                    max_issue_age_ns=max_issue_age_ns, forward_delay_upper_ns=forward_delay_upper_ns,
                    policy_note=policy_note)
            except (ValueError, TypeError, AttributeError, OverflowError):
                return self._result('time_evidence_invalid', envelope=None)
            if not timing['timestamp_policy_satisfied']:
                return self._result('time_conditions_unsatisfied', envelope=None)
            if base >= 2**63-1:
                self.disconnect('generation_exhausted')
                return self._result('generation_exhausted', envelope=None)
        if self._sequence >= 2**32-1:
            self.disconnect('request_sequence_exhausted')
            return self._result('request_sequence_exhausted', envelope=None)
        deadline = now.lower_ns+self.request_timeout_ns
        integer(deadline, 'request deadline')
        if now.upper_ns >= deadline:
            return self._result('request_time_uncertain', envelope=None)
        self._sequence += 1
        request = f'{self._prefix}:{self._sequence:x}'
        command = dict(v=1, kind=kind)
        if kind != 'STOP':
            command.update(boot=receipt.payload['boot'], generation=base,
                           lease=receipt.payload['lease'], client=self.client_id, seq=self._sequence)
        self._revoke()
        self._pending = PendingControl(request, kind, now, deadline, base)
        return self._result('recorded_not_sent', envelope=dict(type='command', request=request, command=command))

    def observe(self, payload, *, received, connection_id):
        # 옛 연결에서 늦게 배달된 callback은 현재 시계를 갱신하지 않는다.
        if connection_id != self._observer.connection_id:
            return self._result('connection_mismatch', ack_correlated=False)
        if not self._advance(received):
            return self._result('closed', ack_correlated=False)
        result = self._observer.observe(payload, received=received, connection_id=connection_id)
        if self._observer.closed or result['reason'] == 'boot_mismatch':
            self.disconnect('session_changed_unconfirmed')
            return self._result('closed', ack_correlated=False)
        # 거절된 새 세대 offer도 observer의 세대를 바꿀 수 있다. 옛 허가를 유지하지 않는다.
        generation = self._observer.generation
        if self._arm_generation is not None and generation != self._arm_generation:
            self._revoke()
        p = self.pending
        if p is not None and p.kind != 'STOP' and generation not in (p.base_generation, p.base_generation+1):
            self._revoke()
            self._finish('generation_changed_unconfirmed')
        if not result['accepted']:
            return self._result(result['reason'], ack_correlated=False)
        kind = payload['type']
        if kind == 'ack':
            return self._consume_ack(payload, received)
        if kind in ('offer', 'state', 'diag'):
            self._consume_state(kind, payload, received)
        self._confirm()
        return self._result('observed', ack_correlated=False)

    def _consume_ack(self, payload, received):
        p = self.pending
        if p is None or payload['request'] != p.request:
            return self._result('unmatched_ack', ack_correlated=False)
        if received.lower_ns < p.started.upper_ns:
            self._revoke()
            self._finish('ack_order_uncertain')
            return self._result('ack_order_uncertain', ack_correlated=False)
        if self._ack is not None:
            # 같은 ACK를 받아도 기한/ARM 기준을 갱신하지 않는다.
            if not payload['accepted'] or payload['reason'] != CONTROL_REASONS[p.kind]:
                self._revoke()
                self._finish('conflicting_ack_unconfirmed')
                return self._result('conflicting_ack_unconfirmed', ack_correlated=False)
            return self._result('duplicate_ack', ack_correlated=False)
        if not payload['accepted']:
            self._revoke()
            self._finish('rejected')
            return self._result('rejected', ack_correlated=True)
        if payload['reason'] != CONTROL_REASONS[p.kind]:
            self._revoke()
            self._finish('ack_reason_mismatch')
            return self._result('ack_reason_mismatch', ack_correlated=False)
        self._ack = received
        if p.kind == 'STOP':
            self._finish('stop_ack_not_physical_stop')
        else:
            self._confirm()
        return self._result('ack_recorded', ack_correlated=True)

    def _consume_state(self, kind, payload, received):
        state = payload['state']
        if self._arm_generation is not None:
            if (state not in ARMED or (kind == 'offer' and payload['owner'] != self.client_id)
                    or (kind in ('offer', 'diag') and not payload['output_ready'])):
                self._revoke()
        p = self.pending
        if p is None or p.kind == 'STOP' or payload['generation'] != p.base_generation+1:
            return
        expected = ARMED if p.kind == 'ARM' else {'DISARMED'}
        compatible = state in expected
        if p.kind == 'ARM' and kind in ('offer', 'diag') and not payload['output_ready']:
            compatible = False
        if p.kind == 'ARM' and kind == 'offer' and payload['owner'] != self.client_id:
            compatible = False
        if not compatible:
            self._revoke()
            self._finish('transition_incompatible_unconfirmed')
        elif kind == 'offer' and received.lower_ns >= p.started.upper_ns:
            self._confirmation = Receipt(self.latest('offer').payload_json, received,
                                         self._observer.connection_id)

    def _confirm(self):
        p = self.pending
        if p is None or self._ack is None or self._confirmation is None:
            return
        if p.kind == 'ARM':
            self._arm_generation = p.base_generation+1
            self._arm_barrier = HostTimeWindow(self._observer.host_epoch,
                max(self._ack.lower_ns, self._confirmation.received.lower_ns),
                max(self._ack.upper_ns, self._confirmation.received.upper_ns))
        elif p.kind == 'CLEAR':
            self._stop_requested = False
        self._finish('transition_evidenced_not_output_permission')

    def review_source(self, fit, now, source, *, max_issue_age_ns, forward_delay_upper_ns,
                      source_ttl_ns, policy_note):
        """원천 시간/단조성까지 조건을 검토하고 성공한 원천만 소비한다. DRIVE 생성 없음."""
        if not self._advance(now):
            return self._result('closed', session_conditions_satisfied=False, command_created=False)
        receipt = self.latest('offer')
        if (self._stop_requested or self._arm_barrier is None or receipt is None or self.pending is not None
                or receipt.payload['generation'] != self._arm_generation
                or receipt.payload['state'] not in ARMED or receipt.payload['owner'] != self.client_id
                or not receipt.payload['output_ready'] or now.lower_ns < receipt.received.upper_ns):
            return self._result('arm_evidence_missing', session_conditions_satisfied=False, command_created=False)
        if not isinstance(source, HostTimeWindow):
            raise ValueError('typed source window required')
        # 불확실한 구간이 겹치면 순서를 확정할 수 없다. 같은 시각도 재사용하지 않는다.
        if False and self._source_watermark is not None and source.lower_ns <= self._source_watermark.upper_ns:
            return self._result('source_repeated_or_order_uncertain', session_conditions_satisfied=False,
                                command_created=False)
        try:
            timing = review_source_time(receipt.payload, fit, receipt.received, now, source, self._arm_barrier,
                mapping_host_epoch=self._observer.host_epoch, expected_boot=self._observer.boot,
                expected_generation=self._arm_generation, max_lease_us=self._observer.max_lease_us,
                max_issue_age_ns=max_issue_age_ns, forward_delay_upper_ns=forward_delay_upper_ns,
                source_ttl_ns=source_ttl_ns, policy_note=policy_note)
        except (ValueError, TypeError, AttributeError, OverflowError):
            return self._result('time_evidence_invalid', session_conditions_satisfied=False, command_created=False)
        satisfied = timing['timestamp_policy_satisfied']
        if satisfied:
            self._source_watermark = source
        return self._result('conditions_reviewed', session_conditions_satisfied=satisfied,
            arm_ack_correlated=True, source_watermark_checked=True, timing=timing, command_created=False)
