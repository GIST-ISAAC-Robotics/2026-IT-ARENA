"""단일 소유 MCU 연결. ROS/장치 열기·자동 ARM 없이 /drive/safe 명령만 준비한다."""
from collections import Counter, deque
import queue
import threading
import uuid

from arena_vehicle_interface.actuation_wire import Decoder, encode
from arena_vehicle_interface.clock_freshness import HostTimeWindow
from arena_vehicle_interface.serial_ingress import SerialIngress
from receiver_owned_actuation_host import ReceiverLeaseHostSession
from runtime_offer_adapter import adapt_runtime_payload


class ActuationMailbox:
    """STOP 우선, 제어 최대8개, 미송신 DRIVE 최신1개. 이미 쓰는 바이트는 취소 못한다."""
    def __init__(self):
        self._lock = threading.Lock()
        self._controls = deque()
        self._drive = self._stop = None
        self.counts = Counter()

    def put_nowait(self, packet):
        if type(packet) is not bytes or not 1 <= len(packet) <= 1544:
            raise ValueError('bounded immutable command wire required')
        decoder = Decoder()
        rows = decoder.feed(packet)
        if decoder.errors or decoder.buffer or len(rows) != 1 or encode(rows[0]) != packet:
            raise ValueError('one canonical complete command frame required')
        row = rows[0]
        command = row.get('command')
        if (set(row) != {'type', 'request', 'command'} or row['type'] != 'command'
                or type(command) is not dict or command.get('kind') not in
                ('DRIVE', 'ARM', 'CLEAR', 'DISARM', 'STOP')):
            raise ValueError('known actuation command required')
        kind = command['kind']
        with self._lock:
            if kind == 'STOP':
                self.counts['stop_purged_controls'] += len(self._controls)
                self.counts['stop_purged_drive'] += self._drive is not None
                self.counts['stop_replaced'] += self._stop is not None
                self._controls.clear()
                self._drive = None
                self._stop = packet
            elif kind == 'DRIVE':
                if self._stop is not None:
                    raise RuntimeError('STOP transmission pending')
                self.counts['drive_replaced'] += self._drive is not None
                self._drive = packet
            else:
                if len(self._controls) >= 8:
                    raise queue.Full
                self.counts['control_purged_drive'] += self._drive is not None
                self._drive = None
                self._controls.append(packet)
            self.counts['queued_' + kind] += 1

    def get_nowait(self):
        with self._lock:
            if self._stop is not None:
                value, self._stop = self._stop, None
                return value
            if self._controls:
                return self._controls.popleft()
            if self._drive is not None:
                value, self._drive = self._drive, None
                return value
            raise queue.Empty


class ActuationSerialIngress(SerialIngress):
    """검증된 I/O 작업·receipt 시각 보존을 재사용하고 송신 대기열만 교체한다."""
    def __init__(self, port, *, capacity=512, factory=None):
        super().__init__(port, capacity=capacity, factory=factory)
        self.commands = ActuationMailbox()


class RuntimeActuationLink:
    """한 연결·한 호스트 작업용. 재접속/MCU 재시작 뒤 ARM을 자동 복원하지 않는다."""
    def __init__(self, *, host_epoch, client_id, timing, max_speed_mps,
                 max_steering_rad, max_lease_us, request_timeout_ns, drive_enabled=False):
        if type(drive_enabled) is not bool:
            raise ValueError('explicit drive enable boolean required')
        self.connection_id = uuid.uuid4().hex
        self.options = dict(host_epoch=host_epoch, connection_id=self.connection_id,
            client_id=client_id, timing=timing, max_speed_mps=max_speed_mps,
            max_steering_rad=max_steering_rad, max_lease_us=max_lease_us,
            request_timeout_ns=request_timeout_ns)
        ReceiverLeaseHostSession(boot='validation-only', **self.options)
        self.session = None
        self.drive_enabled = drive_enabled
        self.closed = False
        self.counts = Counter()

    def window(self, monotonic_ns):
        return HostTimeWindow(self.options['host_epoch'], monotonic_ns, monotonic_ns)

    def observe(self, payload, *, receipt_ns, processed_ns):
        if self.closed:
            return dict(accepted=False, reason='connection_closed')
        parts = adapt_runtime_payload(payload, max_lease_us=self.options['max_lease_us'])
        row = parts.compact
        if self.session is None:
            if row['type'] != 'offer':
                return dict(accepted=False, reason='initial_offer_required', parts=parts)
            self.session = ReceiverLeaseHostSession(boot=row['boot'], **self.options)
        result = self.session.observe(row, received=self.window(receipt_ns),
            processed=self.window(processed_ns), connection_id=self.connection_id)
        if self.session.closed:
            self.closed = True
        self.counts['rx_' + row['type']] += 1
        return dict(result, parts=parts)

    def control(self, kind, now_ns):
        if kind not in ('ARM', 'CLEAR', 'DISARM', 'STOP'):
            raise ValueError('explicit control required')
        if kind == 'STOP' and (self.session is None or self.closed):
            row = dict(type='command', request='stop-' + uuid.uuid4().hex,
                       command=dict(v=1, kind='STOP'))
            return dict(reason='stop_prepared_unconfirmed', envelope=row, wire=encode(row))
        if self.closed or self.session is None:
            return dict(reason='current_offer_required', envelope=None)
        if kind == 'ARM' and not self.drive_enabled:
            return dict(reason='drive_disabled_by_host', envelope=None)
        result = self.session.begin_control(kind, self.window(now_ns))
        if self.session.closed:
            self.closed = True
            if kind == 'STOP':
                return self.control(kind, now_ns)
        if result.get('envelope') is not None:
            result['wire'] = encode(result['envelope'])
        return result

    def drive(self, *, speed_mps, steering_rad, source, now_ns):
        if self.closed or self.session is None or not self.drive_enabled:
            return dict(reason='drive_disabled_or_unconnected', command_created=False)
        result = self.session.prepare_drive(speed_mps=speed_mps, steering_rad=steering_rad,
                                            source=source, now=self.window(now_ns))
        self.counts['drive_prepared' if result['command_created'] else 'drive_rejected'] += 1
        return result

    def poll(self, now_ns):
        if self.session is not None and not self.closed:
            self.session.poll(self.window(now_ns))
            self.closed = self.session.closed

    def close(self, reason='connection_closed'):
        self.closed = True
        if self.session is not None:
            self.session.disconnect(reason)
