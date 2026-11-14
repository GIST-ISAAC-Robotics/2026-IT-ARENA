"""UNO Q ESV1: 기존 IA JSON 명령과 별개인 수신기 lease 한정 서보 상태."""
from collections import Counter, deque
import queue
import re
import struct
import threading
import zlib

from arena_vehicle_interface.actuation_wire import Decoder, encode
from arena_vehicle_interface.serial_ingress import SerialIngress

ESV = struct.Struct('<4s8sQQQIBq')
BENCH = struct.Struct('<4s8sQQQBBHI')
BAUDRATE = 1_000_000
MAX_OBSERVATION_AGE_US = 100_000


def encode_status(*, boot, generation, lease, status_seq, observation_age_us, ready, applied_drive_seq):
    if not isinstance(boot, str) or re.fullmatch('[0-9a-fA-F]{8}', boot) is None:
        raise ValueError('ESV1 requires the MCU eight-hex boot identifier')
    for name, value in (('generation', generation), ('lease', lease), ('status_seq', status_seq)):
        if type(value) is not int or not 0 <= value < 2**63:
            raise ValueError('bounded ESV1 '+name+' required')
    if not lease or not status_seq:
        raise ValueError('positive lease and boot-wide status sequence required')
    if type(observation_age_us) is not int or not 0 <= observation_age_us < 2**32:
        raise ValueError('uint32 observation age required')
    if type(ready) is not bool or (ready and observation_age_us >= MAX_OBSERVATION_AGE_US):
        raise ValueError('fresh boolean readiness required')
    if type(applied_drive_seq) is not int or not -1 <= applied_drive_seq < 2**63:
        raise ValueError('applied DRIVE sequence or -1 required')
    payload = ESV.pack(b'ESV1', boot.encode('ascii'), generation, lease, status_seq,
                       observation_age_us, ready, applied_drive_seq)
    body = struct.pack('<H', len(payload)) + payload
    return b'IA' + body + struct.pack('<I', zlib.crc32(body))


def decode_status(wire):
    if (type(wire) is not bytes or len(wire) != ESV.size+8 or wire[:4] != b'IA1\x00'
            or zlib.crc32(wire[2:-4]) != struct.unpack('<I', wire[-4:])[0]):
        raise ValueError('one complete CRC-checked ESV1 frame required')
    magic, boot, generation, lease, seq, age, ready, applied = ESV.unpack(wire[4:-4])
    if magic != b'ESV1' or ready not in (0, 1):
        raise ValueError('ESV1 magic/readiness invalid')
    row = dict(boot=boot.decode('ascii'), generation=generation, lease=lease,
               status_seq=seq, observation_age_us=age, ready=bool(ready), applied_drive_seq=applied)
    if encode_status(**row) != wire:
        raise ValueError('noncanonical ESV1 frame')
    return row


def encode_bench(*, boot, generation, lease, sequence, action, direction=0, duty_permille=0, duration_us=0):
    # 같은 식별자 표현 범위. action STOP도 유한 정상 frame으로 만든다.
    encode_status(boot=boot, generation=generation, lease=lease, status_seq=sequence,
                  observation_age_us=0, ready=False, applied_drive_seq=-1)
    if type(action) is not int or action not in (1, 2, 3, 4) or type(direction) is not int or direction not in (0, 1):
        raise ValueError('known bench action and explicit DIR level required')
    if (type(duty_permille) is not int or type(duration_us) is not int
            or not 0 <= duty_permille <= 150 or not 0 <= duration_us <= 5_000_000):
        raise ValueError('bench bound is 150 permille and 5 s')
    if (action == 2 and (duty_permille == 0 or duration_us == 0)) or (
            action != 2 and (duty_permille != 0 or duration_us != 0)):
        raise ValueError('PULSE alone carries nonzero duty/duration')
    payload = BENCH.pack(b'BNV1', boot.encode('ascii'), generation, lease, sequence,
                         action, direction, duty_permille, duration_us)
    body = struct.pack('<H', len(payload))+payload
    return b'IA'+body+struct.pack('<I', zlib.crc32(body))


def decode_bench(wire):
    if (type(wire) is not bytes or len(wire) != BENCH.size+8 or wire[:4] != b'IA,\x00'
            or zlib.crc32(wire[2:-4]) != struct.unpack('<I', wire[-4:])[0]):
        raise ValueError('one CRC-checked BNV1 frame required')
    magic, boot, generation, lease, seq, action, direction, duty, duration = BENCH.unpack(wire[4:-4])
    if magic != b'BNV1':raise ValueError('BNV1 magic')
    row = dict(boot=boot.decode('ascii'), generation=generation, lease=lease, sequence=seq,
               action=action, direction=direction, duty_permille=duty, duration_us=duration)
    if encode_bench(**row) != wire:raise ValueError('noncanonical bench frame')
    return row


class UnoMailbox:
    """STOP 우선, 제어 8개, 최신 DRIVE/ESV 각 1개. 이미 쓰는 frame은 취소 불가."""
    def __init__(self):
        self.lock = threading.Lock()
        self.controls = deque()
        self.drive = self.status = self.stop = self.keepalive = None
        self.counts = Counter()

    def put_nowait(self, packet):
        if type(packet) is not bytes or not 1 <= len(packet) <= 1544:
            raise ValueError('bounded immutable frame required')
        external = packet[4:8] == b'ESV1'
        if packet[4:8] == b'BNV1':
            row = decode_bench(packet)
            kind = {1: 'BEN_ARM', 2: 'BEN_PULSE', 3: 'STOP', 4: 'BEN_KEEPALIVE'}[row['action']]
        elif external:
            decode_status(packet)
            kind = 'ESV'
        else:
            decoder = Decoder()
            rows = decoder.feed(packet)
            if decoder.errors or decoder.buffer or len(rows) != 1 or encode(rows[0]) != packet:
                raise ValueError('one canonical command required')
            row = rows[0]
            if (set(row) != {'type', 'request', 'command'} or row['type'] != 'command'
                    or type(row['command']) is not dict):
                raise ValueError('command envelope required')
            kind = row['command'].get('kind')
            if kind not in ('ARM', 'CLEAR', 'DISARM', 'STOP', 'DRIVE'):
                raise ValueError('known command required')
        with self.lock:
            if kind == 'STOP':
                self.controls.clear()
                self.drive = self.status = self.keepalive = None
                self.stop = packet
            elif self.stop is not None:
                raise RuntimeError('STOP transmission pending')
            elif kind == 'ESV':
                self.counts['status_replaced'] += self.status is not None
                self.status = packet
            elif kind == 'BEN_KEEPALIVE':
                self.keepalive = packet
            elif kind == 'DRIVE':
                self.counts['drive_replaced'] += self.drive is not None
                self.drive = packet
            else:
                if len(self.controls) >= 8:
                    raise queue.Full
                self.drive = None
                if kind in ('CLEAR', 'DISARM'):
                    self.status = self.keepalive = None
                self.controls.append(packet)
            self.counts['queued_'+kind] += 1

    def get_nowait(self):
        with self.lock:
            if self.stop is not None:
                result, self.stop = self.stop, None
                return result
            # ARM 직전 상태가 먼저 도착해야 한다. ESV는 한 슬롯이라 DRIVE를 굶기지 않는다.
            if self.status is not None:
                result, self.status = self.status, None
                return result
            if self.controls:
                return self.controls.popleft()
            if self.keepalive is not None:
                result, self.keepalive = self.keepalive, None
                return result
            if self.drive is not None:
                result, self.drive = self.drive, None
                return result
            raise queue.Empty


def serial_factory(**options):
    """기존 공용 ingress의 921600 기본값은 변경하지 않는다."""
    import serial
    options.update(baudrate=BAUDRATE, write_timeout=.02, exclusive=True)
    return serial.Serial(**options)


class UnoIngress(SerialIngress):
    def __init__(self, port, *, factory=None):
        selected = factory or serial_factory
        def at_one_megabaud(**options):
            options.update(baudrate=BAUDRATE, write_timeout=.02)
            return selected(**options)
        super().__init__(port, factory=at_one_megabaud)
        self.commands = UnoMailbox()
