"""미연결 후보: 압축 없는 DEC_1 FIFO의 슬롯과 원시 tick만 연관한다.

호스트/ROS 시각·영점·자세를 생성하지 않는다. 사용자 장착 정보나 실제
취득시각 검증 없이 이 결과를 제어기 입력으로 승격하지 않는다.
"""
import struct
from collections import Counter
from dataclasses import dataclass


@dataclass(frozen=True)
class FifoSample:
    epoch: int
    sequence: int
    kind: str
    raw_xyz: tuple[int, int, int]
    sensor_tick: int
    slot: int
    read_begin_us: int
    read_end_us: int
    host_monotonic_ns: int
    time_basis: str = 'sensor_ticks_unmapped'


class FifoAssociator:
    """한FIFO슬롯마다timestamp가 있고 압축이 꺼진 진단설정 전용."""
    def __init__(self):
        self.stats = Counter()
        self.boot = self.sequence = None
        self.epoch = -1
        self.restart_pending = False
        self.clear_clock()

    def clear_slot(self):
        self.slot = None
        self.seen = set()

    def clear_clock(self):
        self.previous_raw = None
        self.ticks = 0
        self.clear_slot()

    def invalidate(self, reason, *, clock=False):
        self.stats[reason] += 1
        self.clear_clock() if clock else self.clear_slot()

    def feed(self, row):
        boot, seq = row['boot'], row['sequence']
        if self.boot != boot:
            self.epoch += 1
            self.boot, self.sequence = boot, None
            self.restart_pending = False
            self.clear_clock()
            self.stats['epochs'] += 1
        if self.sequence is not None and (seq - self.sequence) % 2**32 != 1:
            self.invalidate('wire_gap')
        self.sequence = seq
        if row.get('fifo_status', 0) & 0x40:
            self.invalidate('fifo_overrun')
            return None
        if row.get('spi_errors', 0) or row.get('tx_errors', 0):
            self.invalidate('transport_error')
            return None
        if row['type'] != 1:
            if row['type'] == 2 and not row.get('active', True):
                self.invalidate('stopped', clock=True)
                self.restart_pending = True
            return None
        tag = row['tag_raw']
        if type(tag) is not int or not 0 <= tag < 256 or tag.bit_count() % 2:
            self.invalidate('tag_parity')
            return None
        kind, slot = tag >> 3, (tag >> 1) & 3
        payload = bytes.fromhex(row['payload_hex'])
        if len(payload) != 16 or row['read_end_us'] < row['read_begin_us']:
            self.invalidate('malformed_frame')
            return None
        if kind == 4:
            if self.restart_pending:
                self.epoch += 1
                self.restart_pending = False
                self.clear_clock()
                self.stats['collection_restart'] += 1
            raw = struct.unpack_from('<I', payload)[0]
            if self.previous_raw is None:
                self.ticks = raw
            else:
                delta = (raw - self.previous_raw) % 2**32
                if not 0 < delta < 2**31:
                    self.epoch += 1
                    self.invalidate('sensor_clock_discontinuity', clock=True)
                    return None
                self.ticks += delta
            self.previous_raw = raw
            self.slot, self.seen = slot, set()
            self.stats['timestamp'] += 1
            return None
        if kind not in (1, 2):
            self.invalidate('unsupported_fifo_tag')
            return None
        if self.slot is None or self.slot != slot:
            self.invalidate('timestamp_slot_missing')
            return None
        if kind in self.seen:
            self.invalidate('duplicate_stream_in_slot')
            return None
        self.seen.add(kind)
        self.stats['gyro' if kind == 1 else 'accel'] += 1
        return FifoSample(self.epoch, seq, 'gyro' if kind == 1 else 'accel',
            struct.unpack_from('<hhh', payload), self.ticks, slot,
            row['read_begin_us'], row['read_end_us'], row['host_monotonic_ns'])
