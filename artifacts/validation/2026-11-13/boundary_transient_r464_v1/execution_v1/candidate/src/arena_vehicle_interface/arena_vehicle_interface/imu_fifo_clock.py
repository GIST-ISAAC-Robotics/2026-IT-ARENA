"""FIFO 시각의 선택형 온라인 추정. 실물 운영 시각 검증으로 승격하지 않는다.

센서 tick→MCU는 이미 수신한 레지스터 읽기 구간만 사용한다. MCU→호스트는
기존 실제 읽기/수신 offset 추정이다. 후자의 절대 오차는 아래 MCU 구간에
포함되지 않는다. 가속도/각속도 값, 실제 read/receipt 시각은 수정하지 않는다.
"""
from collections import Counter, deque
from dataclasses import dataclass

from .imu_fifo_time import FifoTimeSlots, fit_sensor_to_mcu
from .imu_read_clock import ImuReadClock


@dataclass(frozen=True)
class FifoGyroStamp:
    sample: dict
    stamp_ns: int
    generation: int
    receipt_age_ns: int
    sensor_tick: int
    mcu_lower_ns: int
    mcu_upper_ns: int
    register_anchor_sequence: int
    read_based_stamp_ns: int


class ImuFifoClock:
    time_basis = 'causal_fifo_tick_to_mcu_and_minimum_receipt_offset_estimate'
    time_contract_verified = False
    sensor_acquisition_timestamp = False  # ROS 절대 취득시각 검증은 아직 없다.

    def __init__(self, *, warmup_ns=250_000_000):
        self.slots = FifoTimeSlots()
        self.read_clock = ImuReadClock(warmup_ns=warmup_ns)
        self.anchors = deque(maxlen=8)
        self.fit = None
        self.generation = 0
        self.counts = Counter()
        self.last_stamp_ns = None
        self.last_read_generation = self.read_clock.generation

    @property
    def boot(self):
        return self.slots.boot

    def _clear_fit(self, reason):
        self.fit = None
        self.anchors.clear()
        self.generation += 1
        self.last_stamp_ns = None
        self.counts['reset_' + reason] += 1

    def _sensor_interval(self, row):
        before_generation = self.slots.generation
        before_sequence, before_boot = self.slots.sequence, self.slots.boot
        paired = self.slots.feed(row)
        if self.slots.generation != before_generation:
            self._clear_fit('sensor_epoch')
        # 거부한 과거 report로 현재 fit을 바꾸지 않는다.
        if row['boot'] != self.slots.boot or (
                before_boot == row['boot'] and before_sequence is not None
                and (row['sequence'] - before_sequence) % 2**32 == 0):
            return None
        if before_boot == row['boot'] and before_sequence is not None:
            if (row['sequence'] - before_sequence) % 2**32 >= 2**31:
                return None
        if row.get('spi_errors', 0) or row.get('tx_errors', 0):
            return None
        if row['type'] == 2 and row.get('active') is True:
            if self.anchors and row['sensor_timestamp_raw'] <= self.anchors[-1]['sensor_timestamp_raw']:
                self._clear_fit('register_counter_restart')
            self.anchors.append(dict(row))
            if len(self.anchors) >= 2:
                try:
                    self.fit = fit_sensor_to_mcu(self.anchors, boot=row['boot'])
                    self.counts['fit_updated'] += 1
                except ValueError:
                    # 현재 report가 기존 시계와 양립하지 않으면 이전 fit을 쓰지 않는다.
                    self._clear_fit('inconsistent_register_clock')
                    self.anchors.append(dict(row))
            return None
        if paired is None or row.get('sensor_tag') != 1:
            return None
        if self.fit is None:
            self.counts['fit_warmup'] += 1
            return None
        try:
            interval = self.fit.map_interval(paired['sensor_tick_raw'] * 25, boot=str(row['boot']))
        except ValueError:
            self.counts['mapping_outside_window'] += 1
            return None
        if interval.host_lower_ns > row['read_end_us'] * 1000:
            self.counts['capture_after_read'] += 1
            return None
        return (paired, interval, self.anchors[-1]['sequence'])

    def convert_batch(self, frames, *, host_monotonic_ns, ros_receipt_ns):
        frames = tuple(frames)
        pairs = {}
        gyros = []
        for row in frames:
            mapped = self._sensor_interval(row)
            if mapped is not None:
                pairs[(row['boot'], row['sequence'])] = mapped
            if 'gyro_rad_s' in row:
                gyros.append(row)
        # 실제 read_end를 사용한다. 추정 취득시각으로 읽기시각을 덮어쓰지 않는다.
        read_stamps = self.read_clock.convert_batch(gyros,
            host_monotonic_ns=host_monotonic_ns, ros_receipt_ns=ros_receipt_ns)
        if self.read_clock.generation != self.last_read_generation:
            self.last_stamp_ns = None
            self.generation += 1
            self.last_read_generation = self.read_clock.generation
        result = []
        for base in read_stamps:
            row = base.sample
            mapped = pairs.get((row['boot'], row['sequence']))
            if mapped is None:
                continue
            paired, interval, anchor_sequence = mapped
            # 같은 수신 묶음의 뒤쪽에서 sensor epoch가 바뀌면 이전 세대는 내보내지 않는다.
            if paired['generation'] != self.slots.generation:
                self.counts['superseded_sensor_epoch'] += 1
                continue
            midpoint = (interval.host_lower_ns + interval.host_upper_ns) // 2
            stamp = base.stamp_ns + midpoint - row['read_end_us'] * 1000
            if stamp < 0 or stamp > ros_receipt_ns:
                self.counts['invalid_estimate'] += 1
                continue
            if self.last_stamp_ns is not None and stamp <= self.last_stamp_ns:
                self.counts['nonmonotonic_estimate'] += 1
                continue
            result.append(FifoGyroStamp(row, stamp, self.generation,
                ros_receipt_ns - stamp, paired['sensor_tick'], interval.host_lower_ns,
                interval.host_upper_ns, anchor_sequence, base.stamp_ns))
            self.last_stamp_ns = stamp
            self.counts['published'] += 1
        return result
