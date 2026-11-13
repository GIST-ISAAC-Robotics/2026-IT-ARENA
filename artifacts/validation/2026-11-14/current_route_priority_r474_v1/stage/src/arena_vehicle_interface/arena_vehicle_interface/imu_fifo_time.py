"""현재 비압축/동일 BDR/DEC_1 기록의 FIFO tick 대응. ROS 시각/교정 채택 없음.

ST AN5272 Rev5 §9.2.9, §9.11의 기본 timestamp 접근을 사용한다.
timestamp 슬롯과 tag counter를 대조하고 누락/overrun 뒤 새 timestamp를 기다린다.
"""
from collections import Counter
import copy


class FifoTimeSlots:
    def __init__(self):
        self.boot = None
        self.retired_boots = set()
        self.sequence = None
        self.generation = 0
        self.anchor = None
        self.last_tick = None
        self.unwrapped_tick = None
        self.seen = set()
        self.counts = Counter()
        self.overrun_active = False

    def reset(self, reason):
        self.generation += 1
        self.anchor = self.last_tick = self.unwrapped_tick = None
        self.seen.clear()
        self.counts['reset_' + reason] += 1

    def feed(self, row):
        boot, sequence = row['boot'], row['sequence']
        if type(boot) is not int or not 0 <= boot < 2**32 or type(sequence) is not int or not 0 <= sequence < 2**32:
            raise ValueError('bounded boot/sequence required')
        if boot in self.retired_boots:
            self.counts['old_boot'] += 1
            return None
        if self.boot != boot:
            if self.boot is not None:self.retired_boots.add(self.boot)
            self.boot, self.sequence = boot, None
            self.reset('boot')
        if self.sequence is not None:
            jump=(sequence-self.sequence) % 2**32
            if jump == 0 or jump >= 2**31:
                self.counts['old_sequence'] += 1
                return None
            if jump != 1:self.reset('sequence_gap')
        self.sequence = sequence
        if row.get('spi_errors', 0) or row.get('tx_errors', 0):
            self.reset('reported_io_error')
            return None
        if row['type'] == 2 and row.get('active') is False:
            self.reset('sensor_inactive')
            return None
        if row['type'] != 1:
            return None
        overrun = bool(row['fifo_status'] & 0x40)
        if overrun and not self.overrun_active:
            self.reset('fifo_overrun')
        self.overrun_active = overrun
        raw = row['tag_raw']
        if type(raw) is not int or not 0 <= raw <= 255 or raw.bit_count() % 2:
            self.reset('tag_parity')
            return None
        sensor, counter = raw >> 3, (raw >> 1) & 3
        if sensor == 4:
            tick = row['sensor_timestamp_raw']
            if type(tick) is not int or not 0 <= tick < 2**32:
                raise ValueError('bounded sensor tick required')
            if self.last_tick is None:
                expanded = tick
            else:
                delta = (tick - self.last_tick) % 2**32
                # 현재208Hz 설정의 한 슬롯은 약192tick. 1초 이상 공백은 별도 세대.
                if not 0 < delta < 40_000:
                    self.reset('sensor_tick_restart_or_gap')
                    expanded = tick
                else:
                    expanded = self.unwrapped_tick + delta
            self.last_tick, self.unwrapped_tick = tick, expanded
            self.anchor = dict(tick=expanded, raw=tick, counter=counter,
                               sequence=sequence, read_begin_us=row['read_begin_us'])
            self.seen.clear()
            self.counts['timestamp'] += 1
            return None
        if sensor not in (1, 2):
            self.reset('unsupported_fifo_format')
            return None
        if self.anchor is None or counter != self.anchor['counter'] or sensor in self.seen:
            self.anchor = None
            self.counts['unpaired'] += 1
            return None
        self.seen.add(sensor)
        self.counts['gyro' if sensor == 1 else 'accel'] += 1
        return dict(sample=copy.deepcopy(row), sensor_tick=self.anchor['tick'],
                    sensor_tick_raw=self.anchor['raw'], timestamp_sequence=self.anchor['sequence'],
                    fifo_slot=counter, generation=self.generation, boot=boot,
                    time_basis='fifo_slot_sensor_tick', tick_seconds_applied=False,
                    ros_stamp_verified=False, ready_to_drive=False)


def fit_sensor_to_mcu(anchors, *, boot):
    """TIMESTAMP 레지스터 읽기의 MCU 구간으로 센서 시계를 조건부 affine 대응.

    값이 읽힌 순간은 MCU read_begin/end 사이에 있어야 한다. raw*25us는
    좌표 단위일 뿐 실제 tick=25us라는 단정이 아니며 ±10% 배율을 fit한다.
    최소 편도 지연이나 FIFO read 시각을 취득시각으로 가정하지 않는다.
    """
    from arena_vehicle_interface.clock_bounds import ClockAssumptions, ClockExchange, fit_clock
    rows = tuple(anchors)
    if len(rows) < 2:
        raise ValueError('two or more timestamp register brackets required')
    exchanges = []
    for i, row in enumerate(rows):
        if row['boot'] != boot or row['type'] != 2 or row.get('active') is not True:
            raise ValueError('same active sensor epoch required')
        tick = row['sensor_timestamp_raw']
        if i and tick <= rows[i-1]['sensor_timestamp_raw']:
            raise ValueError('register counter reset/wrap needs a new fit epoch')
        nominal_us = tick * 25
        exchanges.append(ClockExchange(str(boot), i, row['read_begin_us']*1000,
            nominal_us, nominal_us, row['read_end_us']*1000))
    # 진단 모델의 명시 가정. 절대 오차 보증/온도 전 범위 검증으로 승격하지 않는다.
    assumptions = ClockAssumptions(max_rate_error_ppm=100_000,
        host_timestamp_error_ns=1_000, mcu_timestamp_error_us=25,
        max_window_us=60_000_000, max_extrapolation_us=1_500_000)
    return fit_clock(exchanges, assumptions)
