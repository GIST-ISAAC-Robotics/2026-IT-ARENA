"""진단용 MCU 읽기 시계 연결. 센서 취득시각 교정이나 운영 승인이 아니다."""
from dataclasses import dataclass


@dataclass(frozen=True)
class EstimatedGyroStamp:
    sample: dict
    stamp_ns: int
    generation: int
    receipt_age_ns: int


class ImuReadClock:
    """한 수신 묶음의 마지막 읽기를 기준으로 표본 간 간격을 보존한다.

    호스트 수신시각-MCU 읽기시각의 최솟값으로 전송 지연 변동만 줄인다.
    최소 전송 지연, FIFO 이전 취득 지연, 두 시계의 배율은 검증하지 않는다.
    취득시각으로 표시하거나 자율 제어의 timing_verified로 승격하지 않는다.
    """
    time_basis = 'minimum_receipt_offset_mcu_read_estimate'
    time_contract_verified = False

    def __init__(self, warmup_ns=250_000_000, clock_step_ns=5_000_000):
        if (type(warmup_ns) is not int or warmup_ns < 0
                or type(clock_step_ns) is not int or clock_step_ns <= 0):
            raise ValueError('invalid diagnostic clock configuration')
        self.warmup_ns = warmup_ns
        self.clock_step_ns = clock_step_ns
        self.generation = 0
        self.boot = None
        self.retired_boots = set()
        self.last_host_ns = self.last_ros_offset_ns = None
        self.counts = dict(published=0, warmup=0, invalid=0, old_boot=0,
                           nonmonotonic=0, clock_reset=0, boot_reset=0)
        self._clear()

    def _clear(self):
        self.first_read_ns = self.last_read_ns = self.offset_ns = None
        self.last_stamp_ns = None
        self.generation += 1

    def convert_batch(self, samples, *, host_monotonic_ns, ros_receipt_ns):
        if (type(host_monotonic_ns) is not int or host_monotonic_ns < 0
                or type(ros_receipt_ns) is not int or ros_receipt_ns < 0):
            raise ValueError('receipt clocks must be nonnegative integer nanoseconds')
        ros_offset = ros_receipt_ns - host_monotonic_ns
        if self.last_host_ns is not None and (
                host_monotonic_ns < self.last_host_ns
                or abs(ros_offset-self.last_ros_offset_ns) > self.clock_step_ns):
            self._clear()
            self.counts['clock_reset'] += 1
        self.last_host_ns, self.last_ros_offset_ns = host_monotonic_ns, ros_offset
        # 다른 boot가 같은 UART 묶음에 들어와도 앞뒤 이력을 합치지 않는다.
        segments = []
        for row in samples:
            begin, end, boot = (row.get(k) for k in ('read_begin_us', 'read_end_us', 'boot'))
            if (any(type(v) is not int or v < 0 for v in (begin, end, boot))
                    or begin > end or end-begin > 20_000 or not 0 <= boot < 2**32):
                self.counts['invalid'] += 1
                continue
            if not segments or segments[-1][0] != boot:
                segments.append((boot, []))
            segments[-1][1].append(row)
        result = []
        for boot, rows in segments:
            if boot in self.retired_boots:
                self.counts['old_boot'] += len(rows)
                continue
            if boot != self.boot:
                if self.boot is not None:
                    self.retired_boots.add(self.boot)
                self.boot = boot
                self._clear()
                self.counts['boot_reset'] += 1
            valid = []
            for row in rows:
                read_ns = row['read_end_us'] * 1000
                if self.last_read_ns is not None and read_ns <= self.last_read_ns:
                    self.counts['nonmonotonic'] += 1
                    continue
                if self.first_read_ns is None:
                    self.first_read_ns = read_ns
                self.last_read_ns = read_ns
                valid.append((read_ns, row))
            if not valid:
                continue
            # 묶음 전체에서 먼저 offset을 정해야 큐에 모인 gyro들의 간격이 남는다.
            candidate = host_monotonic_ns - valid[-1][0]
            self.offset_ns = candidate if self.offset_ns is None else min(self.offset_ns, candidate)
            for read_ns, row in valid:
                if read_ns-self.first_read_ns < self.warmup_ns:
                    self.counts['warmup'] += 1
                    continue
                stamp_ns = read_ns + self.offset_ns + ros_offset
                if stamp_ns < 0 or (self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns):
                    self.counts['nonmonotonic'] += 1
                    continue
                assert stamp_ns <= ros_receipt_ns
                result.append(EstimatedGyroStamp(row, stamp_ns, self.generation,
                                                ros_receipt_ns-stamp_ns))
                self.last_stamp_ns = stamp_ns
                self.counts['published'] += 1
        return result
