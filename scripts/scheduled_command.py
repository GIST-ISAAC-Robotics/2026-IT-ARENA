"""고정 요청을 시뮬레이션 시계의 정수 ns 격자에 맞춰 발행하기 위한 평가 전용 스케줄러.

기존 평가기의 기본 발행(`now-last>=.019` 폴링)은 바꾸지 않는다. 이 모듈은 선택형(opt-in)이다.
- 예정 tick은 `origin + k*period`로만 계산한다(누적 float 덧셈 없음, drift 없음).
- 한 번의 poll에서는 최대 한 tick만 낸다. 놓친 tick은 진단으로 세고 burst 재생하지 않는다.
- 실제 발행 시각(관측한 시계)을 예정 시각으로 바꾸어 기록하지 않는다. 둘 다 남긴다.
- 시계 역행·큰 점프는 ScheduleError로 중단한다. 같은 시계값 반복은 발행 없이 넘어간다.
"""
from bisect import bisect_right
from dataclasses import dataclass, field

NS_PER_S = 1_000_000_000
# 등록 실행에서 허용하는 값. 순수 클래스는 범위 안의 정수 위상을 모두 받지만 CLI는 이 둘만 허용한다.
REGISTERED_PERIOD_NS = 20_000_000
REGISTERED_PHASES_NS = (0, 10_000_000)
# 이보다 큰 시계 전진은 /clock 단절로 보고 시험을 중단한다(보호층 명령 수명 0.2 s보다 작게 둔다).
DEFAULT_MAX_JUMP_NS = 100_000_000


class ScheduleError(RuntimeError):
    """시계 역행·큰 점프 등 시험을 안전하게 중단해야 하는 조건."""


def seconds_to_ns(value):
    """프로필 시각(등록 프로필은 ms 단위)을 정수 ns로 고정한다. 1 µs 격자에 있지 않으면 거부한다."""
    micro = round(value*1_000_000)
    if abs(micro-value*1_000_000) > 1e-6:
        raise ValueError(f'time {value!r} is not on the 1 us grid')
    return micro*1_000


def align_up(earliest_ns, period_ns, phase_ns):
    """earliest 이상이면서 `t % period == phase`인 가장 이른 정수 시각."""
    _check_grid(period_ns, phase_ns)
    if type(earliest_ns) is not int:
        raise ValueError('earliest time must be integer ns')
    k = -((phase_ns-earliest_ns)//period_ns)  # ceil((earliest-phase)/period)
    return phase_ns+k*period_ns


def _check_grid(period_ns, phase_ns):
    if type(period_ns) is not int or period_ns <= 0:
        raise ValueError('period must be a positive integer ns')
    if type(phase_ns) is not int or not 0 <= phase_ns < period_ns:
        raise ValueError('phase must be an integer ns in [0, period)')


class ProfileSampler:
    """`previous_sample` 프로필을 정수 ns로 표본화한다. 경계 시각에서 float 비교를 피한다."""

    def __init__(self, profile):
        self.times_ns = [seconds_to_ns(row[0]) for row in profile.samples]
        self.values = [tuple(row[1:]) for row in profile.samples]
        self.duration_ns = seconds_to_ns(profile.duration_s)

    def at_ns(self, elapsed_ns):
        if type(elapsed_ns) is not int:
            raise ValueError('elapsed must be integer ns')
        if elapsed_ns < 0 or elapsed_ns >= self.duration_ns:
            return 0., 0.
        return self.values[bisect_right(self.times_ns, elapsed_ns)-1]


@dataclass
class Tick:
    index: int              # origin 기준 tick 번호
    scheduled_ns: int       # 예정 시각
    observed_ns: int        # 발행을 결정한 시계값(실제 발행 시각의 하한)
    lateness_ns: int        # observed - scheduled (>=0)
    missed_before: int      # 이 tick 직전에 건너뛴 tick 수(재생하지 않음)


@dataclass
class GridSchedule:
    """origin 이후 `origin+k*period`마다 최대 한 번 발행 여부를 알려 준다."""
    period_ns: int
    phase_ns: int
    origin_ns: int
    max_jump_ns: int = DEFAULT_MAX_JUMP_NS
    next_index: int = 0
    last_now_ns: int = None
    duplicate_clock_polls: int = 0
    missed_ticks: int = 0
    missed_events: list = field(default_factory=list)
    max_lateness_ns: int = 0
    emitted: int = 0

    def __post_init__(self):
        _check_grid(self.period_ns, self.phase_ns)
        if type(self.origin_ns) is not int or self.origin_ns % self.period_ns != self.phase_ns:
            raise ValueError('origin must lie on the registered grid')
        if type(self.max_jump_ns) is not int or self.max_jump_ns < self.period_ns:
            raise ValueError('max jump must be an integer >= period')

    def scheduled(self, index):
        return self.origin_ns+index*self.period_ns

    def poll(self, now_ns):
        """현재 시계값으로 낼 tick 하나 또는 None. 역행/큰 점프는 ScheduleError."""
        if type(now_ns) is not int:
            raise ScheduleError('clock value must be integer ns')
        if self.last_now_ns is not None:
            if now_ns < self.last_now_ns:
                raise ScheduleError(f'clock regressed {self.last_now_ns} -> {now_ns}')
            if now_ns == self.last_now_ns:
                self.duplicate_clock_polls += 1
            elif now_ns-self.last_now_ns > self.max_jump_ns:
                raise ScheduleError(f'clock jumped {now_ns-self.last_now_ns} ns')
        self.last_now_ns = now_ns
        due = self.scheduled(self.next_index)
        if now_ns < due:
            return None
        # 이미 지난 tick들 중 가장 최근 것 하나만 낸다.
        latest = self.next_index+(now_ns-due)//self.period_ns
        missed = latest-self.next_index
        if missed:
            self.missed_ticks += missed
            self.missed_events.append({'from_index': self.next_index, 'count': missed, 'observed_ns': now_ns})
        scheduled = self.scheduled(latest)
        self.next_index = latest+1
        lateness = now_ns-scheduled
        self.max_lateness_ns = max(self.max_lateness_ns, lateness)
        self.emitted += 1
        return Tick(latest, scheduled, now_ns, lateness, missed)

    def diagnostics(self):
        return {'period_ns': self.period_ns, 'phase_ns': self.phase_ns, 'origin_ns': self.origin_ns,
                'emitted': self.emitted, 'missed_ticks': self.missed_ticks, 'missed_events': self.missed_events[:50],
                'max_lateness_ns': self.max_lateness_ns, 'duplicate_clock_polls': self.duplicate_clock_polls,
                'max_jump_ns': self.max_jump_ns}


def audit_all_ticks(rows, period_ns, late_limit_ns=None):
    """등록 프로필 창 밖(ARM 뒤 대기·프로필 뒤 0 유지·STOP 이후)까지 평가기가 낸 모든 tick 감사.

    유지값 tick이라도 지연·누락은 같은 기준으로 실패다(2026-10-09 r2 후속 계약에서 추가, 기존 창 감사는 그대로).
    """
    late_limit_ns = period_ns//2 if late_limit_ns is None else late_limit_ns
    indices = [r['index'] for r in rows]
    lateness = [r['observed_ns']-r['scheduled_ns'] for r in rows]
    late = [{'index': r['index'], 'scheduled_ns': r['scheduled_ns'], 'lateness_ns': v}
            for r, v in zip(rows, lateness) if v > late_limit_ns]
    missed = [{'index': r['index'], 'scheduled_ns': r['scheduled_ns'], 'missed_before': r.get('missed_before', 0)}
              for r in rows if r.get('missed_before', 0)]
    checks = {
        'all_ticks_contiguous_from_origin': bool(rows) and indices == list(range(len(rows))),
        'all_ticks_not_backdated': bool(rows) and all(v >= 0 for v in lateness),
        'all_ticks_lateness_within_limit': bool(rows) and not late,
        'all_ticks_no_missed': bool(rows) and not missed,
    }
    return {'checks': checks, 'passed': all(checks.values()), 'ticks': len(rows),
            'lateness_ns': {'max': max(lateness, default=None), 'nonzero': sum(v > 0 for v in lateness),
                            'limit': late_limit_ns},
            'late_ticks': late[:50], 'missed': missed[:50],
            'scope': 'origin 이후 평가기가 발행한 모든 격자 tick(프로필 창 밖 유지값 포함)'}


FAULT_KINDS = ('late', 'miss')


@dataclass
class ScheduleFaultInjection:
    """평가 전용 발행 지연 주입. 평가기 자신의 poll을 잠시 보류해 실제로 늦게(또는 한 tick을 건너뛰어) 발행한다.

    제어기·보호층·MCU·플러그인은 건드리지 않는다. 결과 JSON을 사후 변조하지 않으며,
    감사 판정은 실제 발행 기록에서 그대로 계산한다. 기본은 사용하지 않는다.
    - late: 목표 tick을 delay(등록 한도 초과, 한 주기 미만)만큼 늦게 발행 → lateness 실패.
    - miss: 한 주기 이상 두 주기 미만 보류 → 목표 tick 미발행·다음 tick missed_before=1.
    """
    kind: str
    target_ns: int
    delay_ns: int
    withheld_polls: int = 0
    first_withheld_ns: int = None
    released_ns: int = None

    @staticmethod
    def validate(kind, at_ms, delay_ms, sampler, period_ns, late_limit_ns=None):
        """CLI 단계 검사. 목표는 등록 창 안의 유지값 tick이어야 하며 전환 시각을 바꾸지 않는다."""
        late_limit_ns = period_ns//2 if late_limit_ns is None else late_limit_ns
        if kind not in FAULT_KINDS:
            raise ValueError('unknown schedule fault kind')
        if type(at_ms) is not int or type(delay_ms) is not int:
            raise ValueError('fault time and delay must be integer ms')
        rel_ns, delay_ns = at_ms*1_000_000, delay_ms*1_000_000
        if rel_ns % period_ns:
            raise ValueError('fault target must lie on the registered grid')
        spans = 2 if kind == 'late' else 3  # 보류 동안 지나가는 tick까지 유지값이어야 한다.
        if rel_ns < period_ns or rel_ns+spans*period_ns > sampler.duration_ns:
            raise ValueError('fault target must be inside the registered audit window with margins')
        values = {sampler.at_ns(rel_ns+k*period_ns) for k in range(-1, spans)}
        if len(values) != 1:
            raise ValueError('fault target must be a hold-value tick (no profile transition nearby)')
        if kind == 'late' and not late_limit_ns < delay_ns < period_ns:
            raise ValueError('late fault delay must exceed the registered limit and stay below one period')
        if kind == 'miss' and not period_ns <= delay_ns < 2*period_ns:
            raise ValueError('miss fault delay must skip exactly one tick')
        return rel_ns, delay_ns

    def withhold(self, now_ns):
        """True면 이번 poll을 보류한다. 목표 예정 시각부터 delay 동안 한 번만 작동한다."""
        if self.released_ns is not None or now_ns < self.target_ns:
            return False
        if now_ns < self.target_ns+self.delay_ns:
            self.withheld_polls += 1
            if self.first_withheld_ns is None:
                self.first_withheld_ns = now_ns
            return True
        self.released_ns = now_ns
        return False

    def expected_failed_checks(self):
        if self.kind == 'late':
            return {'request_audit': ['lateness_within_limit'], 'request_audit_all_ticks': ['all_ticks_lateness_within_limit']}
        return {'request_audit': ['every_registered_tick_once', 'no_missed_ticks'],
                'request_audit_all_ticks': ['all_ticks_contiguous_from_origin', 'all_ticks_no_missed']}

    def outcome(self, rows, period_ns):
        """실제 발행 기록에서 주입이 의도대로 작동했는지. JSON을 고치지 않고 읽기만 한다."""
        by_scheduled = {r['scheduled_ns']: r for r in rows}
        target, following = by_scheduled.get(self.target_ns), by_scheduled.get(self.target_ns+period_ns)
        if self.kind == 'late':
            applied = bool(target and target['observed_ns']-target['scheduled_ns'] >= self.delay_ns
                           and target.get('missed_before', 0) == 0)
        else:
            applied = bool(target is None and following and following.get('missed_before', 0) == 1)
        return {'kind': self.kind, 'target_scheduled_ns': self.target_ns, 'delay_ns': self.delay_ns,
                'withheld_polls': self.withheld_polls, 'first_withheld_ns': self.first_withheld_ns,
                'released_ns': self.released_ns, 'target_tick': target, 'following_tick': following,
                'applied': applied and self.withheld_polls > 0,
                'expected_failed_checks': self.expected_failed_checks(),
                'scope': '평가기 발행 poll 보류만. 제어기·보호층·MCU·플러그인 미변경, 사후 JSON 변조 없음'}


def audit_scheduled_requests(rows, sampler, start_ns, end_ns, period_ns, phase_ns, late_limit_ns=None):
    """격자 발행 모드 전용 요청 감사. 기존 `audit_requests`(폴링 모드)의 판정은 바꾸지 않는다.

    값은 예정 시각 기준 프로필 표본과 비교하고, 실제 발행 시각 기준 표본과의 차이는 따로 센다.
    """
    _check_grid(period_ns, phase_ns)
    late_limit_ns = period_ns//2 if late_limit_ns is None else late_limit_ns
    active = [r for r in rows if start_ns <= r['scheduled_ns'] <= end_ns]
    expected = list(range(start_ns, end_ns+1, period_ns))
    scheduled = [r['scheduled_ns'] for r in active]
    lateness = [r['observed_ns']-r['scheduled_ns'] for r in active]
    value_mismatch = [r['scheduled_ns'] for r in active
                      if (r['speed_mps'], r['steering_rad']) != sampler.at_ns(r['scheduled_ns']-start_ns)]
    actual_differs = [r['scheduled_ns'] for r in active
                      if sampler.at_ns(r['observed_ns']-start_ns) != sampler.at_ns(r['scheduled_ns']-start_ns)]
    checks = {
        'start_on_grid': start_ns % period_ns == phase_ns,
        'every_registered_tick_once': scheduled == expected,
        'values_match_scheduled_sample': bool(active) and not value_mismatch,
        'not_backdated': bool(active) and all(v >= 0 for v in lateness),
        'header_is_observed_clock': bool(active) and all(r['header_ns'] == r['observed_ns'] for r in active),
        'lateness_within_limit': bool(active) and max(lateness) <= late_limit_ns,
        'no_missed_ticks': bool(active) and all(r.get('missed_before', 0) == 0 for r in active),
    }
    return {'checks': checks, 'passed': all(checks.values()), 'samples': len(active), 'expected_samples': len(expected),
            'lateness_ns': {'max': max(lateness, default=None), 'min': min(lateness, default=None),
                            'nonzero': sum(v > 0 for v in lateness), 'limit': late_limit_ns},
            'value_mismatch_scheduled_ns': value_mismatch[:20],
            'actual_time_sample_differs': {'count': len(actual_differs), 'scheduled_ns': actual_differs[:20],
                                           'meaning': '실제 발행 시각으로 프로필을 표본화했다면 달랐을 tick. 발행값은 예정 시각 기준이다.'}}
