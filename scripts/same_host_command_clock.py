"""같은 호스트의 physical ROS source를 단조 구간으로 옮긴다. ROS/serial I/O 없음.

표본 사이 kernel clock 불연속 없음과 명시 상대 rate/error 범위가 전제다.
관측하지 않은 jump를 완벽히 검출하거나 실물 가정을 검증했다고 주장하지 않는다.
"""
from dataclasses import dataclass

from arena_vehicle_interface.clock_bounds import integer
from arena_vehicle_interface.clock_pair_audit import ClockPair, capture_pair, epoch
from arena_vehicle_interface.clock_freshness import HostTimeWindow

PPM=1_000_000
PHYSICAL_MODES=(('SYSTEM_TIME',False),('ROS_TIME',False))


class CommandClockError(ValueError):
    """clock_fault=True이면 이 객체는 새 세션으로 교체해야 한다. STOP은 상위 책임."""
    def __init__(self,reason,*,clock_fault=False,transient=False):
        super().__init__(reason)
        self.reason=reason
        self.clock_fault=clock_fault
        self.transient=transient


@dataclass(frozen=True,slots=True)
class SameHostClockBounds:
    source_ttl_ns: int
    max_lookback_ns: int
    max_sample_gap_ns: int
    max_read_width_ns: int
    ros_timestamp_error_ns: int
    host_timestamp_error_ns: int
    max_rate_error_ppm: int
    assume_no_unsampled_clock_step: bool
    policy_note: str

    def __post_init__(self):
        for name in ('source_ttl_ns','max_lookback_ns','max_sample_gap_ns','max_read_width_ns',
                     'ros_timestamp_error_ns','host_timestamp_error_ns'):
            integer(getattr(self,name),name)
        integer(self.max_rate_error_ppm,'relative ROS/monotonic rate ppm',PPM-1)
        if any(getattr(self,name)==0 for name in ('source_ttl_ns','max_lookback_ns','max_sample_gap_ns','max_read_width_ns')):
            raise ValueError('explicit positive time limits required')
        if self.assume_no_unsampled_clock_step is not True:
            raise ValueError('explicit no-unsampled-step assumption required')
        if type(self.policy_note) is not str or not self.policy_note.strip() or len(self.policy_note)>1024:
            raise ValueError('explicit bounded assumption note required')


@dataclass(frozen=True,slots=True)
class CommandSourceWindow:
    source_ros_ns: int
    source_ros_epoch: str
    source: HostTimeWindow
    now: HostTimeWindow
    age_lower_ns: int
    age_upper_ns: int
    anchor_sequence: int
    first_sequence: int
    source_floor_ros_ns: int
    hardware_assumptions_verified: bool=False
    unsampled_clock_steps_excluded_by_observation: bool=False
    source_retimed: bool=False


def ceil_div(numerator,denominator):
    return -(-numerator//denominator)


class SameHostCommandClock:
    """단일 소유자의 짧은 pair stream. offset fit/MCU시계/동시 호출/자동 복구 없음."""
    def __init__(self,*,host_epoch,ros_epoch,bounds):
        epoch(host_epoch);epoch(ros_epoch)
        if not isinstance(bounds,SameHostClockBounds):raise ValueError('typed explicit same-host bounds required')
        self.host_epoch,self.ros_epoch,self.bounds=host_epoch,ros_epoch,bounds
        self._first=None
        self._latest=None
        self._fault=None
        self._last_now_upper=None
        self._last_observed=None
        self._observed_count=0
        self._segment_count=0
        self._reanchors=0
        self._recovering=False
        self._source_floor=None
        self._transient_reason='initial_warmup'

    @property
    def fault(self):return self._fault

    @property
    def sample_count(self):return self._segment_count

    @property
    def observed_count(self):return self._observed_count

    @property
    def ready(self):return self._fault is None and self._segment_count>=2

    @property
    def reanchors(self):return self._reanchors

    @property
    def transient_reason(self):return self._transient_reason

    def _healthy(self):
        if self._fault is not None:raise CommandClockError(self._fault,clock_fault=True)

    def _fail(self,reason):
        self._fault=reason
        raise CommandClockError(reason,clock_fault=True)

    def _invalidate(self,reason):
        if self._segment_count or not self._recovering:self._reanchors+=1
        self._first=self._latest=None
        self._segment_count=0
        self._source_floor=None
        self._recovering=True
        self._transient_reason=reason

    def _accept(self,pair):
        if self._first is None:
            self._first=pair
            if not self._recovering:self._source_floor=pair.ros_ns
        self._latest=pair
        self._segment_count+=1
        if self.ready:
            # 읽기지연/공백 전 또는 warmup 도중 명령을 새 anchor에 재포장하지 않는다.
            if self._recovering:self._source_floor=pair.ros_ns
            self._recovering=False
            self._transient_reason=None

    def capture(self,*,monotonic_ns,ros_now_ns,read_mode):
        """각 ROS 읽기 전후를 기존 capture_pair로 감싼다. 좁은 표본만 골라 버리지 않는다."""
        self._healthy()
        try:
            pair=capture_pair(self.observed_count,self.host_epoch,self.ros_epoch,
                monotonic_ns=monotonic_ns,ros_now_ns=ros_now_ns,read_mode=read_mode)
        except Exception as error:
            self._fault='clock_capture_failed'
            raise CommandClockError(self._fault,clock_fault=True) from error
        self.observe(pair)
        return pair

    def observe(self,pair):
        """모든 연속 pair를 검토한다. 실패 후에는 새 정상 pair로 fault를 지우지 않는다."""
        self._healthy()
        if not isinstance(pair,ClockPair):self._fail('typed_clock_pair_required')
        if (pair.host_epoch,pair.ros_epoch)!=(self.host_epoch,self.ros_epoch):self._fail('clock_epoch_changed')
        if pair.sequence!=self.observed_count:self._fail('clock_pair_sequence_changed')
        if pair.before_mode!=pair.after_mode:self._fail('clock_mode_changed_within_read')
        if pair.before_mode not in PHYSICAL_MODES:self._fail('physical_system_ros_time_required')
        if pair.ros_ns==0:self._fail('zero_ros_clock')
        b=self.bounds;he=b.host_timestamp_error_ns;re=b.ros_timestamp_error_ns
        if pair.host_before_ns<he:self._fail('host_error_precedes_epoch')
        if pair.host_after_ns+he>2**63-1:self._fail('host_error_exceeds_clock_range')
        if self._last_now_upper is not None and pair.host_before_ns-he<self._last_now_upper:
            self._fail('clock_pair_before_last_mapping_now')
        old=self._last_observed
        gap_exceeded=False
        if old is not None:
            if pair.before_mode!=old.after_mode:self._fail('clock_mode_changed_between_reads')
            if pair.host_before_ns<old.host_after_ns:self._fail('host_clock_regressed_or_brackets_overlap')
            if pair.ros_ns<=old.ros_ns:self._fail('ros_clock_regressed_or_paused')
            dh_lo=max(0,pair.host_before_ns-old.host_after_ns-2*he)
            dh_hi=pair.host_after_ns-old.host_before_ns+2*he
            gap_exceeded=dh_hi>b.max_sample_gap_ns
            dr=pair.ros_ns-old.ros_ns
            if ((dr+2*re)*PPM<dh_lo*(PPM-b.max_rate_error_ppm)
                    or (dr-2*re)*PPM>dh_hi*(PPM+b.max_rate_error_ppm)):
                self._fail('clock_step_or_rate_bound_exceeded')
        self._last_observed=pair
        self._observed_count+=1
        if pair.host_after_ns-pair.host_before_ns+2*he>b.max_read_width_ns:
            self._invalidate('clock_read_too_wide')
            raise CommandClockError('clock_read_too_wide',transient=True)
        if gap_exceeded:
            self._invalidate('clock_sample_gap_exceeded')
            self._accept(pair)  # 현재 좁은 읽기는 새 segment의 첫 표본으로만 쓴다.
            raise CommandClockError('clock_sample_gap_exceeded',transient=True)
        self._accept(pair)
        return self.ready

    def map_source(self,source_ros_ns,*,source_ros_epoch,now):
        """원 header stamp를 latest bracket에서 뒤로 변환한다. TTL 경계는 strict다."""
        self._healthy()
        if not isinstance(now,HostTimeWindow):raise CommandClockError('typed_now_window_required')
        if now.host_epoch!=self.host_epoch:self._fail('host_epoch_changed')
        if not self.ready:raise CommandClockError('two_clock_pairs_required',transient=True)
        b=self.bounds;last=self._latest;first=self._first
        he=b.host_timestamp_error_ns;re=b.ros_timestamp_error_ns
        if now.lower_ns<last.host_after_ns+he:self._fail('now_before_latest_clock_pair')
        if self._last_now_upper is not None and now.lower_ns<self._last_now_upper:self._fail('mapping_now_regressed_or_overlapped')
        self._last_now_upper=now.upper_ns
        if now.upper_ns-(last.host_before_ns-he)>b.max_sample_gap_ns:
            self._invalidate('latest_clock_pair_too_old')
            raise CommandClockError('latest_clock_pair_too_old',transient=True)
        try:integer(source_ros_ns,'source ROS stamp');epoch(source_ros_epoch)
        except ValueError as error:raise CommandClockError('invalid_source_timestamp') from error
        if source_ros_ns==0:raise CommandClockError('zero_source_timestamp')
        if source_ros_epoch!=self.ros_epoch:raise CommandClockError('source_ros_epoch_mismatch')
        # source와 첫 표본/anchor의 ROS 읽기 오차를 모두 포함해 순서를 확정한다.
        if source_ros_ns-first.ros_ns<2*re:raise CommandClockError('source_before_first_observation_or_uncertain')
        if source_ros_ns-self._source_floor<2*re:raise CommandClockError('source_before_reanchor_ready_or_uncertain')
        delta=last.ros_ns-source_ros_ns
        if delta<2*re:raise CommandClockError('source_after_latest_pair_or_uncertain')
        # dROS/dMONOTONIC ∈ [1-rate,1+rate]. 모든 산술은 정수 ns이며 반올림은 바깥쪽이다.
        elapsed_lower=((delta-2*re)*PPM)//(PPM+b.max_rate_error_ppm)
        elapsed_upper=ceil_div((delta+2*re)*PPM,PPM-b.max_rate_error_ppm)
        if elapsed_upper>b.max_lookback_ns:raise CommandClockError('source_lookback_exceeded')
        lower=last.host_before_ns-he-elapsed_upper
        upper=last.host_after_ns+he-elapsed_lower
        if lower<0:raise CommandClockError('source_mapping_precedes_host_epoch')
        source=HostTimeWindow(self.host_epoch,lower,upper)
        age_lower=now.lower_ns-source.upper_ns
        age_upper=now.upper_ns-source.lower_ns
        if age_lower<0:raise CommandClockError('source_future_or_age_uncertain')
        if age_upper>=b.source_ttl_ns:raise CommandClockError('source_ttl_exceeded_or_uncertain')
        return CommandSourceWindow(source_ros_ns,self.ros_epoch,source,now,age_lower,age_upper,last.sequence,first.sequence,self._source_floor)
