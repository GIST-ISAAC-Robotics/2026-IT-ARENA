"""호스트 단조/ROS 시각의 읽기 구간 기록. 시계 변환·제어 입력과 미연결."""
from dataclasses import dataclass, asdict

from arena_vehicle_interface.clock_bounds import integer


def epoch(value):
    if not isinstance(value,str) or not value.strip() or len(value)>128:
        raise ValueError('explicit bounded clock epoch required')
    return value


def mode(clock_type,active):
    if clock_type not in ('SYSTEM_TIME','ROS_TIME','STEADY_TIME') or type(active) is not bool:
        raise ValueError('explicit clock type and boolean ROS override required')
    if clock_type!='ROS_TIME' and active:
        raise ValueError('only ROS_TIME can have active ROS override')
    return clock_type,active


@dataclass(frozen=True, slots=True)
class ClockPair:
    sequence: int
    host_epoch: str
    ros_epoch: str
    host_before_ns: int
    ros_ns: int
    host_after_ns: int
    clock_type_before: str
    ros_active_before: bool
    clock_type_after: str
    ros_active_after: bool

    def __post_init__(self):
        for name in ('sequence','host_before_ns','ros_ns','host_after_ns'):
            integer(getattr(self,name),name)
        epoch(self.host_epoch)
        epoch(self.ros_epoch)
        mode(self.clock_type_before,self.ros_active_before)
        mode(self.clock_type_after,self.ros_active_after)
        if self.host_after_ns<self.host_before_ns:
            raise ValueError('monotonic bracket reversed')

    @property
    def before_mode(self):return self.clock_type_before,self.ros_active_before

    @property
    def after_mode(self):return self.clock_type_after,self.ros_active_after


def capture_pair(sequence,host_epoch,ros_epoch,*,monotonic_ns,ros_now_ns,read_mode):
    """ROS 시각 읽기 전후를 monotonic으로 감싼다. 파일 기록은 이 구간 밖에서 한다."""
    before_mode=read_mode()
    before=monotonic_ns()
    observed=ros_now_ns()
    after=monotonic_ns()
    after_mode=read_mode()
    return ClockPair(sequence,host_epoch,ros_epoch,before,observed,after,
                     *before_mode,*after_mode)


def audit_pairs(pairs,*,declared_host_epoch,declared_ros_epoch,completed,expected_count):
    """관측된 역행/정지/모드 변화를 구별하고 일정 offset 가설만 대조한다.

    각 읽기의 offset 구간은 [ros-host_after, ros-host_before]다. 교집합이
    비어도 원인을 NTP/점프/드리프트 중 하나로 단정하지 않는다. 교집합이
    있어도 읽기 사이에 시계 변화가 없었다거나 변환을 적용해도 된다는 뜻은 아니다.
    """
    epoch(declared_host_epoch)
    epoch(declared_ros_epoch)
    if type(completed) is not bool:raise ValueError('explicit completion boolean required')
    integer(expected_count,'expected_count',10000)
    rows=list(pairs)
    if not 2<=len(rows)<=10000 or not all(isinstance(r,ClockPair) for r in rows):
        raise ValueError('two through 10000 typed clock pairs required')
    errors=[]
    if not completed:errors.append('capture_incomplete')
    if len(rows)!=expected_count:errors.append('sample_count_mismatch')
    sequence_complete=all(r.sequence==i for i,r in enumerate(rows))
    if not sequence_complete:errors.append('sequence_missing_duplicate_or_reordered')
    if any((r.host_epoch,r.ros_epoch)!=(declared_host_epoch,declared_ros_epoch) for r in rows):
        errors.append('clock_epoch_changed')
    mode_changes=[]
    regressions=[]
    pauses=[]
    intervals=[]
    for i,r in enumerate(rows):
        if r.before_mode!=r.after_mode:mode_changes.append(i)
        if i:
            old=rows[i-1]
            if r.host_before_ns<old.host_after_ns:errors.append('host_bracket_order_violation')
            if r.before_mode!=old.after_mode:mode_changes.append(i)
            dr=r.ros_ns-old.ros_ns
            if dr<0:regressions.append(i)
            if dr==0 and r.host_before_ns>old.host_after_ns:pauses.append(i)
            dh_lo=r.host_before_ns-old.host_after_ns
            dh_hi=r.host_after_ns-old.host_before_ns
            intervals.append(dict(sequence=r.sequence,ros_delta_ns=dr,
                host_elapsed_lower_ns=dh_lo,host_elapsed_upper_ns=dh_hi,
                elapsed_difference_lower_ns=dr-dh_hi,elapsed_difference_upper_ns=dr-dh_lo))
    lower=max(r.ros_ns-r.host_after_ns for r in rows)
    upper=min(r.ros_ns-r.host_before_ns for r in rows)
    # 시뮬레이션 ROS_TIME은 정지/배속/점프가 허용되므로 실물 offset 후보로 쓰지 않는다.
    physical_modes=all(r.before_mode==r.after_mode and
        r.before_mode in (('SYSTEM_TIME',False),('ROS_TIME',False)) for r in rows)
    stable_mode=not mode_changes
    valid=not errors
    compatible=valid and physical_modes and stable_mode and lower<=upper
    return dict(record_integrity_valid=valid,errors=sorted(set(errors)),samples=len(rows),
        capture_completed=completed,recorded_sequence_complete=sequence_complete,
        host_epoch=declared_host_epoch,ros_epoch=declared_ros_epoch,
        maximum_bracket_width_ns=max(r.host_after_ns-r.host_before_ns for r in rows),
        host_span_ns=rows[-1].host_after_ns-rows[0].host_before_ns,
        sampled_mode_change_sequences=sorted(set(mode_changes)),
        sampled_ros_regression_sequences=regressions,sampled_ros_pause_sequences=pauses,
        sampled_zero_ros_sequences=[r.sequence for r in rows if r.ros_ns==0],
        constant_offset_compatible=compatible,
        diagnostic_offset_intersection_ns=[lower,upper] if compatible else None,
        elapsed_intervals=intervals,
        physical_time_source_observed=physical_modes and stable_mode,
        between_samples_continuity_verified=False,ros_stamp_verified=False,
        clock_mapping_applied=False,ready_to_drive=False)


def pair_dict(pair):return asdict(pair)
