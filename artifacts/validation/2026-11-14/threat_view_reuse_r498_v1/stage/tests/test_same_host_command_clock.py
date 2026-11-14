"""합성 same-host 시계의 원 source 나이·범위·fault 검사. 실제 시계/ROS/장치 없음."""
from dataclasses import replace
from fractions import Fraction
from pathlib import Path
import sys

import pytest

from arena_vehicle_interface.clock_pair_audit import ClockPair
from arena_vehicle_interface.clock_freshness import HostTimeWindow
from arena_vehicle_interface.host_clock_pair import sample_clocks
from same_host_command_clock import SameHostClockBounds,SameHostCommandClock,CommandClockError

BASE=1_000_000_000
OFFSET=1_800_000_000_000_000_000


def bounds(**changes):
    values=dict(source_ttl_ns=100_000_000,max_lookback_ns=80_000_000,max_sample_gap_ns=20_000_000,
        max_read_width_ns=1_000_000,ros_timestamp_error_ns=0,host_timestamp_error_ns=0,
        max_rate_error_ppm=0,assume_no_unsampled_clock_step=True,policy_note='synthetic explicit assumptions; no hardware proof')
    return SameHostClockBounds(**(values|changes))


def pair(index,host,*,width=0,offset=OFFSET,rate=Fraction(1),ros_error=0,**changes):
    observed=offset+int((host+width//2-BASE)*rate)
    return replace(ClockPair(index,'host-A','ros-A',host,observed+ros_error,host+width,
        'ROS_TIME',False,'ROS_TIME',False),**changes)


def clock(policy=None,*,offset=OFFSET,width=0,rate=Fraction(1)):
    value=SameHostCommandClock(host_epoch='host-A',ros_epoch='ros-A',bounds=policy or bounds())
    value.observe(pair(0,BASE,offset=offset,width=width,rate=rate))
    value.observe(pair(1,BASE+10_000_000,offset=offset,width=width,rate=rate))
    return value


def mapped(value,source=OFFSET+5_000_000,now=BASE+12_000_000,**changes):
    return value.map_source(source,source_ros_epoch=changes.get('source_ros_epoch','ros-A'),
        now=changes.get('window',HostTimeWindow('host-A',now,now)))


def reject(reason,call,*,clock_fault=None,transient=None):
    with pytest.raises(CommandClockError) as error:call()
    assert error.value.reason==reason
    if clock_fault is not None:assert error.value.clock_fault is clock_fault
    if transient is not None:assert error.value.transient is transient


def test_original_stamp_age_is_not_rewritten_as_receipt_or_processing_time():
    value=clock();source=OFFSET+5_000_000
    first=mapped(value,source,BASE+12_000_000)
    delayed=mapped(value,source,BASE+19_000_000)
    assert first.source==delayed.source==HostTimeWindow('host-A',BASE+5_000_000,BASE+5_000_000)
    assert first.source_ros_ns==delayed.source_ros_ns==source
    assert first.age_lower_ns==first.age_upper_ns==7_000_000
    assert delayed.age_lower_ns==delayed.age_upper_ns==14_000_000
    assert not first.source_retimed and not first.hardware_assumptions_verified
    assert not first.unsampled_clock_steps_excluded_by_observation


@pytest.mark.parametrize('offset',(1,2**60+17,2**62+19))
def test_arbitrary_large_ros_epoch_offset_does_not_lose_integer_nanoseconds(offset):
    value=clock(offset=offset)
    result=mapped(value,offset+5_000_001)
    assert result.source.lower_ns==result.source.upper_ns==BASE+5_000_001
    assert result.age_upper_ns==6_999_999


@pytest.mark.parametrize('ppm',(-1000,0,1000))
def test_declared_rate_endpoints_contain_exact_synthetic_source(ppm):
    rate=Fraction(1_000_000+ppm,1_000_000)
    value=clock(bounds(max_rate_error_ppm=1000),rate=rate,width=2000)
    actual=BASE+5_000_000
    result=mapped(value,OFFSET+int((actual-BASE)*rate),now=BASE+12_000_000)
    assert result.source.lower_ns<=actual<=result.source.upper_ns
    assert result.age_lower_ns<=7_000_000<=result.age_upper_ns


def test_floor_and_ceil_expand_fractional_nanosecond_bounds():
    value=clock(bounds(max_rate_error_ppm=1))
    result=mapped(value,OFFSET+9_999_999,now=BASE+10_000_001)
    # delta ROS=1ns; 1/(1±1ppm) 경계를 [0,2]ns로 바깥쪽 반올림한다.
    assert result.source==HostTimeWindow('host-A',BASE+10_000_000-2,BASE+10_000_000)
    assert result.age_lower_ns==1 and result.age_upper_ns==3


def test_read_bracket_and_both_timestamp_errors_are_preserved():
    policy=bounds(ros_timestamp_error_ns=7,host_timestamp_error_ns=3)
    value=clock(policy,width=100)
    result=mapped(value,OFFSET+5_000_000,now=BASE+12_000_000)
    assert result.source.lower_ns==BASE+4_999_933
    assert result.source.upper_ns==BASE+5_000_067
    assert result.age_lower_ns==6_999_933 and result.age_upper_ns==7_000_067


def test_capture_uses_existing_pair_order_and_keeps_every_bracket():
    value=SameHostCommandClock(host_epoch='host-A',ros_epoch='ros-A',bounds=bounds())
    steps=[];times=iter([BASE,BASE+100])
    def mono():steps.append('host');return next(times)
    def ros():steps.append('ros');return OFFSET+50
    def mode():steps.append('mode');return 'ROS_TIME',False
    observed=value.capture(monotonic_ns=mono,ros_now_ns=ros,read_mode=mode)
    assert steps==['mode','host','ros','host','mode'] and value.sample_count==1
    # host_clock_pair의 단일읽기 원시 bracket과도 같은 의미. midpoint를 변환에 쓰지 않는다.
    times=iter([BASE,BASE+100])
    _,_,diagnostic=sample_clocks(lambda:next(times),lambda:OFFSET+50,attempts=1)
    assert diagnostic['samples'][0]['before_monotonic_ns']==observed.host_before_ns
    assert diagnostic['samples'][0]['after_monotonic_ns']==observed.host_after_ns


def test_first_source_observation_is_inclusive_without_error():
    result=mapped(clock(),OFFSET)
    assert result.source==HostTimeWindow('host-A',BASE,BASE)


@pytest.mark.parametrize('source,reason',[(0,'zero_source_timestamp'),(-1,'invalid_source_timestamp'),
    (True,'invalid_source_timestamp'),(1.5,'invalid_source_timestamp'),(2**63,'invalid_source_timestamp'),
    (OFFSET-1,'source_before_first_observation_or_uncertain'),
    (OFFSET+10_000_001,'source_after_latest_pair_or_uncertain')])
def test_invalid_old_or_future_sources_rejected_without_retiming(source,reason):
    value=clock();reject(reason,lambda:mapped(value,source),clock_fault=False)
    assert value.fault is None


def test_wrong_source_epoch_or_uncertain_order_rejected():
    reject('source_ros_epoch_mismatch',lambda:mapped(clock(),source_ros_epoch='old-ros'),clock_fault=False)
    value=clock(bounds(ros_timestamp_error_ns=5))
    reject('source_before_first_observation_or_uncertain',lambda:mapped(value,OFFSET+9))
    reject('source_after_latest_pair_or_uncertain',lambda:mapped(value,OFFSET+10_000_000-9))


def test_single_pair_cannot_establish_continuous_sample_segment():
    value=SameHostCommandClock(host_epoch='host-A',ros_epoch='ros-A',bounds=bounds())
    value.observe(pair(0,BASE))
    reject('two_clock_pairs_required',lambda:mapped(value))


def test_lookback_and_processing_age_are_separate_explicit_limits():
    value=clock(bounds(max_lookback_ns=4_999_999))
    reject('source_lookback_exceeded',lambda:mapped(value))
    value=clock(bounds(source_ttl_ns=7_000_000))
    reject('source_ttl_exceeded_or_uncertain',lambda:mapped(value))
    assert mapped(clock(bounds(source_ttl_ns=7_000_001))).age_upper_ns==7_000_000


def test_uncertainty_is_not_subtracted_from_ttl():
    value=clock(bounds(source_ttl_ns=7_000_050),width=100)
    reject('source_ttl_exceeded_or_uncertain',lambda:mapped(value))


@pytest.mark.parametrize('changes,reason',[
    (dict(sequence=3),'clock_pair_sequence_changed'),
    (dict(host_epoch='host-B'),'clock_epoch_changed'),
    (dict(ros_epoch='ros-B'),'clock_epoch_changed'),
    (dict(host_before_ns=BASE+9_999_999,host_after_ns=BASE+10_000_000),'host_clock_regressed_or_brackets_overlap'),
    (dict(ros_ns=OFFSET+9_000_000),'ros_clock_regressed_or_paused'),
    (dict(ros_ns=OFFSET+10_000_000),'ros_clock_regressed_or_paused'),
    (dict(ros_ns=OFFSET+15_000_001),'clock_step_or_rate_bound_exceeded'),
    (dict(ros_ns=OFFSET+14_999_999),'clock_step_or_rate_bound_exceeded'),
    (dict(clock_type_before='SYSTEM_TIME',clock_type_after='SYSTEM_TIME'),'clock_mode_changed_between_reads'),
    (dict(ros_active_after=True),'clock_mode_changed_within_read'),
    (dict(ros_active_before=True,ros_active_after=True),'physical_system_ros_time_required'),
    (dict(clock_type_before='STEADY_TIME',clock_type_after='STEADY_TIME'),'physical_system_ros_time_required'),
    (dict(ros_ns=0),'zero_ros_clock'),
])
def test_bad_continuation_latches_and_cannot_be_cleared_by_new_pair(changes,reason):
    value=clock();row=pair(2,BASE+15_000_000,**changes)
    reject(reason,lambda:value.observe(row),clock_fault=True)
    assert value.fault==reason
    reject(reason,lambda:value.observe(pair(2,BASE+15_000_000)),clock_fault=True)
    reject(reason,lambda:mapped(value),clock_fault=True)


def test_long_sampling_gap_and_old_latest_pair_are_distinct_faults():
    value=clock()
    reject('clock_sample_gap_exceeded',lambda:value.observe(pair(2,BASE+30_000_001)),clock_fault=False,transient=True)
    assert value.fault is None and not value.ready and value.sample_count==1 and value.reanchors==1
    value=clock()
    reject('latest_clock_pair_too_old',lambda:mapped(value,now=BASE+30_000_001),clock_fault=False,transient=True)
    assert value.fault is None and not value.ready and value.sample_count==0 and value.reanchors==1


def test_wide_read_reanchors_with_two_narrow_pairs_and_new_source_only():
    value=clock()
    reject('clock_read_too_wide',lambda:value.observe(pair(2,BASE+15_000_000,width=2_000_000)),clock_fault=False,transient=True)
    assert value.observed_count==3 and value.sample_count==0 and value.reanchors==1 and value.fault is None
    assert value.observe(pair(3,BASE+18_000_000)) is False
    reject('two_clock_pairs_required',lambda:mapped(value,OFFSET+18_000_000,BASE+18_000_000),transient=True)
    assert value.observe(pair(4,BASE+20_000_000)) is True
    reject('source_before_first_observation_or_uncertain',lambda:mapped(value,OFFSET+5_000_000,BASE+20_000_000))
    reject('source_before_reanchor_ready_or_uncertain',lambda:mapped(value,OFFSET+19_000_000,BASE+20_000_000))
    value.observe(pair(5,BASE+22_000_000))
    result=mapped(value,OFFSET+21_000_000,BASE+23_000_000)
    assert result.source.lower_ns==result.source.upper_ns==BASE+21_000_000
    assert result.age_upper_ns==2_000_000 and result.source_floor_ros_ns==OFFSET+20_000_000
    assert value.ready and value.transient_reason is None and value.reanchors==1


def test_stale_latest_recovers_without_restart_or_reusing_pre_warmup_source():
    value=clock()
    reject('latest_clock_pair_too_old',lambda:mapped(value,now=BASE+30_000_001),transient=True)
    reject('clock_sample_gap_exceeded',lambda:value.observe(pair(2,BASE+31_000_000)),transient=True)
    assert value.reanchors==1 and value.sample_count==1
    value.observe(pair(3,BASE+41_000_000))
    reject('source_before_reanchor_ready_or_uncertain',lambda:mapped(value,OFFSET+36_000_000,BASE+41_000_000))
    value.observe(pair(4,BASE+43_000_000))
    assert mapped(value,OFFSET+42_000_000,BASE+44_000_000).age_upper_ns==2_000_000
    assert value.ready and value.fault is None


def test_real_regression_after_transient_still_latches():
    value=clock()
    reject('clock_read_too_wide',lambda:value.observe(pair(2,BASE+15_000_000,width=2_000_000)),transient=True)
    reject('ros_clock_regressed_or_paused',lambda:value.observe(pair(3,BASE+18_000_000,ros_ns=OFFSET+15_000_000)),clock_fault=True)


def test_now_epoch_and_regression_are_faults():
    reject('host_epoch_changed',lambda:mapped(clock(),window=HostTimeWindow('host-B',BASE+12_000_000,BASE+12_000_000)),clock_fault=True)
    reject('now_before_latest_clock_pair',lambda:mapped(clock(),now=BASE+9_999_999),clock_fault=True)


def test_now_order_and_later_sample_include_previous_mapping_now():
    value=clock();mapped(value,now=BASE+14_000_000)
    reject('mapping_now_regressed_or_overlapped',lambda:mapped(value,now=BASE+13_000_000),clock_fault=True)
    value=clock();mapped(value,now=BASE+14_000_000)
    reject('clock_pair_before_last_mapping_now',lambda:value.observe(pair(2,BASE+13_000_000)),clock_fault=True)


def test_same_source_later_anchor_preserves_age_not_zero_age():
    value=clock();original=mapped(value,now=BASE+12_000_000)
    value.observe(pair(2,BASE+15_000_000))
    later=mapped(value,now=BASE+17_000_000)
    assert original.source==later.source
    assert later.age_lower_ns==later.age_upper_ns==12_000_000


def test_capture_failure_latches_instead_of_reusing_last_good_pair():
    value=clock();times=iter([BASE+15_000_000,BASE+14_999_999])
    reject('clock_capture_failed',lambda:value.capture(monotonic_ns=lambda:next(times),
        ros_now_ns=lambda:OFFSET+15_000_000,read_mode=lambda:('ROS_TIME',False)),clock_fault=True)
    assert value.sample_count==2


def test_rate_and_error_compatibility_has_no_extra_epsilon():
    policy=bounds(max_rate_error_ppm=1000,ros_timestamp_error_ns=5)
    value=SameHostCommandClock(host_epoch='host-A',ros_epoch='ros-A',bounds=policy)
    value.observe(pair(0,BASE,ros_error=-5))
    value.observe(pair(1,BASE+10_000_000,rate=Fraction(1001,1000),ros_error=5))
    assert value.sample_count==2
    other=SameHostCommandClock(host_epoch='host-A',ros_epoch='ros-A',bounds=policy)
    other.observe(pair(0,BASE,ros_error=-5))
    reject('clock_step_or_rate_bound_exceeded',lambda:other.observe(pair(1,BASE+10_000_000,rate=Fraction(1001,1000),ros_error=6)))


def test_compatible_drift_does_not_require_constant_offset_fit():
    value=clock(bounds(max_rate_error_ppm=1000),rate=Fraction(1001,1000))
    value.observe(pair(2,BASE+15_000_000,rate=Fraction(1001,1000)))
    result=mapped(value,OFFSET+5_005_000,now=BASE+17_000_000)
    assert result.source.lower_ns<=BASE+5_000_000<=result.source.upper_ns


def test_unsampled_jump_and_return_is_not_claimed_detectable():
    # 같은 두 표본 사이 jump후복귀는 데이터에 없다. 통과는 연속성 전제 아래에서만이다.
    result=mapped(clock())
    assert result.unsampled_clock_steps_excluded_by_observation is False
    assert result.hardware_assumptions_verified is False


@pytest.mark.parametrize('change',[dict(source_ttl_ns=0),dict(max_lookback_ns=0),dict(max_sample_gap_ns=0),
    dict(max_read_width_ns=0),dict(ros_timestamp_error_ns=-1),dict(host_timestamp_error_ns=True),
    dict(max_rate_error_ppm=1_000_000),dict(max_rate_error_ppm=1.5),dict(assume_no_unsampled_clock_step=False),
    dict(assume_no_unsampled_clock_step=1),dict(policy_note='')])
def test_bounds_are_explicit_and_not_automatically_extended(change):
    with pytest.raises(ValueError):bounds(**change)
