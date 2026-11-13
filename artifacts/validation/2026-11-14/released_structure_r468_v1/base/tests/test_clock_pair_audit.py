from dataclasses import replace, asdict
import json

import pytest

from arena_vehicle_interface.clock_pair_audit import ClockPair, capture_pair, audit_pairs
from audit_clock_pairs import audit_bytes, SCHEMA


def pair(i,*,delta=0,active=False):
    h=1_000_000_000+i*20_000_000
    return ClockPair(i,'host-A','ros-A',h,1_800_000_000_000_000_000+h+500+delta,h+1000,
                     'ROS_TIME',active,'ROS_TIME',active)


def audit(rows=None,**kwargs):
    rows=[pair(i) for i in range(3)] if rows is None else rows
    return audit_pairs(rows,**dict(dict(declared_host_epoch='host-A',declared_ros_epoch='ros-A',
                     completed=True,expected_count=len(rows)),**kwargs))


def encoded(rows=None,completed=True):
    rows=[pair(i) for i in range(3)] if rows is None else rows
    header=dict(kind='header',schema=SCHEMA,origin='synthetic',host_clock='time.monotonic_ns',
                host_epoch='host-A',ros_epoch='ros-A',source_context='unit fixture; no hardware')
    records=[header]+[dict(kind='pair',**asdict(r)) for r in rows]+[
        dict(kind='footer',completed=completed,expected_count=len(rows),written_count=len(rows))]
    return ('\n'.join(json.dumps(r) for r in records)+'\n').encode()


def test_reading_order_brackets_only_the_ros_time_call():
    steps=[]
    values=iter([100,120])
    def monotonic():steps.append('host');return next(values)
    def ros():steps.append('ros');return 1000
    def mode():steps.append('mode');return 'ROS_TIME',False
    result=capture_pair(0,'host-A','ros-A',monotonic_ns=monotonic,ros_now_ns=ros,read_mode=mode)
    assert steps==['mode','host','ros','host','mode']
    assert (result.host_before_ns,result.ros_ns,result.host_after_ns)==(100,1000,120)


def test_constant_offset_uses_integer_brackets_not_midpoint_stamp():
    result=audit()
    assert result['record_integrity_valid'] and result['constant_offset_compatible']
    assert result['diagnostic_offset_intersection_ns']==[1_799_999_999_999_999_500,1_800_000_000_000_000_500]
    assert result['elapsed_intervals'][0]['elapsed_difference_lower_ns']==-1000
    assert result['elapsed_intervals'][0]['elapsed_difference_upper_ns']==1000
    for key in ('between_samples_continuity_verified','ros_stamp_verified','clock_mapping_applied','ready_to_drive'):
        assert result[key] is False


def test_offset_incompatibility_does_not_claim_clock_jump_cause():
    result=audit([pair(0),pair(1,delta=10000),pair(2,delta=20000)])
    assert result['record_integrity_valid']
    assert not result['constant_offset_compatible']
    assert result['diagnostic_offset_intersection_ns'] is None
    assert result['sampled_ros_regression_sequences']==[]


def test_ros_pause_is_valid_capture_but_not_physical_time_mapping():
    rows=[replace(pair(i,active=True),ros_ns=1_000_000_000) for i in range(3)]
    result=audit(rows)
    assert result['record_integrity_valid'] and result['sampled_ros_pause_sequences']==[1,2]
    assert not result['constant_offset_compatible']
    assert not result['physical_time_source_observed']


def test_ros_regression_preserved_as_event_not_retimed():
    rows=[replace(pair(0,active=True),ros_ns=200),replace(pair(1,active=True),ros_ns=100)]
    result=audit(rows)
    assert result['record_integrity_valid']
    assert result['sampled_ros_regression_sequences']==[1]
    assert result['elapsed_intervals'][0]['ros_delta_ns']==-100


def test_uninitialized_ros_zero_is_recorded_not_mapped_to_host():
    result=audit([replace(pair(i,active=True),ros_ns=0) for i in range(3)])
    assert result['record_integrity_valid']
    assert result['sampled_zero_ros_sequences']==[0,1,2]
    assert result['sampled_ros_pause_sequences']==[1,2]
    assert not result['constant_offset_compatible']
    assert not result['ros_stamp_verified']


@pytest.mark.parametrize('change_inside',[False,True])
def test_mode_switch_is_explicit_and_disallows_offset(change_inside):
    rows=[pair(0),replace(pair(1),ros_active_before=not change_inside,ros_active_after=True)]
    result=audit(rows)
    assert result['sampled_mode_change_sequences']==[1]
    assert not result['constant_offset_compatible']


@pytest.mark.parametrize('field',['host_epoch','ros_epoch'])
def test_epoch_changes_are_not_merged(field):
    result=audit([pair(0),replace(pair(1),**{field:'new'})])
    assert not result['record_integrity_valid']
    assert 'clock_epoch_changed' in result['errors']
    assert not result['constant_offset_compatible']


def test_host_overlap_or_regression_invalidates_capture():
    result=audit([pair(1),replace(pair(0),sequence=2)])
    assert not result['record_integrity_valid']
    assert 'host_bracket_order_violation' in result['errors']


@pytest.mark.parametrize('sequences',[(0,2),(0,0),(1,2)])
def test_missing_duplicate_or_nonzero_start_sequence_is_invalid(sequences):
    result=audit([replace(pair(i),sequence=s) for i,s in enumerate(sequences)])
    assert not result['record_integrity_valid']
    assert not result['recorded_sequence_complete']


def test_incomplete_capture_and_count_mismatch():
    result=audit(completed=False,expected_count=4)
    assert result['errors']==['capture_incomplete','sample_count_mismatch']


@pytest.mark.parametrize('kwargs',[dict(host_before_ns=-1),dict(host_after_ns=0),dict(sequence=True),
    dict(ros_ns=1.5),dict(ros_active_before=1),dict(clock_type_before='unknown'),
    dict(clock_type_before='SYSTEM_TIME',ros_active_before=True),dict(host_epoch='')])
def test_pair_rejects_invalid_units_modes_and_brackets(kwargs):
    with pytest.raises(ValueError):replace(pair(0),**kwargs)


def test_standalone_steady_clock_capture_is_not_ros_epoch_mapping():
    rows=[replace(pair(i),clock_type_before='STEADY_TIME',clock_type_after='STEADY_TIME') for i in range(3)]
    result=audit(rows)
    assert result['record_integrity_valid'] and not result['constant_offset_compatible']


def test_file_contract_and_input_immutability():
    data=encoded()
    before=data
    result=audit_bytes(data)
    assert data==before and result['samples']==3 and result['origin']=='synthetic'


@pytest.mark.parametrize('bad',[
    encoded().replace(b'"sequence": 0',b'"sequence": 0, "sequence": 0'),
    encoded().replace(b'"sequence": 0',b'"sequence": NaN'),
    b'\n'.join(encoded().splitlines()[:-1]),
    encoded().replace(b'"host_clock": "time.monotonic_ns"',b'"host_clock": "ros_clock"'),
    encoded().replace(b'"written_count": 3',b'"written_count": 2'),
    encoded()+b'\n',
])
def test_malformed_or_ambiguous_files_rejected(bad):
    with pytest.raises((ValueError,TypeError)):audit_bytes(bad)


def test_file_incomplete_footer_is_diagnosed_not_accepted():
    result=audit_bytes(encoded(completed=False))
    assert not result['record_integrity_valid']
