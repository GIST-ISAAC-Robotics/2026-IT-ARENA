import copy
import json
import math
import pytest
from arena_vehicle_interface.c1_native_preview import C1NativePreview
from arena_autonomy.native_lidar_input import decode_measurements
from arena_autonomy.local_scene import scene_from_measurements


def sample(scan=0, first_ns=10_000_000_000):
    return dict(schema='c1_sdk_native_preview_v1', scan=scan,
        begin_monotonic_ns=first_ns-1_000_000, end_monotonic_ns=first_ns+100_000_000,
        sdk_estimated_first_us=first_ns//1000, us_per_sample=200.,
        nodes=[[65530, 4000, 12, 1], [10, 0, 0, 0], [100, 8000, 0, 0], [230, 48004, 12, 0]])


def convert(clock, batch, *, offset=1_700_000_000_000_000_000):
    mono=batch['end_monotonic_ns']+1_000_000
    return clock.convert(batch, receipt_monotonic_ns=mono, receipt_ros_ns=mono+offset)


def test_native_order_nonuniform_angles_invalid_ranges_and_unverified_time_roundtrip():
    payload, status=convert(C1NativePreview(source_epoch=7), sample())
    scan=decode_measurements(json.dumps(payload), expected_time_basis='unverified_estimate')
    assert scan.ranges_m[0]==1 and scan.ranges_m[1]==math.inf and math.isnan(scan.ranges_m[2])
    assert scan.ranges_m[3]>12  # 그대로 보존하고 하류에서 범위 밖으로 분류한다.
    assert scan.angles_rad==tuple(math.pi-v[0]*math.pi/32768 for v in sample()['nodes'])
    assert scan.source_epoch==7 and not status['time_contract_verified']
    assert len(scan.acquired_at_s)==4 and all(b>a for a,b in zip(scan.acquired_at_s,scan.acquired_at_s[1:]))
    with pytest.raises(ValueError,match='differs'):
        decode_measurements(json.dumps(payload),expected_time_basis='simulation_acquisition')


def test_nominal_overlap_is_reported_not_silently_fixed_or_discarded():
    c=C1NativePreview(source_epoch=2)
    a=sample(); a['nodes']*=130
    convert(c,a)
    payload,status=convert(c,sample(1,10_100_000_000))
    assert status['nominal_overlap_ns']==3_800_000
    assert payload['acquired_at_s'][0]==1_700_000_010.1
    assert payload['source_epoch']==2 and c.counts['nominal_overlap_batches']==1


@pytest.mark.parametrize('kind',['sequence','clock'])
def test_restart_and_ros_clock_step_get_a_new_source_epoch(kind):
    c=C1NativePreview(source_epoch=2); convert(c,sample())
    a=sample(1,10_100_000_000)
    if kind=='sequence': a['scan']=0
    payload,status=convert(c,a,offset=1_700_000_000_000_000_000+(200_000_000 if kind=='clock' else 0))
    assert payload['source_epoch']==3 and status['reset_reason']


@pytest.mark.parametrize('field,value',[
    ('us_per_sample',196.),('sdk_estimated_first_us',True),('scan',-1),
    ('nodes',[[65536,4000,12,1],[0,4000,12,0]]),('nodes',[[0,4000,12,1]]),
    ('nodes',[[0,4000,True,1],[100,4000,12,0]])])
def test_invalid_batch_never_advances_state(field,value):
    c=C1NativePreview(source_epoch=2);a=sample();a[field]=value
    with pytest.raises(ValueError):convert(c,a)
    assert c.last is None and not c.counts


def test_transport_delays_do_not_shift_sdk_relative_time_or_hide_age():
    c=C1NativePreview(source_epoch=9);a=sample()
    mono=a['end_monotonic_ns']+90_000_000;offset=1_700_000_000_000_000_000
    payload,status=c.convert(a,receipt_monotonic_ns=mono,receipt_ros_ns=mono+offset)
    assert payload['acquired_at_s'][0]==1_700_000_010.
    assert status['transport_after_grab_ns']==90_000_000
    assert status['sdk_first_age_at_receipt_ns']==190_000_000


@pytest.mark.parametrize('distance,quality,encoded',[(0,0,'+inf'),(0,12,'+inf'),(4000,0,None),(4000,12,1.)])
def test_return_encoding_does_not_create_free_space(distance,quality,encoded):
    data=sample();data['nodes'][0]=[32768,distance,quality,0]
    payload,status=convert(C1NativePreview(source_epoch=7),data)
    assert payload['ranges_m'][0]==encoded
    scan=decode_measurements(json.dumps(payload,allow_nan=False),expected_time_basis='unverified_estimate')
    scene=scene_from_measurements(scan.ranges,angles_rad=scan.angles_rad,range_min=scan.range_min,
        range_max=scan.range_max,acquired_at_s=scan.acquired_at_s,t_ref_s=scan.acquired_at_s[-1])
    assert scene.rays[0].status==('return' if distance and quality else 'unknown')
    if not distance or not quality:assert scene.rays[0].hit_xy_m is None
    assert status['time_contract_verified'] is False and status['nonreturn_implies_free_space'] is False
    assert status['zero_distance_points']==(2 if not distance else 1)
    assert status['invalid_quality_positive_distance_points']==(2 if distance and not quality else 1)
    with pytest.raises(ValueError):decode_measurements(json.dumps(payload),expected_time_basis='simulation_acquisition')

def test_legacy_unknown_has_no_raw_basis_to_relabel():
    payload,_=convert(C1NativePreview(source_epoch=7),sample())
    payload['ranges_m'][1]=None
    scan=decode_measurements(json.dumps(payload),expected_time_basis='unverified_estimate')
    assert math.isnan(scan.ranges[1])
