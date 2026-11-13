import copy
import json
import math
from pathlib import Path

import pytest

from native_traffic_contract import NativeSamplingAudit, native_received_row
from arena_autonomy.measurement_scan import MeasurementScan

ROOT = Path(__file__).resolve().parents[1]
PROFILE = json.loads((ROOT/'config/tests/c1_standard10_native_angles_v1.json').read_text())


def fixture():
    audit = NativeSamplingAudit(PROFILE, 10.)
    records, received = [], []
    for j, index in enumerate((0, 2, 4)):
        angles = audit.patterns[index];n = len(angles);first = 2+j*.1
        times = tuple(first+i*.1/n for i in range(n))
        source = dict(first_ray_stamp_s=first, last_ray_stamp_s=times[-1],
            samples=n, template_index=index, max_capture_time_error_s=.002,
            max_source_angle_quantization_rad=0., discarded_source_gaps=0,
            model='native_sequential_v1', time_model='period_divided_by_count',
            hardware_timing_verified=False)
        scan = MeasurementScan((1.,)*(n-1)+(math.nan,), angles, times,
                               time_basis='simulation_acquisition')
        records.append(source);received.append(native_received_row(scan))
    return audit, records, received


def test_three_actual_c1_template_lengths_and_unknown_preserved():
    audit, records, received = fixture()
    result = audit.audit(records, received)
    assert result['verified'] and result['sample_counts'] == [504, 508, 512]
    assert received[0]['ranges'][-1] is None
    assert received[0]['time_increment'] is None
    assert not result['hardware_timing_verified']
    assert len(received[0]['angles_rad']) == 512


@pytest.mark.parametrize('fault', ['model', 'hardware', 'count', 'template', 'end_time',
    'source_gap', 'capture_error', 'angular_error', 'duplicate', 'time_basis', 'reversed',
    'times_sorted_wrong', 'fixed_500', 'shifted_angle', 'unknown_deleted', 'bad_frame', 'receipt_time'])
def test_wrong_representation_or_timing_cannot_pass(fault):
    audit, records, received = fixture();r=records[1];s=received[1]
    if fault == 'model': r['model'] = 'sequential'
    if fault == 'hardware': r['hardware_timing_verified'] = True
    if fault == 'count': r['samples'] = 500
    if fault == 'template': r['template_index'] = -1
    if fault == 'end_time': r['last_ray_stamp_s'] += .001
    if fault == 'source_gap': r['discarded_source_gaps'] = 1
    if fault == 'capture_error': r['max_capture_time_error_s'] = .003
    if fault == 'angular_error': r['max_source_angle_quantization_rad'] = .1
    if fault == 'duplicate': received[1] = copy.deepcopy(received[0])
    if fault == 'time_basis': s['time_basis'] = 'validated_sensor_acquisition'
    if fault == 'reversed': s['angles_rad'].reverse()
    if fault == 'times_sorted_wrong': s['acquired_at_s'].reverse()
    if fault == 'fixed_500': s['ranges'] = s['ranges'][:500]
    if fault == 'shifted_angle': s['angles_rad'][25] += .001
    if fault == 'unknown_deleted': s['ranges'].pop()
    if fault == 'bad_frame': s['source_frame_id'] = 'map'
    if fault == 'receipt_time': s['stamp_s'] += .05
    assert not audit.audit(records, received)['verified']


def test_empty_and_reordered_source_fail():
    audit, records, received = fixture()
    assert not audit.audit([], received)['verified']
    assert not audit.audit(records, [])['verified']
    assert not audit.audit(records[::-1], received)['verified']


def test_declared_rate_must_match_selected_template_and_time_basis():
    with pytest.raises(ValueError): NativeSamplingAudit(PROFILE, 12.)
    profile = copy.deepcopy(PROFILE);profile['hardware_timing_verified'] = True
    with pytest.raises(ValueError): NativeSamplingAudit(profile, 10.)


def test_wrapped_source_angle_is_equivalent_not_resorted():
    audit, records, received = fixture()
    received[0]['angles_rad'][0] += math.tau
    assert audit.audit(records, received)['verified']


def test_received_evidence_preserves_each_nonfinite_kind():
    audit, records, received = fixture()
    row = received[0]
    scan = MeasurementScan((math.inf, -math.inf, math.nan, 0.) + (1.,) * (len(row['ranges']) - 4),
        tuple(row['angles_rad']), tuple(row['acquired_at_s']), time_basis='simulation_acquisition')
    received[0] = native_received_row(scan)
    assert received[0]['ranges'][:4] == ['+inf', '-inf', None, 0.]
    assert audit.audit(records, received)['verified']
    # 기록 형식만 검사한다. 과거 null로부터 원값을 추정하지 않는다.
    assert received[1]['ranges'][-1] is None


@pytest.mark.parametrize('value', ['NaN', 'Infinity', '+Infinity', True, {}, math.inf, math.nan])
def test_received_evidence_rejects_ambiguous_or_unencoded_ranges(value):
    audit, records, received = fixture()
    received[0]['ranges'][0] = value
    assert not audit.audit(records, received)['verified']


def test_cumulative_loss_does_not_erase_completed_array_diagnostics():
    audit, records, received = fixture()
    for r in records: r['discarded_source_gaps']=2
    result=audit.audit(records,received)
    assert not result['verified'] and result['source_errors']==[0,1,2]
    payload=result['completed_payload']
    assert payload['received_arrays_match_completed_sources']
    assert payload['all_completed_sources_received']
    assert not payload['acquisition_loss_free']
    assert payload['acquisition_gap_count']==2
    assert [r['discarded_source_gaps'] for r in records]==[2,2,2]


def test_missing_completed_source_is_reported_separately_from_received_correspondence():
    audit, records, received = fixture()
    result=audit.audit(records,received[1:])['completed_payload']
    assert result['received_arrays_match_completed_sources']
    assert not result['all_completed_sources_received']
    assert result['unreceived_completed_sources']==1
    assert result['acquisition_loss_free']


@pytest.mark.parametrize('values',[[0,2,1],[0,-1,0],[0,True,2],[0,None,2]])
def test_invalid_acquisition_counter_is_not_reported_loss_free(values):
    audit, records, received=fixture()
    for row,value in zip(records,values):row['discarded_source_gaps']=value
    payload=audit.audit(records,received)['completed_payload']
    assert not payload['acquisition_counter_valid']
    assert not payload['acquisition_loss_free']
    assert payload['acquisition_gap_count'] is None


@pytest.mark.parametrize('fault',['angle','duplicate','source_order','unknown_removed'])
def test_completed_payload_diagnostics_still_reject_corruption(fault):
    audit, records, received=fixture()
    for r in records:r['discarded_source_gaps']=1
    if fault=='angle':received[0]['angles_rad'][3]+=.1
    if fault=='duplicate':received[1]=copy.deepcopy(received[0])
    if fault=='source_order':records.reverse()
    if fault=='unknown_removed':received[0]['ranges'].pop()
    result=audit.audit(records,received)
    assert not result['verified']
    assert not result['completed_payload']['received_arrays_match_completed_sources']
