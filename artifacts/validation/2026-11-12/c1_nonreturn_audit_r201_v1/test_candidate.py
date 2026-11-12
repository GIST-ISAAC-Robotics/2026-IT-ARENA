"""원시 거리0/무효 품질 구분, 기존 unknown·시각 승인 경계의 반례."""
import importlib.util
import json
import math
from pathlib import Path
import pytest
from arena_autonomy.native_lidar_input import decode_measurements
from arena_autonomy.local_scene import scene_from_measurements
from test_c1_native_preview import sample,convert

spec=importlib.util.spec_from_file_location('r201_candidate',Path(__file__).with_name('candidate_c1_native_preview.py'))
candidate=importlib.util.module_from_spec(spec);spec.loader.exec_module(candidate)

@pytest.mark.parametrize('distance,quality,encoded',[(0,0,'+inf'),(0,12,'+inf'),(4000,0,None),(4000,12,1.)])
def test_return_encoding_does_not_create_free_space(distance,quality,encoded):
    data=sample();data['nodes'][0]=[32768,distance,quality,0]
    payload,status=convert(candidate.C1NativePreview(source_epoch=7),data)
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
    payload,_=convert(candidate.C1NativePreview(source_epoch=7),sample())
    payload['ranges_m'][1]=None
    scan=decode_measurements(json.dumps(payload),expected_time_basis='unverified_estimate')
    assert math.isnan(scan.ranges[1])

def test_all_saved_raw_values_change_only_confirmed_zero_distance():
    root=Path(__file__).resolve().parents[4]
    d=root/'build/r166b_recovered/artifacts/validation/2026-11-12/c1_ros_preview_r166_v1/capture'
    raws=[json.loads(s) for s in (d/'sdk_raw.jsonl').open()]
    records=[json.loads(s) for s in (d/'converted.jsonl').open()]
    adapter=None;changed=0
    for raw,old in zip(raws,records):
        payload=old['payload']
        if isinstance(payload,str):payload=json.loads(payload)
        if adapter is None:adapter=candidate.C1NativePreview(source_epoch=payload['source_epoch'])
        new,status=adapter.convert(raw,receipt_monotonic_ns=old['receipt_monotonic_ns'],receipt_ros_ns=old['receipt_ros_ns'])
        expected=dict(payload,ranges_m=['+inf' if node[1]==0 else value for node,value in zip(raw['nodes'],payload['ranges_m'])])
        assert new==expected
        changed+=status['zero_distance_points']
        assert status['nominal_overlap_ns']==old['diagnostic']['nominal_overlap_ns']
        assert status['nominal_end_after_receipt_ns']==old['diagnostic']['nominal_end_after_receipt_ns']
    assert len(raws)==len(records)==204 and changed==22888
