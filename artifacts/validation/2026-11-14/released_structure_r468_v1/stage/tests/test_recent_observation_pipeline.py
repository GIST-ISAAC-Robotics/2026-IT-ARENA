import math
from types import SimpleNamespace

import pytest
import validate_pipeline_faults_ros as validation


def test_profile_is_explicit_and_used_by_independent_roles():
    parameters=validation.controller_profile_parameters('g1_recent_observation_candidate')
    assert parameters['g1_recent_observation_enabled'] is True
    assert parameters['g1_corridor_fallback_enabled'] is True
    assert parameters['g1_boundary_evidence_mode']=='corner_distance_candidate'
    assert parameters['g1_occupancy_timing_mode']=='per_ray_candidate'
    assert validation.controller_profile_parameters('r0')=={}
    assert 'g1_recent_observation_enabled' not in validation.controller_profile_parameters('g1_per_ray_candidate')


def test_side_fault_does_not_fill_original_unknown_or_mutate_input():
    source=[1.]*500
    source[11]=math.inf
    result=validation.apply_scan_fault(source,'side_missing')
    assert source[375]==1. and math.isinf(source[11])
    assert math.isinf(result[375]) and math.isinf(result[11])
    assert result[:375]==source[:375] and result[376:]==source[376:]
    with pytest.raises(ValueError):validation.apply_scan_fault([1.]*400,'side_missing')


def test_side_phase_requires_both_nodes_and_returns_to_full_observation():
    obj=validation.PipelineValidator.__new__(validation.PipelineValidator)
    obj.report={}
    obj.sensor_fault='none'
    obj.mark=lambda name:{'sequence':1,'name':name}
    evidence=dict(mode='g1_follow',occupancy_timing_mode='per_ray_candidate',unfiltered_occupancy=True,
                  occupied_points=416,occupancy_padding_points=416,recent_observation_selected=True)
    data={'drive':.2,'safe':.2,'applied':.2,'autonomy':evidence,'safety':{'recent_observation':{'supported':True}}}
    checked=[]
    obj.fresh=lambda key,seq,predicate:predicate(data[key])
    def wait(predicate,name):
        assert obj.sensor_fault=='side_missing' and predicate()
        data['safety']['recent_observation']['supported']=False
        assert not predicate()
        data['safety']['recent_observation']['supported']=True
    obj.wait=wait
    def finish(case,expected,duration):
        assert duration==.3 and all(p(data[k]) for k,p in expected.items())
        for k in ('drive','safe','applied'):
            assert not expected[k](0.) and not expected[k](.21)
        checked.append(case['name'])
    obj.finish=finish
    def moving(name):
        assert obj.sensor_fault=='none'
        checked.append(name)
    obj.moving=moving
    obj.run_recent_observation()
    assert checked==['recent_side_observation_independent_cap','recent_side_observation_fresh_full_recovery']


def test_evidence_expiry_is_separate_from_unchanged_continuity_check():
    obj=validation.PipelineValidator.__new__(validation.PipelineValidator)
    obj.report={'recent_check_mode':'evidence_expiry'}
    called=[]
    obj.run_recent_observation_evidence_expiry=lambda:called.append('explicit')
    obj.run_recent_observation()
    assert called==['explicit']


def test_point_evidence_demands_independent_unknown_check_and_later_expiry():
    obj=validation.PipelineValidator.__new__(validation.PipelineValidator)
    obj.t=1.;obj.sensor_fault='none'
    evidence=dict(mode='g1_follow',occupancy_timing_mode='per_ray_candidate',unfiltered_occupancy=True,
                  occupied_points=416,occupancy_padding_points=416,recent_observation_selected=True)
    data={'drive':.01,'safe':.01,'applied':.01,'autonomy':evidence,
          'safety':{'recent_observation':{'supported':True},'front_observation':{'reason':'front_unobserved'}}}
    obj.monitor=SimpleNamespace(latest=data)
    record={}
    def mark(name):record.update(name=name,sequence=1);return record
    obj.mark=mark;obj.capture_zeros=lambda case:None
    obj.fresh=lambda key,seq,predicate:predicate(data[key])
    def wait(predicate,label):
        assert predicate()
        for topic in ('drive','safe','applied'):
            saved=data[topic]
            for bad in (0.,.21):
                data[topic]=bad;assert not predicate()
            data[topic]=saved
        data['safety']['front_observation']['reason']='ok';assert not predicate()
        data['safety']['front_observation']['reason']='front_unobserved'
        data['safety']['recent_observation']['supported']=False;assert not predicate()
        data['safety']['recent_observation']['supported']=True
    obj.wait=wait;checked=[]
    def stop(name,action,expected):
        action();assert obj.sensor_fault=='scan_drop'
        assert expected['safe'](0.) and not expected['safe'](.01)
        assert expected['safety']({'reason':'stale_input'})
        assert not expected['safety']({'reason':'clear'})
        checked.append(name)
    obj.stop_case=stop
    def moving(name):assert obj.sensor_fault=='none';checked.append(name)
    obj.moving=moving
    obj.run_recent_observation_evidence_expiry()
    assert record['passed'] and not record['continuous_progress_verified']
    assert not record['minimum_progress_required']
    assert checked==['recent_evidence_expiry_stops','recent_evidence_fresh_full_recovery']
