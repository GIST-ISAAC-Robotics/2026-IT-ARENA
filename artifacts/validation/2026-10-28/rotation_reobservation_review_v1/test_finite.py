import numpy as np
import copy
import pytest

import finite_segment_candidate as trial
from test_consistent import surfaces
from test_candidate import fit


def test_small_tangential_crossing_is_measured_to_real_segment_endpoint():
    mid,current,ids=surfaces()
    # Same finite segments score the actual small correction, without re-matching.
    old=mid-np.array([.003,.002])
    correct=trial.common_support_loss(old,current,ids,[.003,.002])
    assert correct['passed'] and correct['after_rms_m']<1e-12
    assert correct['matches']==correct['before_matches']


def test_passing_end_of_wall_is_not_zero_error_on_infinite_extension():
    mid,current,ids=surfaces()
    result=trial.common_support_loss(mid,current,ids,[.15,0.])
    assert not result['passed'] and result['after_rms_m']>.005
    assert result['beyond_initial_segment']>0


def test_wrong_motion_on_clean_surface_cannot_improve_score():
    mid,current,ids=surfaces()
    result=trial.common_support_loss(mid,current,ids,[.003,.002])
    assert not result['passed'] and result['after_rms_m']>result['before_rms_m']


def test_missing_rays_do_not_create_finite_surface():
    mid,current,ids=surfaces()
    assert not trial.common_support_loss(mid,current,ids*2,[0.,0.])['passed']


@pytest.mark.parametrize('fault',['large_shift','high_residual','unsettled','weak_support',
    'unobservable','nan','missing','negative_residual'])
def test_small_alignment_keeps_absolute_and_observability_limits(fault):
    value=fit()
    if fault=='large_shift':value['steps'][0]['step_m']=[.011,0.]
    if fault=='high_residual':value['steps'][-1]['residual_rms_m']=.0051
    if fault=='unsettled':value['steps'][-1]['step_m']=[.0001,0.]
    if fault=='weak_support':value['steps'][0]['fraction']=.64
    if fault=='unobservable':value['steps'][-1]['normal_eigen_ratio']=.019
    if fault=='nan':value['steps'][0]['step_m']=[float('nan'),0.]
    if fault=='missing':del value['steps'][0]['fraction']
    if fault=='negative_residual':value['steps'][-1]['residual_rms_m']=-.001
    assert trial.bounded_solution(value) is None


@pytest.mark.parametrize('reason',['localized_support','translation_unobservable',
    'unsettled_or_high_residual','translation_outside_bound','few_surface_matches'])
def test_non_relative_gain_failures_are_not_changed(monkeypatch,reason):
    value={**fit(),'reason':reason}
    monkeypatch.setattr(trial,'original_translation',lambda *args:value)
    assert trial.surface_translation(None,None,None) is value


def test_small_alignment_requires_two_holdouts_and_does_not_move_inputs(monkeypatch):
    mid,current,ids=surfaces();old=mid-np.array([.003,0.])
    before=[v.copy() for v in (old,current,ids)]
    monkeypatch.setattr(trial,'original_translation',lambda *args:copy.deepcopy(fit()))
    result=trial.surface_translation(old,current,ids)
    assert result['accepted'] and result['reason']=='small_finite_support_consistent'
    for original,value in zip(before,(old,current,ids)):np.testing.assert_array_equal(original,value)
    calls=[]
    def loss(*args):
        calls.append(1);return dict(passed=len(calls)<3)
    monkeypatch.setattr(trial,'common_support_loss',loss)
    assert not trial.surface_translation(old,current,ids)['accepted']
    assert len(calls)==3


def test_tangent_motion_past_individual_segment_stays_finite():
    # Three separated long sides provide two translation axes. Each old point is
    # near its original segment endpoint; tangent motion crosses many endpoints.
    mid,current,ids=surfaces();old=mid+np.array([.009,-.002])
    result=trial.common_support_loss(old,current,ids,[.003,.002])
    assert result['beyond_initial_segment']>0
    # Tangent crossing has a nonzero endpoint cost; it is never an infinite plane.
    assert 0<result['after_rms_m'] and not result['passed']
    assert result['matches']==result['before_matches']
