import copy
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[4]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
import small_alignment_candidate as trial


def fit(dx=.003):
    return dict(accepted=False,reason='weak_improvement',steps=[
        dict(step_m=[dx,0.],residual_rms_m=.0048,fraction=.8,normal_eigen_ratio=.3),
        dict(step_m=[0.,0.],residual_rms_m=.0038,fraction=.8,normal_eigen_ratio=.3)])


def test_small_low_residual_consistent_partitions_are_compared(monkeypatch):
    responses=iter([fit(),fit(.0034),fit(.0027)])
    counts=[]
    def fake(old,*args):
        counts.append(len(old));return next(responses)
    monkeypatch.setattr(trial,'original_translation',fake)
    result=trial.surface_translation(np.zeros((200,2)),None,None)
    assert result['accepted'] and result['relative_gain_rejected']
    assert result['translation_m']==[.003,0.] and counts==[200,100,100]
    assert result['split_difference_m']==pytest.approx(.0007)


@pytest.mark.parametrize('reason',['localized_support','translation_unobservable',
    'unsettled_or_high_residual','translation_outside_bound','few_surface_matches'])
def test_other_geometric_rejections_are_not_rescued(monkeypatch,reason):
    original={**fit(),'reason':reason}
    monkeypatch.setattr(trial,'original_translation',lambda *args:original)
    assert trial.surface_translation(np.zeros((200,2)),None,None) is original


def test_accepted_legacy_result_is_returned_without_reprocessing(monkeypatch):
    original={**fit(),'accepted':True,'reason':'bounded_surface_correspondence'}
    calls=[]
    def fake(*args):calls.append(1);return original
    monkeypatch.setattr(trial,'original_translation',fake)
    assert trial.surface_translation(None,None,None) is original and len(calls)==1


@pytest.mark.parametrize('fault',['large_shift','high_residual','unsettled','weak_support',
    'unobservable','worse','nan','missing','negative_residual'])
def test_absolute_quality_gates_remain(fault):
    value=fit()
    if fault=='large_shift':value['steps'][0]['step_m']=[.011,0.]
    if fault=='high_residual':value['steps'][-1]['residual_rms_m']=.0051
    if fault=='unsettled':value['steps'][-1]['step_m']=[.0001,0.]
    if fault=='weak_support':value['steps'][0]['fraction']=.64
    if fault=='unobservable':value['steps'][-1]['normal_eigen_ratio']=.019
    if fault=='worse':value['steps'][0]['residual_rms_m']=.003
    if fault=='nan':value['steps'][0]['step_m']=[float('nan'),0.]
    if fault=='missing':del value['steps'][0]['fraction']
    if fault=='negative_residual':value['steps'][-1]['residual_rms_m']=-.001
    assert trial.small_converged(value) is None


@pytest.mark.parametrize('fault',['split_disagreement','split_failure','insufficient_points'])
def test_independent_partition_support_is_required(monkeypatch,fault):
    original=fit();pieces=[fit(.0048),fit(.0012)]
    if fault=='split_failure':pieces[0]['reason']='translation_unobservable'
    responses=iter([original,*pieces])
    monkeypatch.setattr(trial,'original_translation',lambda *args:next(responses))
    result=trial.surface_translation(np.zeros((99 if fault=='insufficient_points' else 200,2)),None,None)
    assert result is original and not result['accepted']
