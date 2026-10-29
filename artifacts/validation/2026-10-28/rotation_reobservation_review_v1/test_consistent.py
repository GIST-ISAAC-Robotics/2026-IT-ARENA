import copy
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[4]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
import consistent_alignment_candidate as trial
from test_candidate import fit


def surfaces():
    lower=np.column_stack((np.linspace(-1.,1.,101),np.full(101,-.6)))
    front=np.column_stack((np.ones(61),np.linspace(-.6,.6,61)))
    upper=np.column_stack((np.linspace(1.,-1.,101),np.full(101,.6)))
    current=np.vstack((lower,front,upper))
    ids=np.r_[np.arange(101),np.arange(110,171),np.arange(180,281)]
    mid=np.vstack([(p[:-1]+p[1:])/2 for p in (lower,front,upper)])
    return mid,current,ids


def test_same_samples_confirm_small_translation_and_reject_wrong_direction():
    mid,current,ids=surfaces();shift=np.array([.003,-.002])
    old=mid-shift
    good=trial.common_support_loss(old,current,ids,shift)
    wrong=trial.common_support_loss(old,current,ids,-shift)
    assert good['passed'] and good['after_rms_m']<1e-12
    assert good['matches']==good['before_matches']==len(old)
    assert not wrong['passed'] and wrong['after_rms_m']>wrong['before_rms_m']


@pytest.mark.parametrize('failure',['parallel','missing_rays','endpoints','nan','noninteger_ids'])
def test_observation_gaps_or_unobservable_geometry_cannot_validate_shift(failure):
    mid,current,ids=surfaces();shift=np.array([.003,-.002]);old=mid-shift
    if failure=='parallel':
        old=old[np.abs(mid[:,1])>.59]
        keep=np.abs(current[:,1])>.59;current,ids=current[keep],ids[keep]
    elif failure=='missing_rays':ids=ids*2
    elif failure=='endpoints':shift=np.array([.2,.2])
    elif failure=='nan':old[0,0]=np.nan
    elif failure=='noninteger_ids':ids=ids.astype(float)
    assert not trial.common_support_loss(old,current,ids,shift)['passed']


def test_slight_change_in_moving_match_rms_requires_fixed_support_instead():
    value=fit();value['steps'][0]['residual_rms_m']=.00436
    value['steps'][-1]['residual_rms_m']=.00454
    np.testing.assert_allclose(trial.bounded_solution(value),[.003,0.])
    # The helper alone never authorizes a match: fixed-support and holdout checks follow.
    mid,current,ids=surfaces()
    assert not trial.common_support_loss(mid,current,ids,[.003,0.])['passed']


def test_proposed_branch_preserves_accepted_legacy_and_other_rejections(monkeypatch):
    for value in [dict(accepted=True,reason='bounded_surface_correspondence'),
                  dict(accepted=False,reason='localized_support')]:
        monkeypatch.setattr(trial,'original_translation',lambda *args:value)
        assert trial.surface_translation(None,None,None) is value


def test_every_partition_must_improve_heldout_support(monkeypatch):
    mid,current,ids=surfaces();old=mid-np.array([.003,0.])
    monkeypatch.setattr(trial,'original_translation',lambda *args:copy.deepcopy(fit()))
    result=trial.surface_translation(old,current,ids)
    assert result['accepted'] and result['reason']=='small_fixed_support_consistent'
    calls=[]
    def loss(*args):
        calls.append(1);return dict(passed=len(calls)<3)
    monkeypatch.setattr(trial,'common_support_loss',loss)
    assert not trial.surface_translation(old,current,ids)['accepted']
    assert len(calls)==3


def test_partition_disagreement_cannot_override_good_full_fit(monkeypatch):
    mid,current,ids=surfaces();old=mid-np.array([.003,0.])
    responses=iter([fit(),fit(.006)])
    monkeypatch.setattr(trial,'original_translation',lambda *args:next(responses))
    assert not trial.surface_translation(old,current,ids)['accepted']
