import math
import itertools

import numpy as np
import pytest

from probe_observed_curve_length import ordered_length_lower_bound


def test_straight_line_does_not_accumulate_point_errors():
    p=np.column_stack((np.linspace(0.,1.,101),np.zeros(101)))
    result=ordered_length_lower_bound(p,.03)
    assert result['lower_bound_m']==pytest.approx(.94)
    assert result['witness_indices']==[0,100]


def test_dp_matches_all_subsequences():
    p=np.random.default_rng(5).uniform(-.2,.2,(9,2))
    result=ordered_length_lower_bound(p,.03)
    expected=0.
    for count in range(2,len(p)+1):
        for indices in itertools.combinations(range(len(p)),count):
            expected=max(expected,float(np.maximum(0.,np.linalg.norm(np.diff(p[list(indices)],axis=0),axis=1)-.06).sum()))
    assert result['lower_bound_m']==pytest.approx(expected)


@pytest.mark.parametrize('seed',range(8))
def test_bounded_point_noise_never_exceeds_true_arc_length(seed):
    theta=np.linspace(0.,math.pi*.75,80)
    truth=.5*np.column_stack((np.sin(theta),1-np.cos(theta)))
    rng=np.random.default_rng(seed)
    directions=rng.uniform(-math.pi,math.pi,len(truth))
    radii=rng.uniform(0.,.03,len(truth))
    measured=truth+np.column_stack((np.cos(directions),np.sin(directions)))*radii[:,None]
    before=measured.copy()
    bound=ordered_length_lower_bound(measured,.03)
    assert bound['lower_bound_m']<=.5*math.pi*.75+1e-12
    np.testing.assert_array_equal(measured,before)
    assert bound['witness_indices']==sorted(set(bound['witness_indices']))


def test_noise_zigzag_does_not_inflate_straight_surface():
    p=np.column_stack((np.linspace(0.,.2,101),np.where(np.arange(101)%2,.03,-.03)))
    assert np.linalg.norm(np.diff(p,axis=0),axis=1).sum()>5.
    assert ordered_length_lower_bound(p,.03)['lower_bound_m']<=.2


def test_short_corrugated_object_is_counterexample_to_wall_identity():
    p=np.column_stack((np.linspace(0.,.2,41),np.where(np.arange(41)%2,.07,-.07)))
    assert np.linalg.norm(np.ptp(p,axis=0))<.25
    assert ordered_length_lower_bound(p,.03)['lower_bound_m']>.65


def test_empty_and_single_point():
    for p in (np.empty((0,2)),[[0.,0.]]):
        assert ordered_length_lower_bound(p,.03)['lower_bound_m']==0.


@pytest.mark.parametrize('p,error',[(0.,.03),([[0.,math.nan]],.03),([[0.,0.]],-.1),
                                    ([[0.,0.]],math.inf),([[0.,0.]],True)])
def test_invalid_input(p,error):
    with pytest.raises(ValueError):ordered_length_lower_bound(p,error)
