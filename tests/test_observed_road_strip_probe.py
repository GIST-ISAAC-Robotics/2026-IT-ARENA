import math
import numpy as np
import pytest
from observed_road_strip_probe import side_bound,road_strip


def walls(bias=0.):
    x=np.linspace(-1.,1.,401)
    p=np.column_stack((x,np.full_like(x,.425)))
    r=np.column_stack((x,np.full_like(x,-.425)))
    return p+np.column_stack((np.zeros_like(x),np.full_like(x,bias))),r-np.column_stack((np.zeros_like(x),np.full_like(x,bias)))


@pytest.mark.parametrize('bias',[-.03,0.,.03])
def test_full_bias_never_expands_proven_road_outside_true_road(bias):
    left,right=walls(bias)
    result=road_strip(left,right,[-.1,0.,.1],slope_bound=.08)
    assert result['all_supported'] and not result['driving_authorized']
    for row in result['samples']:
        assert -.225<=row['lower_m']<row['upper_m']<=.225


def test_arbitrary_bounded_errors_on_lipschitz_curve_preserve_true_wall_bounds():
    rng=np.random.default_rng(128)
    x=np.linspace(-1.,1.,501)
    truth=lambda x:.425+.02*np.sin(3*x)
    p=np.column_stack((x,truth(x)))
    direction=rng.uniform(-math.pi,math.pi,len(x));radius=rng.uniform(0.,.03,len(x))
    measured=p+radius[:,None]*np.column_stack((np.cos(direction),np.sin(direction)))
    q=np.linspace(-.8,.8,81)
    left=side_bound(measured,q,side='left',slope_bound=.06,maximum_gap_m=.14)
    assert all(r['supported'] for r in left)
    assert all(r['bound_m']<=truth(r['x_m'])+1e-12 for r in left)
    right=side_bound(-measured[::-1],q,side='right',slope_bound=.06,maximum_gap_m=.14)
    assert all(r['supported'] for r in right)
    assert all(r['bound_m']>=-truth(-r['x_m'])-1e-12 for r in right)


def test_unknown_gap_and_ends_are_not_extrapolated():
    left,right=walls()
    left=left[np.abs(left[:,0])>.2]
    result=road_strip(left,right,[0.,1.1],slope_bound=.08)
    assert not result['all_supported']
    assert result['samples'][0]['left']['reason']=='observation_gap'
    assert result['samples'][1]['left']['reason']=='no_robust_bracket'
    assert all(row['lower_m'] is None for row in result['samples'])


def test_empty_side_is_not_inferred_from_other_wall_or_nominal_width():
    _,right=walls()
    result=road_strip(np.empty((0,2)),right,[0.],slope_bound=.08)
    assert not result['all_supported'] and result['samples'][0]['lower_m'] is None


def test_closer_surface_shrinks_bound_and_remains_conditional_not_wall_classifier():
    left,right=walls();left[:,1]-=.15
    result=road_strip(left,right,[0.],slope_bound=.08)
    assert result['samples'][0]['upper_m']<.1
    assert 'correct same-side walls' in result['conditional_on']


@pytest.mark.parametrize('kw',[{'slope_bound':-.1},{'error_m':-1.},{'maximum_gap_m':0.}])
def test_bad_bounds_rejected(kw):
    left,right=walls();args={'slope_bound':.08};args.update(kw)
    with pytest.raises(ValueError):road_strip(left,right,[0.],**args)
