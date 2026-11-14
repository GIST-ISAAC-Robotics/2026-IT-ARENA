"""1 mm 영역에만 전체 6도 공백 예산을 적용하며 기존 거부 조건을 유지한다."""
from dataclasses import replace
import math
import numpy as np
import pytest
from arena_autonomy.released_rear_sweep import bounded_released_rear_sweep
from test_rear_fringe_trial import case
from test_released_rear_sweep import probe
import test_traffic_release_fringe as linked

WIDE=tuple(range(449,455))


@pytest.mark.parametrize('point,missing',[((-.047,.0755),WIDE),((-.047,-.0755),tuple(range(45,51)))])
def test_six_degree_budget_only_for_submillimeter_rear_region(point,missing):
    old,out,raw,scene=probe(point=point,missing=missing)
    assert not old['accepted'] and old['reason']=='wide_nonreturn'
    assert out['accepted'] and out['policy']=='released_low_speed_body_fringe_v2'
    assert out['maximum_exterior_m']==.001
    assert out['maximum_gap_rad']==math.radians(6)
    assert out['previous_rejection']=='wide_nonreturn'
    assert not out['unknown_filled'] and out['residual_hidden_object_risk']
    assert sum(g['gap_rad'] for g in out['gaps'])<=math.radians(6)


@pytest.mark.parametrize('kwargs',[
    {'point':(-.047,.076001)}, {'point':(.001,.0755)}, {'point':(-.101,.0755)},
    {'speed':.351}, {'speed':-.01}, {'target':0.}, {'target':.351},
    {'value':math.nan}, {'value':-math.inf}, {'value':.02},
    {'neighbor':.19}, {'neighbor':.4}, {'mismatch':True},
    {'missing':tuple(range(447,457))}, {'missing':tuple(range(451,500))},
])
def test_larger_motion_fast_near_invalid_or_over_budget_is_not_allowed(kwargs):
    options=dict(point=(-.047,.0755),missing=WIDE);options.update(kwargs)
    assert not probe(**options)[1]['accepted']


def test_two_gaps_cannot_each_spend_the_full_budget():
    missing=tuple(range(449,455))+tuple(range(45,51))
    _,raw,scene=case(point=(-.047,.0755),missing=missing)
    out=bounded_released_rear_sweep(scene,raw,[[-.047,.0755],[-.047,-.0755]],
                                  np.full(500,.1),.2,.15,.2,.35,.01)
    assert not out['accepted']


def execute(monkeypatch,enabled=True,**kwargs):
    original=linked.ps.PassReleaseSelector
    monkeypatch.setattr(linked.ps,'PassReleaseSelector',lambda c:original(replace(c,released_rear_sweep_trial=enabled)))
    output=linked.output
    def with_proof(*args,**kw):
        v,a,d=output(*args,**kw)
        return v,a,{**d,'partial_observation':dict(allowed=True,reason='fully_observed')}
    monkeypatch.setattr(linked,'output',with_proof)
    return linked.execute(monkeypatch,point=(-.047,.0755),missing=WIDE,**kwargs)[1]


def test_release_retained_history_keeps_curved_baseline_with_explicit_risk(monkeypatch):
    v,a,d=execute(monkeypatch)
    assert v==.35 and a==.2
    assert d['selector_reason']=='released_bounded_rear_sweep'
    assert d['selector_released_rear_sweep']['policy']=='released_low_speed_body_fringe_v2'
    assert d['selector_region']['unobserved']==1 and not d['selector_unknown_filled']


@pytest.mark.parametrize('kwargs',[
    {'enabled':False},{'present':True},
    {'target':0.,'baseline_reason':'follow_gap_hold'}, {'front_mask':1}, {'tail_extent':.01001},
])
def test_stops_and_live_threat_are_not_changed(monkeypatch,kwargs):
    assert execute(monkeypatch,**kwargs)[0]==0.


def test_confirmed_pass_keeps_same_submillimeter_budget(monkeypatch):
    v,a,d=execute(monkeypatch,retained=False,target=.35)
    assert v==.35 and a==.2
    assert d['selector_reason']=='released_confirmed_rear_fringe'
    assert d['selector_released_rear_sweep']['policy']=='released_low_speed_body_fringe_v2'
    assert d['selector_released_rear_sweep']['maximum_exterior_m']==.001
    assert d['selector_region']['unobserved']==1 and not d['selector_unknown_filled']
