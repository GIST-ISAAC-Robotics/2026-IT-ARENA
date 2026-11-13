"""Older measured points enter structure memory without widening association distance."""
import json
import numpy as np
import pytest
from arena_autonomy.known_structure import structure_policy, classify, MEMORY_LIMIT
from arena_autonomy.partial_observation import PartialObservationPolicy
from test_known_structure_policy import masked
from test_local_follow import history


def run_case(*, speed=.2, steering=0., point=(-.3,.5), full=False, age=.2, tracked=True, flank=True):
    sectors=[[126.,4.]]
    raw=masked(sectors=sectors)
    if not flank:
        _,_,rows=classify(raw,sectors)
        raw.ranges[rows[0]['flanks'][0]]=float('nan')
    valid,_,rows=classify(raw,sectors)
    p=structure_policy(PartialObservationPolicy,json.dumps(sectors))()
    points=np.array([point])
    prior=(points.copy(),.6-age,.3)
    if tracked:p.previous=prior
    # A later scan saw a nearby but different return; >2 cm does not imply identity.
    p.structure_previous=(np.array([[-.34,.5]]),.5)
    if full:
        p.structure_hidden=(np.tile([-.34,.5],(MEMORY_LIMIT,1)),.6)
        p.structure_last_source=.5
    p.structure_active_call=(raw,valid,rows,[],history(),.6,.5,speed,steering,steering,.135,.2,.15)
    report={};original=np.ones((2,1),dtype=bool)
    out=p._refine_threat(original,points,prior,None,None,None,.6,None,None,
        speed,age,steering,steering,.135,.2,.15,report)
    return p,out,original,report,raw


def test_retained_point_itself_is_preserved_and_collision_checked():
    p,out,original,report,raw=run_case()
    assert original.all() and not out.any()
    assert np.any(np.all(p.structure_hidden[0]==[-.3,.5],axis=1))
    assert p.structure_precomputed['retained_previous_shadow_points']==1
    assert report['structure_rear_memory_transfer']['maximum_memory_match_m']==0
    assert np.isnan(raw.ranges).any()


def test_turn_into_transferred_point_is_still_limited():
    p,out,_,_,_=run_case(speed=2.4,steering=.37)
    assert not out.any()
    assert p.structure_precomputed['memory_cap_mps']<2.4


@pytest.mark.parametrize('kwargs',[
    {'point':(.3,.5)}, {'point':(-.3,-.5)}, {'full':True},
    {'age':.31}, {'tracked':False}, {'flank':False}])
def test_unrelated_front_expired_unexplained_or_capacity_case_keeps_threat(kwargs):
    p,out,original,_,_=run_case(**kwargs)
    assert original.all() and out.all()
    assert p.structure_precomputed is None or 'retained_previous_shadow_points' not in p.structure_precomputed
    if kwargs.get('full'):assert len(p.structure_hidden[0])==MEMORY_LIMIT
