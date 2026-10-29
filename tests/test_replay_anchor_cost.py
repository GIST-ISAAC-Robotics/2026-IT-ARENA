"""원시 비용을 보존하고 알려진 진단 위치 두 곳만 재생 비교에서 분리한다."""
import copy
import math
import pytest
from arena_autonomy.follow_trace import comparable_decision, equal_output


def output():
    return [.35,.2,{'reason':'local_follow_corridor_candidate','partial_observation':{
        'allowed':True,'surface_correspondence':{'accepted':True,'processing_ms':3.},
        'anchor_diagnostic':{'matched':True,'released':True,'fit':{
            'processing_ms':8.,'accepted':True,'translation_m':[.01,.02]}}}}]


def test_known_costs_are_ignored_without_mutating_raw_evidence():
    raw=output();before=copy.deepcopy(raw);other=copy.deepcopy(raw)
    other[2]['partial_observation']['surface_correspondence']['processing_ms']=42.
    other[2]['partial_observation']['anchor_diagnostic']['fit']['processing_ms']=14.
    assert equal_output(comparable_decision(raw),comparable_decision(other))
    assert raw==before
    assert other[2]['partial_observation']['anchor_diagnostic']['fit']['processing_ms']==14.


@pytest.mark.parametrize('field',['speed','steer','reason','allowed','released','matched','accepted','translation','other_cost','missing_cost'])
def test_command_geometry_permission_and_other_fields_remain_comparable(field):
    raw=output();other=copy.deepcopy(raw)
    observation=other[2]['partial_observation'];anchor=observation['anchor_diagnostic']
    if field=='speed':other[0]+=.01
    elif field=='steer':other[1]+=.01
    elif field=='reason':other[2]['reason']='hold'
    elif field=='allowed':observation['allowed']=False
    elif field in ('released','matched'):anchor[field]=False
    elif field=='accepted':anchor['fit']['accepted']=False
    elif field=='translation':anchor['fit']['translation_m'][0]+=.001
    elif field=='other_cost':anchor['processing_ms']=1.
    else:del anchor['fit']['processing_ms']
    assert not equal_output(comparable_decision(raw),comparable_decision(other))


@pytest.mark.parametrize('bad',[True,-1.,math.inf,-math.inf,math.nan,'3',None])
def test_invalid_anchor_cost_is_rejected(bad):
    raw=output();raw[2]['partial_observation']['anchor_diagnostic']['fit']['processing_ms']=bad
    with pytest.raises(ValueError,match='invalid surface processing cost'):
        comparable_decision(raw)


@pytest.mark.parametrize('observation',[None,{}, {'anchor_diagnostic':None}, {'anchor_diagnostic':{'released':False}},
    {'anchor_diagnostic':{'fit':None}}])
def test_absent_optional_diagnostics_are_not_invented(observation):
    raw=[0.,0.,{'partial_observation':observation}]
    assert comparable_decision(raw)==raw
