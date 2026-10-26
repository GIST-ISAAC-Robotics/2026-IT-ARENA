"""중앙 복귀 보류는 큰 복귀 순서를 명시적으로 바꾸지만 원래 검사를 면제하지 않는다."""
from dataclasses import replace
import numpy as np
import pytest
import arena_autonomy.m3_execution_trial as mx
from test_m3_small_center_straight import active,step_order,road_path,ON as SMALL
from test_m3_parallel_straight import trial as threat_trial
from test_m3_observed_follow import timed_scene
from test_m3_execution_trial import padding
from test_local_follow import history

ON=replace(SMALL,straight_surface_commitment=True,defer_center_on_straight=True)
OFF=replace(ON,defer_center_on_straight=False)


def test_explicit_typed_option_and_prerequisites():
    assert mx.ExecutionConfig().defer_center_on_straight is False
    with pytest.raises(ValueError):replace(ON,defer_center_on_straight=1)
    for key in ('straight_start','command_fallback','release_to_baseline'):
        with pytest.raises(ValueError):replace(ON,**{key:False})


@pytest.mark.parametrize('offset',[-.09,.09])
def test_large_offset_can_keep_heading_only_after_actual_plan_and_command_checks(offset):
    on,seen,_=step_order(ON,offset)
    off,_,_=step_order(OFF,offset)
    assert list(off['checks'])[0]=='center'
    assert list(on['checks'])[0]=='straight'
    assert on['deferred_center']['ok']
    if on['selected']=='straight':
        assert seen[0]=='straight' and on['checks']['straight']['eligible_for_trial']
        assert on['checks']['straight']['command']['accepted']
    assert on['threats']==off['threats'] and on['release_gate']==off['release_gate']


def test_rejected_straight_command_keeps_center_hold_order():
    out,seen,_=step_order(ON,-.09,accept=lambda n:n!='straight')
    old,_,_=step_order(OFF,-.09)
    assert seen[0]=='straight' and not out['checks']['straight']['command']['accepted']
    assert list(out['checks'])[1:]==[k for k in old['checks'] if k!='straight']
    assert out['selected']!='straight'


@pytest.mark.parametrize('heading,radius',[(.03,None),(0.,1.2),(0.,4.)])
def test_curve_or_heading_error_does_not_prefer_straight(heading,radius):
    trial=active(ON,-.09);scene,_=timed_scene(.4,-.09)
    result=trial.step(scene,[],road_path(-.09,heading,radius),.35,padding(scene),
        lambda *a:{'accepted':True,'reason':None},history=history())
    assert not result['deferred_center']['ok']
    assert list(result['checks'])[0]=='center'


@pytest.mark.parametrize('offset,dropout',[(-.09,lambda deg:np.abs(deg)<12.),(.30,None)])
def test_unknown_or_disallowed_absolute_offset_is_not_excused(offset,dropout):
    trial=active(ON,offset);scene,path=timed_scene(.4,offset,dropout=dropout)
    result=trial.step(scene,[],path,.35,padding(scene),lambda *a:{'accepted':True,'reason':None},history=history())
    assert result['selected']!='straight'
    assert not result['checks']['straight']['eligible_for_trial']


def test_present_threat_order_and_records_remain_unchanged():
    a,seen_a,first=threat_trial(ON)
    b,seen_b,second=threat_trial(OFF)
    assert 'deferred_center' not in a
    assert a['threats']==b['threats'] and a['release_gate']==b['release_gate']
    assert list(a['checks'])==list(b['checks']) and seen_a==seen_b
    assert first.records.keys()==second.records.keys()


def test_confirmed_baseline_release_is_not_replaced_by_lane_retention():
    trial=active(ON,-.09);scene,path=timed_scene(.4,-.09)
    trial._release=lambda *a:(True,{'release':True,'reason':'test_confirmed'})
    result=trial.step(scene,[],path,.35,padding(scene),history=history())
    assert result['selected']=='baseline' and not trial.active
    assert 'deferred_center' not in result
