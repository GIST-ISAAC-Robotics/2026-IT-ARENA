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


def first_absence(execution, heading=0., radius=None, dropout=None, accept=lambda n:True):
    trial=active(execution,-.09)
    trial.clear_fresh=0
    scene,_=timed_scene(.4,-.09,dropout=dropout)
    seen=[]
    def command(name,candidate):
        seen.append(name)
        return {'accepted':accept(name),'reason':None if accept(name) else 'test_rejection'}
    result=trial.step(scene,[],road_path(-.09,heading,radius),.35,padding(scene),command,history=history())
    return result,trial,seen


def test_first_absence_keeps_heading_without_skipping_fresh_release_confirmation():
    on,trial,seen=first_absence(ON)
    off,old,_=first_absence(OFF)
    assert list(on['checks'])[0]=='straight' and on['deferred_center']['ok']
    assert list(off['checks'])[0]=='hold'
    assert trial.clear_fresh==old.clear_fresh==1 and trial.active and old.active
    assert 'center' not in on['checks'] and 'center' not in off['checks']
    assert on['release_gate']==off['release_gate'] and on['threats']==off['threats']
    assert on['selected']=='straight' and seen==['straight']


@pytest.mark.parametrize('heading,radius',[(.03,None),(0.,1.2)])
def test_first_absence_curve_or_heading_keeps_hold_first(heading,radius):
    result,trial,_=first_absence(ON,heading=heading,radius=radius)
    assert not result['deferred_center']['ok']
    assert list(result['checks'])[0]=='hold' and trial.clear_fresh==1


def test_first_absence_failed_command_does_not_skip_hold():
    out,trial,seen=first_absence(ON,accept=lambda n:n!='straight')
    assert seen[0]=='straight' and out['selected']!='straight'
    assert not out['checks']['straight']['command']['accepted']
    assert list(out['checks'])==['straight','hold'] and trial.clear_fresh==1


def test_first_absence_unknown_still_rejects_straight():
    out,trial,_=first_absence(ON,dropout=lambda deg:np.abs(deg)<12.)
    assert out['selected']!='straight'
    assert not out['checks']['straight']['eligible_for_trial']
    assert trial.clear_fresh==1 and trial.active
