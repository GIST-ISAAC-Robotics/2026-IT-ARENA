"""진단 기록 차이를 실제 명령/상태 차이와 혼동하지 않는 저장 비교 계약."""
from copy import deepcopy
import pytest
from probe_grid_deferred_center import rejected_yield_diagnostic_only


def pair():
    old=[.35,0.,{'m3_candidate':'straight','m3_lateral':{'checks':{'straight':{'eligible_for_trial':True}},
                                                     'state':{'clear_fresh':2}}}]
    new=deepcopy(old)
    new[2]['m3_lateral']['checks']['yield']={'eligible_for_trial':False,'reasons':['current_sweep_unobserved']}
    return old,new


def test_added_rejected_check_is_diagnostic():
    old,new=pair()
    original=deepcopy(new)
    assert rejected_yield_diagnostic_only(old,new)
    assert new==original


@pytest.mark.parametrize('change',['speed','steering','selected','state','accepted','missing_eligibility','existing_yield'])
def test_other_changes_cannot_pass_as_diagnostic(change):
    old,new=pair()
    if change=='speed':new[0]+=1e-12
    elif change=='steering':new[1]+=1e-12
    elif change=='selected':new[2]['m3_candidate']='yield'
    elif change=='state':new[2]['m3_lateral']['state']['clear_fresh']=3
    elif change=='accepted':new[2]['m3_lateral']['checks']['yield']['eligible_for_trial']=True
    elif change=='missing_eligibility':del new[2]['m3_lateral']['checks']['yield']['eligible_for_trial']
    else:old[2]['m3_lateral']['checks']['yield']={'eligible_for_trial':False}
    assert not rejected_yield_diagnostic_only(old,new)


def test_equal_output_is_not_a_new_diagnostic():
    old,_=pair()
    assert not rejected_yield_diagnostic_only(old,deepcopy(old))


def test_plan_pass_but_actual_command_rejection_is_diagnostic():
    old,new=pair()
    check=new[2]['m3_lateral']['checks']['yield']
    check.update(eligible_for_trial=True,command={'accepted':False,'reason':'m3_command_unobserved'})
    assert rejected_yield_diagnostic_only(old,new)
    check['command']['accepted']=True
    assert not rejected_yield_diagnostic_only(old,new)


def test_unknown_eligibility_is_not_a_rejected_diagnostic():
    old,new=pair()
    check=new[2]['m3_lateral']['checks']['yield']
    check.update(eligible_for_trial=None,command={'accepted':False})
    assert not rejected_yield_diagnostic_only(old,new)
