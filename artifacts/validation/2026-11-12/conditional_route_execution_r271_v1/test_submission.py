import copy
import pytest
from submission_policy import assess
from scheduled_route import percentile


def arguments():
    return dict(now_s=10.,now_wall_s=100.,plan_submitted_wall_s=99.95,plan_budget_s=.18,
        current_budget_s=.035,last_application_s=9.90,last_command_reference_s=9.86,
        current_reference_s=10.,source_first_s=9.90,path_origin_s=9.72)


def test_recheck_can_finish_before_plan_and_old_command_deadline_is_at_risk():
    a=arguments();before=copy.deepcopy(a);r=assess(**a)
    assert r['offer'] and r['reason']=='useful_before_new_plan'
    assert a==before and r['original_command_deadline_s']==pytest.approx(10.06)


def test_soon_finishing_plan_does_not_get_a_wasted_competing_check():
    a=arguments();a['plan_budget_s']=.07
    assert assess(**a)['reason']=='new_plan_will_supersede'


def test_apply_interval_is_included_in_supersession_prediction():
    a=arguments();a['last_application_s']=9.99;a['plan_budget_s']=.15
    assert assess(**a)['reason']=='new_plan_will_supersede'


def test_normal_plan_before_deadline_needs_no_fallback():
    a=arguments();a['last_command_reference_s']=9.99
    assert assess(**a)['reason']=='new_plan_before_command_deadline'


@pytest.mark.parametrize('field,value',[('source_first_s',9.70),('current_reference_s',9.79),('path_origin_s',9.50)])
def test_predictably_expired_current_or_geometry_is_not_submitted(field,value):
    a=arguments();a[field]=value
    if field=='current_reference_s':a['source_first_s']=9.75
    assert assess(**a)['reason']=='cannot_apply_within_existing_budget'


@pytest.mark.parametrize('field,value',[('now_s',float('nan')),('last_command_reference_s',-float('inf'))])
def test_uninitialized_or_invalid_cost_state_never_proposes_fallback(field,value):
    a=arguments();a[field]=value;assert not assess(**a)['offer']


def test_monotonic_reversal_does_not_enable_fallback():
    a=arguments();a['now_wall_s']=99.
    assert assess(**a)['reason']=='scheduling_clock_boundary'


def test_completion_prediction_never_grants_a_drive_command():
    r=assess(**arguments())
    assert 'speed_mps' not in r and 'steering_rad' not in r
    assert percentile([.1,.2,.3,.4],.25)==pytest.approx(.175)
