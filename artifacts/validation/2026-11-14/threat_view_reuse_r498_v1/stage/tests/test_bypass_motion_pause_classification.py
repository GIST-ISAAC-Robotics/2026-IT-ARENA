from copy import deepcopy
from types import SimpleNamespace as N
import pytest
from arena_autonomy.bypass_motion_pause_trial import signed_bypass_pause


def example():
    owner=N(mode='BYPASS',execution=N(entry_policy='corridor_local_v1'),config=N(max_speed_mps=1.),bypass=N(guide=N(path=[1])))
    normal=(0.,.01,dict(reason='pass_release_reverse_motion_hold',selector_choice='hold_fault',
        selector_reason='reverse_motion_hold',reverse_motion=True,measured_speed_mps=-.15,input_error=None))
    return owner,normal


def test_only_classification_changes_and_normal_result_is_preserved():
    owner,normal=example();saved=deepcopy(normal)
    assert signed_bypass_pause(owner,normal,-.15)
    assert normal==saved
    owner.mode='REACQUIRE';assert signed_bypass_pause(owner,normal,-.15)


@pytest.mark.parametrize('key,value',[('reason','input_fault'),('selector_reason','source_reset'),
    ('reverse_motion',False),('input_error','motion history unavailable'),('measured_speed_mps',-.2),
    ('measured_speed_mps',float('nan')),('selector_choice','baseline')])
def test_other_fault_or_inconsistent_measurement_is_not_exempted(key,value):
    owner,normal=example();normal[2][key]=value
    assert not signed_bypass_pause(owner,normal,-.15)


@pytest.mark.parametrize('speed',[0.,.15,-1.1,float('nan'),float('inf'),True])
def test_not_a_finite_in_range_reverse_pause(speed):
    owner,normal=example();normal[2]['measured_speed_mps']=speed
    assert not signed_bypass_pause(owner,normal,speed)


def test_follow_other_policy_no_path_and_positive_output_remain_rejected():
    owner,normal=example();owner.mode='FOLLOW';assert not signed_bypass_pause(owner,normal,-.15)
    owner,normal=example();owner.execution.entry_policy='registered_v0';assert not signed_bypass_pause(owner,normal,-.15)
    owner,normal=example();owner.bypass.guide.path=None;assert not signed_bypass_pause(owner,normal,-.15)
    owner,normal=example();assert not signed_bypass_pause(owner,(.1,*normal[1:]),-.15)
