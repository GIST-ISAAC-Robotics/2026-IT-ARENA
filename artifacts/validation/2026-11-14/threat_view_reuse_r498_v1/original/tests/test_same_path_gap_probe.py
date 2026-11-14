"""동일 관측/명령 계약의 제한 양성 사례와 범위 밖 반례."""
import copy
import math
import numpy as np
import pytest
from probe_same_path_gap import candidate
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import history,scan


def fixture(extra=None):
    raw=scan(.5,[8.]*500);raw.ranges[375]=math.inf
    for i,v in (extra or {}).items():raw.ranges[i]=v
    scene=motion_scene(raw,history(),.6,timing_verified=True)
    proof=dict(allowed=True,reason='bounded_side_missing',timed_missing_diagnostic=dict(
        reference_s=.6,source_first_s=.5,checked_speed_mps=.35,checked_steering_rad=.2,
        checked_previous_steering_rad=.2,checked_age_s=.1,wheelbase_m=.135,length_m=.2,width_m=.15))
    command=dict(now=.6,speed=0.,target=.35,steering=.2,previous=.2)
    return raw,scene,proof,command


def evaluate(extra=None,proof_update=None,command_update=None,queries=None):
    raw,scene,p,c=fixture(extra)
    if proof_update:p.update(proof_update)
    if command_update:c.update(command_update)
    func,_=candidate()
    return func(scene,raw,np.array([[.06,.08]]) if queries is None else queries,
                np.full(500,.1),.2,.15,p,c)


def test_same_path_small_side_gap_is_risk_not_free_space():
    r=evaluate();assert r['accepted'] and not r['unknown_filled']
    assert r['residual_hidden_object_risk'] and r['maximum_exterior_m'] is None


@pytest.mark.parametrize('change',[dict(steering=.21),dict(previous=.21),dict(target=.36),
                                  dict(speed=.4),dict(now=.61),dict(speed=-.001)])
def test_motion_or_time_change_cannot_borrow_approval(change):
    assert not evaluate(command_update=change)['accepted']


def test_unapproved_partial_cannot_borrow_approval():
    assert not evaluate(proof_update={'allowed':False})['accepted']


def test_outside_original_sweep_cannot_borrow_approval():
    assert not evaluate(queries=np.array([[.06,.5]]))['accepted']


def test_current_close_return_is_preserved():
    assert evaluate(extra={374:.09})['reason']=='known_occupied_padding'


def test_extra_front_gap_stays_rejected():
    assert evaluate(extra={250:math.inf})['reason']=='unknown_outside_side_scope'


def test_wide_side_gap_stays_rejected():
    assert not evaluate(extra={i:math.inf for i in range(373,379)})['accepted']


def test_invalid_return_is_not_a_positive_infinity_exception():
    assert not evaluate(extra={375:float('nan')})['accepted']
