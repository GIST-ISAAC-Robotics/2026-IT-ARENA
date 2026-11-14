"""유효한 전진 관측은 양보를 연장하되 고정 면·잡음·고장·정지는 연장하지 않는다."""
from dataclasses import replace
import json

import pytest

from arena_autonomy.bypass_handoff import BypassHandoffFollow
from arena_autonomy.bypass_yield_progress import ProgressRelativeYield, ProgressRelativeYieldCommand
from arena_autonomy.bypass_relative_yield import BoundedRelativeYield
from test_bypass_relative_yield import row, subject
from test_bypass_relative_yield_pause import hold, scene
import g1_handoff_eval


def observed(i, shift=0.):
    r = row(i)
    r.update(along_low=.1+shift, along_high=.3+shift)
    return r


def active():
    observer = ProgressRelativeYield()
    for i in range(9):
        observer.update(row(i))
    assert observer.detail['suggest_yield']
    return observer


def test_old_policy_and_default_preserved():
    old=subject()
    assert type(old._relative_yield.observer) is BoundedRelativeYield
    new=BypassHandoffFollow(execution=replace(old.execution,
        relative_yield_policy='observed_progress_yield_v3'),allow_simulation_trial=True)
    assert isinstance(new._relative_yield,ProgressRelativeYieldCommand)
    assert subject(False)._relative_yield is None


def test_continuous_advancing_face_outlasts_original_budget_without_motion_permission():
    o=active()
    for i in range(9,83):
        result=o.update(observed(i,.01*(i-8)))
        assert result['suggest_yield']
        assert not result['control_authorized'] and not result['unknown_filled']
    assert result['total_yield_s']>7. and result['progress_budget_renewals']>=6
    assert result['budget_age_s']<1.3
    r=observed(83,.75);r['phase']='rejoin'
    assert not o.update(r)['suggest_yield']


@pytest.mark.parametrize('kind',['static','noise','backward'])
def test_no_extension_without_forward_progress(kind):
    o=active()
    for i in range(9,57):
        shift=0. if kind=='static' else (.03*(-1)**i if kind=='noise' else -.001*(i-8))
        assert o.update(observed(i,shift))['suggest_yield']
        assert o.renewals==0
    assert not o.update(observed(57))['suggest_yield']
    assert o.detail['reason']=='continuation_budget_expired'


def test_duplicate_source_cannot_renew_and_progress_stops_renewing_when_face_stops():
    o=active()
    o.update(observed(9,.10))
    n=o.renewals;since=o.yield_since
    # 같은 스캔의 좌표만 바뀌어도 새 갱신 근거로 쓰지 않는다.
    assert o.update(observed(9,.20))['suggest_yield']
    assert o.renewals==n and o.yield_since==since
    for i in range(10,58):
        assert o.update(observed(i,.10))['suggest_yield']
    assert not o.update(observed(58,.10))['suggest_yield']


@pytest.mark.parametrize('change',[
    {'epoch':2},{'normal':.6},{'source':5.,'now':5.1},
    {'wall_confirmations':0},{'heading':.3},{'speed':-.1},{'method':'other'},
])
def test_expansion_does_not_override_current_evidence_checks(change):
    o=active();r=observed(9,.11);r.update(change)
    assert not o.update(r)['suggest_yield']
    assert o.yield_since is None and o.progress_mark is None


def test_motion_pause_keeps_one_second_limit_and_never_renews_budget():
    owner=subject();helper=ProgressRelativeYieldCommand()
    helper.observer=active();owner._relative_yield=helper
    import numpy as np
    owner.mode='BYPASS';owner.bypass.guide.path=np.array([[0.,0.],[1.,0.]])
    since=helper.observer.yield_since
    for i in range(11):
        assert hold(owner,helper,2.+.1*i)['motion_pause_intent_retained']
    assert helper.observer.yield_since==since
    assert not hold(owner,helper,3.1)['motion_pause_intent_retained']
    helper.reset();assert helper.observer.yield_since is None


def test_selected_flag_must_match_recorded_policy(tmp_path):
    p=tmp_path/'header.jsonl'
    p.write_text(json.dumps(dict(kind='header',data=dict(follow_config={},
        execution_config=dict(relative_yield_policy='observed_progress_yield_v3')))))
    assert g1_handoff_eval.inspect_header(p,'paced',progress=True)['checks']['policy']
    assert not g1_handoff_eval.inspect_header(p,'paced')['checks']['policy']
    with pytest.raises(ValueError):
        g1_handoff_eval.launch_arguments('g1',None,{},True,progress=True)
