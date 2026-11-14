from dataclasses import replace
from unittest.mock import patch

import numpy as np
import pytest

from test_bypass_handoff import context,follow_space,moving_monitor


@pytest.mark.parametrize('forward,extra,old_request,expected',[
    (.25,.16,.9,.45),  # 0.25 + (0.16 m / 0.8 s)
    (.25,-.16,.9,.25),  # 진입 여유 안에서 상대보다 더 빠르게 접근하지 않는다
    (.25,.16,.30,.30),  # G1 경로/점유 상한 준수
    (.25,.16,0.,0.),    # 정상 HOLD를 전진으로 바꾸지 않는다
    (0.,.16,.9,.20),    # 상대가 움직이지 않으면 기존 고정 진입 여유
    (-.25,.16,.9,.20),  # 우리 쪽으로 오는 반환에는 접근분을 더하지 않는다
])
def test_closing_is_an_observed_relative_speed_plus_excess_reserve(forward,extra,old_request,expected):
    ctrl=follow_space()
    ctrl.execution=replace(ctrl.execution,moving_follow_reserve=True,closing_follow_reserve=True)
    ctrl.monitor,_=moving_monitor([forward]*12)
    _,_,sc=context(.6)
    front=max(ctrl.bypass.guide.entry.transitions_m)+extra-ctrl.config.wheelbase_m/2
    cloud=np.c_[np.full(5,front),np.linspace(-.06,.06,5)]
    with patch('arena_autonomy.bypass_handoff.front_patch',return_value=(cloud,'front')),patch(
            'arena_autonomy.bypass_handoff.straight_reference',return_value=({},{})):
        speed,_=ctrl._follow_entry_space(sc,(old_request,0.,{}),0.,old_request)
    assert speed==pytest.approx(expected,abs=.01)


def test_closing_requires_both_pace_evidence_and_registered_entry_reserve():
    with pytest.raises(ValueError,match='moving follow'):
        replace(follow_space().execution,closing_follow_reserve=True)


@pytest.mark.parametrize('normal,expected',[(.8,.5),(.2,.2),(0.,0.)])
def test_recent_motion_only_limits_extra_reserve_and_does_not_release_normal_hold(normal,expected):
    from types import SimpleNamespace
    ctrl=follow_space()
    ctrl.execution=replace(ctrl.execution,moving_follow_reserve=True,closing_follow_reserve=True,recent_forward_reserve=True)
    ctrl.monitor,_=moving_monitor([.85]*12)
    ctrl.monitor.recent=SimpleNamespace(rate=.5)
    _,_,sc=context(.6)
    front=max(ctrl.bypass.guide.entry.transitions_m)-ctrl.config.wheelbase_m/2
    cloud=np.c_[np.full(5,front),np.linspace(-.06,.06,5)]
    with patch('arena_autonomy.bypass_handoff.front_patch',return_value=(cloud,'front')),patch(
            'arena_autonomy.bypass_handoff.straight_reference',return_value=({},{})):
        speed,_=ctrl._follow_entry_space(sc,(normal,0.,{}),0.,normal)
    assert speed==pytest.approx(expected)


def test_recent_credit_requires_the_registered_closing_policy():
    with pytest.raises(ValueError,match='closing follow'):
        replace(follow_space().execution,recent_forward_reserve=True)
