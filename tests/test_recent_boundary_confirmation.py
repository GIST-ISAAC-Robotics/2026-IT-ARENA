"""만료 경로는 현재 관측의 비교용일 뿐, 주행 가능한 기억이 아니다."""
from dataclasses import replace

import pytest

from test_junction_priority import make_controller
from test_local_follow import scene,history


def prepared(on=True):
    controller,h=make_controller(on)
    # 최근 승인 경로는 있으나 오래된 초기 확인 후보는 사용할 수 없는 상태.
    controller.pending=None
    controller.pending_count=0
    controller.pending_ref=None
    assert controller.path_source==pytest.approx(.7)
    return controller,h


def test_first_expired_tick_uses_matching_current_observation_without_an_extra_hold():
    controller,h=prepared()
    speed,_,detail=controller.command(scene(1.02),h,.3,'left')
    assert speed>0 and detail['reason']=='local_follow'
    assert detail['recent_boundary_confirmation']=='current_observation_after_memory_expiry'
    assert detail['path_source_s']==pytest.approx(.92)
    assert not detail['boundary_from_memory']
    assert controller.recent_path is None


def test_disabled_policy_still_requires_new_distinct_confirmation():
    controller,h=prepared(False)
    assert controller.command(scene(1.02),h,.3,'left')[0]==0


def test_too_old_reference_does_not_confirm_current_first_observation():
    controller,h=prepared()
    assert controller.command(scene(1.32),h,.3,'left')[0]==0


def test_changed_current_geometry_is_not_confirmed_by_old_path():
    controller,h=prepared()
    controller.path[:,1]+=.2
    assert controller.command(scene(1.02),h,.3,'left')[0]==0


def test_current_close_obstacle_still_stops():
    controller,h=prepared()
    result=controller.command(scene(1.02,obstacle_x=.10),h,.3,'left')
    assert result[0]==0 and result[2]['behavior']=='HOLD'


@pytest.mark.parametrize('kind',['stale_scan','empty','new_epoch'])
def test_missing_or_changed_current_input_cannot_use_expired_path(kind):
    controller,h=prepared()
    current=scene(1.02,source=.7) if kind=='stale_scan' else scene(1.02,epoch=int(kind=='new_epoch'))
    if kind=='empty':current=replace(current,source_first_s=None,source_last_s=None,rays=())
    assert controller.command(current,h,.3,'left')[0]==0


def test_excessive_motion_still_rejects_geometry_reference():
    controller,_=prepared()
    assert controller.command(scene(1.02),history(5.),.3,'left')[0]==0
