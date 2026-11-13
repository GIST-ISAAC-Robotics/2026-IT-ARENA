"""HOLD 출력의 상태 누락과 실제 입력 고장/현재 경로 거절을 구별한다."""
from dataclasses import replace
import pytest
from test_observed_rejoin import subject,call


def held_subject(enabled=True):
    follow,detail=subject()
    follow.execution=replace(follow.execution,bypass=replace(follow.execution.bypass,rear_wall_witness=enabled))
    follow.bypass.guide.phase='rejoin'
    follow.bypass.command=lambda *a,**k:(0.,0.,dict(reason='entry_guide_current_wall_too_short'))
    return follow,detail


def test_current_g1_can_take_over_after_hold_omits_actual_rejoin_phase():
    follow,_=held_subject()
    for t in (.5,.5):
        out=call(follow,t)
        assert follow.mode=='REACQUIRE' and out[2]['observed_rejoin_ready']
        assert out[0]==0.  # 같은 스캔 반복은 두 번째 확인이 아니다.
    out=call(follow,.6)
    assert follow.mode=='FOLLOW' and out[:2]==(.35,.02)


@pytest.mark.parametrize('changed',['off','reset','path','gap','side','g1_stop'])
def test_state_only_does_not_supply_current_permission(changed):
    follow,detail=held_subject(enabled=changed!='off')
    if changed=='reset':follow.bypass.guide.reset()
    elif changed=='path':detail['path_points']=[]
    elif changed=='gap':detail['swept_clearance_m']=0.
    elif changed=='side':detail['selector_sides']['right']['present']=True
    elif changed=='g1_stop':follow.normal.command=lambda *a,**k:(0.,.02,detail)
    call(follow,.5);out=call(follow,.6)
    assert follow.mode=='BYPASS' and out[0]==0.
