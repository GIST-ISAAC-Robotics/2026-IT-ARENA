"""관측 창·부호 있는 예약식·알려진 부분 소실 반례를 구분해 확인한다."""
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import pytest

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from candidate import FrontMotionCredit,MotionCreditCandidate
from arena_autonomy.lidar_motion import MotionHistory,MotionConfig
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig


def history():
    h=MotionHistory(MotionConfig(history_s=5.))
    for t in np.arange(0.,4.,.01):
        h.add_mean_wheel(float(t),0.);h.add_gyro(float(t),0.)
    return h


def cloud(x,y=0.):
    return np.array([[x,-.12+y],[x,-.11+y],[x,-.10+y]])


def observe(observer,h,t,points,epoch=0):
    sc=SimpleNamespace(t_ref_s=t,source_first_s=round(t-.1,9),source_frame_id='laser_frame',source_epoch=epoch)
    with patch('candidate.source_valid',return_value=True),patch('candidate.front_patch',return_value=(points,'fixture')):
        return observer.update(sc,h,policy='sparse_returns_v1',cap=.35)


@pytest.mark.parametrize('speed,expected',[(0.,0.),(.25,.1),(-.25,0.),(2.,.35)])
def test_longitudinal_motion_and_cap(speed,expected):
    o=FrontMotionCredit();h=history()
    for k in range(10):r=observe(o,h,.5+.1*k,cloud(1.+speed*.1*k))
    assert r['credit_mps']==pytest.approx(expected)


def test_lateral_motion_and_bounded_range_noise_do_not_grant_forward_credit():
    o=FrontMotionCredit();h=history()
    for k in range(12):
        r=observe(o,h,.5+.1*k,cloud(1.+(.03 if k%2 else -.03),.02*k))
        assert r['credit_mps']==0.


def test_fresh_sources_required_and_duplicates_do_not_extend_history():
    o=FrontMotionCredit();h=history()
    for k in range(6):
        r=observe(o,h,.5+.1*k,cloud(1.+.025*k))
        assert r['credit_mps']==0.
    length=len(o.rows)
    for _ in range(5):observe(o,h,1.,cloud(1.125))
    assert len(o.rows)==length
    assert observe(o,h,1.1,cloud(1.15))['credit_mps']>0.


@pytest.mark.parametrize('reset_kind',['epoch','gap','backward','missing','invalid'])
def test_discontinuity_discards_credit(reset_kind):
    o=FrontMotionCredit();h=history()
    for k in range(10):observe(o,h,.5+.1*k,cloud(1.+.025*k))
    assert o.result['credit_mps']>0
    if reset_kind=='epoch':r=observe(o,h,1.5,cloud(1.25),epoch=1)
    elif reset_kind=='gap':r=observe(o,h,1.8,cloud(1.25))
    elif reset_kind=='backward':r=observe(o,h,1.2,cloud(1.25))
    elif reset_kind=='missing':r=observe(o,h,1.5,None)
    else:
        sc=SimpleNamespace(t_ref_s=1.8,source_first_s=1.4)
        with patch('candidate.source_valid',return_value=False):r=o.update(sc,h,policy='sparse_returns_v1',cap=.35)
    assert r['credit_mps']==0. and len(o.rows)<=1


def test_occlusion_can_still_mimic_forward_motion_documented_limit():
    o=FrontMotionCredit();h=history()
    # 같은 정지차의 앞/뒤 지지점 중 앞부분만 사라지면 전방 분위값도 움직인다.
    for k in range(9):observe(o,h,.5+.1*k,np.vstack((cloud(1.),cloud(1.16))))
    r=observe(o,h,1.4,cloud(1.16))
    assert 0.<r['credit_mps']<=.35
    assert r['identity_or_stationarity_proven'] is False


@pytest.mark.parametrize('distance',[.5,.91,.92,1.,1.2,1.8])
@pytest.mark.parametrize('credit',[0.,.1,.35])
@pytest.mark.parametrize('target',[0.,.2,1.])
def test_signed_reserve_never_advances_inside_credit_floor(distance,credit,target):
    h=MotionCreditCandidate(execution=HandoffExecutionConfig(preparation_policy='observed_entry_space_v1',
        persistence_window_policy='trim_before_compare_v1',follow_entry_space_enabled=True),allow_simulation_trial=True)
    old_cap=max(0.,(distance-1.2)/.8)
    evidence=dict(reason='observed_entry_distance_reserve',rear_axle_to_patch_m=distance,
        reserve_m=1.2,approach_window_s=.8,cap_mps=old_cap,applied=old_cap<target)
    with patch.object(BypassHandoffFollow,'_follow_entry_space',return_value=(min(target,old_cap),evidence)),patch.object(
            h.motion_credit,'update',return_value=dict(credit_mps=credit)):
        value,detail=h._follow_entry_space(SimpleNamespace(),(target,0.,{}),0.,target)
    assert value==pytest.approx(min(target,max(0.,(distance-1.2)/.8+credit)))
    assert 0<=value<=target
    if distance<1.2-.8*credit-1e-9:assert value==0.
    if credit==0:assert value==min(target,old_cap)
