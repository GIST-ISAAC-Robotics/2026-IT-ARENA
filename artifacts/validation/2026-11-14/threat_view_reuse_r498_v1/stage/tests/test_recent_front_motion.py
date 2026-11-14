"""반환 이동과 정지 누적 분리를 검증한다. 실제 C1 잡음 분포 시험은 아니다."""
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import pytest
from arena_autonomy.lidar_motion import MotionConfig,MotionHistory
from dataclasses import replace
from arena_autonomy.bypass_handoff import BlockingPersistence, HandoffExecutionConfig


def run(speeds,*,noise=None):
    hist=MotionHistory(MotionConfig(wheel_radius_m=.0325,history_s=3.))
    for t in np.arange(0,3.,.01):
        hist.add_mean_wheel(float(t),.35/.0325);hist.add_gyro(float(t),0.)
    config=HandoffExecutionConfig(preparation_policy='observed_entry_space_v1',persistence_window_policy='trim_before_compare_v1',follow_entry_space_enabled=True,moving_follow_reserve=True,closing_follow_reserve=True,recent_forward_reserve=True)
    monitor=BlockingPersistence(config);x=1.;rows=[]
    for i,vel in enumerate(speeds):
        t=.5+i*.1
        if i:x+=(vel-.35)*.1
        offset=0. if noise is None else noise[i]
        pts=np.c_[np.full(7,x+offset),np.linspace(-.18,-.1,7)]
        sc=SimpleNamespace(t_ref_s=t,source_first_s=t-.1,source_epoch=0,source_frame_id='laser_frame')
        with patch('arena_autonomy.bypass_handoff.source_valid',return_value=True),patch(
            'arena_autonomy.bypass_handoff.front_patch',return_value=(pts,'face')):
            r=monitor.update(sc,hist)
        rows.append((monitor.forward_rate_mps,monitor.recent.rate,r['ready']))
    return monitor,rows


def test_fast_front_has_conservative_credit_without_stationary_handoff():
    monitor,rows=run([.85]*12)
    assert all(old==0 for old,_,_ in rows)
    assert all(new==pytest.approx(.5) for _,new,_ in rows[2:])
    assert not any(ready for _,_,ready in rows)
    # 정지하면 새 스캔에서 추가 전진율을 거둔다. 정지 누적의 허가는 그대로 늦게 생긴다.
    monitor,rows=run([.85]*6+[0.]*12)
    assert all(new==0 for _,new,_ in rows[6:])
    assert rows[-1][2]


@pytest.mark.parametrize('noise',[
    [0.,.03,-.03]*6,[-.03,0.,.03]*6,[.03,0.,-.03]*6])
def test_stationary_face_noise_does_not_supply_forward_credit(noise):
    _,rows=run([0.]*len(noise),noise=noise)
    assert max(new for _,new,_ in rows)<1e-12


def test_backwards_or_a_single_jump_is_not_forward_pace():
    _,rows=run([-.85]*12)
    assert all(new==0 for _,new,_ in rows)
    _,rows=run([0.,0.,3.5,0.,0.,0.,0.])
    assert all(new==0 for _,new,_ in rows)


def test_unknown_and_epoch_change_discard_recent_points():
    monitor,_=run([.85]*6)
    assert monitor.recent.rate>0
    sc=SimpleNamespace(t_ref_s=1.1,source_first_s=1.,source_epoch=1,source_frame_id='laser_frame')
    cloud=np.c_[np.full(7,1.3),np.linspace(-.18,-.1,7)]
    with patch('arena_autonomy.bypass_handoff.source_valid',return_value=True),patch(
            'arena_autonomy.bypass_handoff.front_patch',return_value=(cloud,'face')):
        monitor.update(sc,None)  # epoch이 달라 과거 점을 이송하려고 해서는 안 된다.
    assert len(monitor.recent.rows)==1 and monitor.recent.rate==0
    sc.t_ref_s=1.2;sc.source_first_s=1.1
    with patch('arena_autonomy.bypass_handoff.source_valid',return_value=True),patch(
            'arena_autonomy.bypass_handoff.front_patch',return_value=(None,'no_patch')):
        monitor.update(sc,None)
    assert not monitor.recent.rows and monitor.recent.rate==0
