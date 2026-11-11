"""GUARD 감속 재검사: 형식상 목표가 아니라 측정속도를 포함한 같은 보호 조건."""
from dataclasses import replace
import pytest
import arena_autonomy.pass_release_trial as ps
from test_guard_curve_trial import guard

PARTIAL={'allowed':True,'reason':'fully_observed'}


def speed_region(monkeypatch,state,*,works=.15,change=None):
    def region(scene,speed,target,steering,previous,dt,*rest,**kwargs):
        state['regions'].append((speed,target,steering,previous,dt))
        return dict(unobserved=0,masked_left=4,masked_right=0,masked_front_samples=0,
                    masked_tail_extent_m=.009 if max(speed,target)<=works else .012,**(change or {}))
    monkeypatch.setattr(ps,'motion_region',region)


def test_same_complete_guard_rechecked_without_clearing_side_history(guard,monkeypatch):
    ctl,state,call=guard
    ctl.follow_config=replace(ctl.follow_config,traffic_recovery_trial=True)
    speed_region(monkeypatch,state)
    out=call(speed=0.,angle=0.,partial=PARTIAL)
    assert out[0]==.15 and out[2]['selector_choice']=='guard_baseline'
    assert [r[1] for r in state['regions']]==[.35,.25,.15]
    assert all(r[0]==0. and r[2:]==(0.,0.,.02) for r in state['regions'])
    assert ctl.mode=='GUARD' and ctl.sides['left']['present'] and not ctl.sides['left']['passed']
    assert out[2]['selector_guard_speed_recheck']['applied']
    assert not out[2]['selector_unknown_filled']


@pytest.mark.parametrize('speed',[.35,.8])
def test_measured_motion_is_not_replaced_by_lower_target(guard,monkeypatch,speed):
    ctl,state,call=guard
    ctl.follow_config=replace(ctl.follow_config,traffic_recovery_trial=True)
    speed_region(monkeypatch,state)
    out=call(speed=speed,angle=0.,partial=PARTIAL)
    assert out[0]==0. and len(state['regions'])==1
    assert 'selector_guard_speed_recheck' not in out[2]


@pytest.mark.parametrize('partial',[None,{},dict(allowed=False,reason='fully_observed'),
                                    dict(allowed=True,reason='bounded_side_missing')])
def test_only_existing_observed_path_rechecked(guard,monkeypatch,partial):
    ctl,state,call=guard
    ctl.follow_config=replace(ctl.follow_config,traffic_recovery_trial=True)
    speed_region(monkeypatch,state)
    out=call(speed=0.,angle=0.,partial=partial)
    assert out[0]==0. and len(state['regions'])==1


def test_recovery_off_has_identical_original_rejection(guard,monkeypatch):
    _,state,call=guard
    speed_region(monkeypatch,state)
    out=call(speed=0.,angle=0.,partial=PARTIAL)
    assert out[0]==0. and len(state['regions'])==1
    assert 'selector_guard_speed_recheck' not in out[2]


def test_failed_retries_keep_original_hold_and_evidence(guard,monkeypatch):
    ctl,state,call=guard
    ctl.follow_config=replace(ctl.follow_config,traffic_recovery_trial=True)
    speed_region(monkeypatch,state,works=0.)
    out=call(speed=0.,angle=0.,partial=PARTIAL)
    assert out[:2]==(0.,0.) and len(state['regions'])==3
    assert not out[2]['selector_guard_speed_recheck']['applied']
    assert out[2]['selector_guard_candidate']['region']['masked_tail_extent_m']==.012


@pytest.mark.parametrize('key',['unobserved','masked_front_samples'])
def test_unobserved_or_front_mask_is_not_given_speed_permission(guard,monkeypatch,key):
    ctl,state,call=guard
    ctl.follow_config=replace(ctl.follow_config,traffic_recovery_trial=True)
    def region(*args,**kwargs):
        state['regions'].append(args[1:6])
        return dict(unobserved=int(key=='unobserved'),masked_front_samples=int(key=='masked_front_samples'),
                    masked_left=4,masked_right=0,masked_tail_extent_m=.012)
    monkeypatch.setattr(ps,'motion_region',region)
    out=call(speed=0.,angle=0.,partial=PARTIAL)
    assert out[0]==0. and len(state['regions'])==1


def test_guard_toward_side_rejection_is_not_retried(guard,monkeypatch):
    ctl,state,call=guard
    ctl.follow_config=replace(ctl.follow_config,traffic_recovery_trial=True)
    speed_region(monkeypatch,state)
    out=call(speed=0.,angle=.2,partial=PARTIAL)
    assert out[0]==0. and out[2]['selector_reason']=='guard_curve_rejected_toward_left'
    assert not state['regions']
