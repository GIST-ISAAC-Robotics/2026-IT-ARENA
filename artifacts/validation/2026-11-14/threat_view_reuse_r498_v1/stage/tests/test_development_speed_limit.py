"""개발 상한이 출발/재ARM을 대신하지 않으며 경기 기본 모드를 바꾸지 않는지 확인."""
from types import SimpleNamespace
import math
import pytest
from arena_autonomy.race_session import RaceSupervisor
from arena_autonomy.race_ops_core import OpsCore
from race_operator import build_command, matching_result, preflight


def setup(enabled=True):
    v=SimpleNamespace(epoch=0,signal=SimpleNamespace(started=False),fault=None)
    r=RaceSupervisor(v,session='session1',now_s=10.)
    if enabled:r.configure_development_speed_limit(2.8,.3)
    r.mcu.update(dict(v=1,boot='mcu1',generation=0,state='DISARMED',now_us=10_000_000,estop_held=False),10.,100.)
    return r


def command(r,**kw):
    return dict(v=1,request='limit1',op='set_speed_limit',session=r.session,
                race_epoch=r.permission.race_epoch,intervention=r.permission.intervention,speed_limit_mps=.2,**kw)


def test_default_rejects_new_operation_and_preserves_output():
    r=setup(False)
    assert r.handle_command(command(r),10.,100.)['reason']=='development_mode_disabled'
    assert r.limit_speed(2.5)==2.5 and r.development_speed_limit is None


def test_setting_is_bound_held_and_does_not_start_or_arm():
    r=setup();before=(r.permission.intervention,r.permission.latch,r.rearm_required,r.permission.started)
    c=command(r);result=r.handle_command(c,10.,100.)
    assert result['accepted'] and matching_result(c,result)
    assert before==(r.permission.intervention,r.permission.latch,r.rearm_required,r.permission.started)
    assert r.limit_speed(2.)==.2 and r.limit_speed(0.)==0. and r.limit_speed(-1.)==-.2
    assert r.limit_speed(.1)==.1
    assert not r.handle_command(c,10.,100.)['accepted']
    assert r.handle_command(dict(v=1,request='limit1',op='stop'),10.,100.)['accepted']
    assert r.development_speed_limit['limit_mps']==.2


@pytest.mark.parametrize('field,value,reason',[('session','old','session_mismatch'),('race_epoch',7,'race_epoch_mismatch'),
 ('intervention',7,'intervention_mismatch'),('speed_limit_mps',True,'speed_limit_out_of_range'),
 ('speed_limit_mps',float('nan'),'speed_limit_out_of_range'),('speed_limit_mps',float('inf'),'speed_limit_out_of_range'),
 ('speed_limit_mps',0.,'speed_limit_out_of_range'),('speed_limit_mps',3.,'speed_limit_out_of_range')])
def test_invalid_or_stale_request_cannot_change_limit(field,value,reason):
    r=setup();c=command(r);c[field]=value
    assert r.handle_command(c,10.,100.)['reason']==reason
    assert r.development_speed_limit['limit_mps']==.3


@pytest.mark.parametrize('case,reason',[('running','stop_required'),('resume','stop_required'),
 ('armed','mcu_stop_required'),('stale','mcu_status_missing'),('missing','mcu_status_missing')])
def test_control_state_rechecked_at_execution(case,reason):
    r=setup();now=10.
    if case=='running':r.permission.latch=None
    if case=='resume':r.resume={'request':'start1'}
    if case=='armed':r.mcu.latest['state']='ACTIVE'
    if case=='stale':now=11.
    if case=='missing':r.mcu.latest=None
    assert r.handle_command(command(r),now,100.)['reason']==reason
    assert r.development_speed_limit['limit_mps']==.3


def test_operator_queue_and_cli_keep_the_value_and_boundaries():
    r=setup();race=r.snapshot(10.,100.);core=OpsCore('car1')
    core.update_status(dict(state='RACE_LATCHED',race=race),100.)
    view={k:race[k] for k in ('session','race_id','race_epoch','intervention')}
    assert core.view(100.,100.)['controls']['set_speed_limit']['allowed']
    assert core.submit('set_speed_limit','ui1',view,'client',None,100.,value=.2)['stage']=='queued'
    assert core.next_action()['value']==.2
    assert core.submit('set_speed_limit','ui2',view,'client',None,100.,value=float('nan'))['adapter_reason']=='speed_limit_out_of_range'
    c=build_command('set_speed_limit',session=r.session,race_epoch=0,intervention=1,speed_limit_mps=.2)
    preflight(c,race)
    core.update_status(dict(state='RUNNING',race=dict(race,latch=None)),100.1)
    assert not core.view(100.1,100.)['controls']['set_speed_limit']['allowed']


@pytest.mark.parametrize('initial',[0.,-.1,3.,float('nan'),True])
def test_invalid_initial_limit_rejected(initial):
    r=setup(False)
    with pytest.raises(ValueError):r.configure_development_speed_limit(2.8,initial)
