"""실제 _select/_guard_curve 연결과 호출 범위. 기하 입력은 명시 합성 fixture다."""
from copy import deepcopy
import inspect
from types import SimpleNamespace

import pytest
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.bypass_handoff import BypassHandoffFollow
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.race_runtime_policy import guard_crawl as gc, released_plan as rp

@pytest.fixture
def rig(monkeypatch):
    cfg=FollowConfig(partial_observation_policy='bounded_local_v4',parallel_hold_trial=True,
        pass_release_trial=True,guard_curve_trial=True,traffic_recovery_trial=True,
        boundary_evidence_mode='corner_distance_candidate',occupancy_timing_mode='per_ray_candidate')
    state=dict(views=[],targets=[],fail=False)
    scene=SimpleNamespace(t_ref_s=1.,source_first_s=.9,source_epoch=0)
    path=[[-1.,0.],[0.,0.],[1.,0.],[2.,0.]]
    base=(.7,0.,dict(reason='local_follow',behavior='FOLLOW',path_points=path,
        boundary_from_memory=False,path_source_s=.9,partial_observation=dict(allowed=True,reason='fully_observed')))
    parallel=(0.,0.,dict(reason='parallel_hold_reference_not_straight',behavior='HOLD'))
    def view(*args):
        result=dict(frame=ps.mx.ExecutionFrame(path),state=None,
            left=dict(present=True,points=3,rearmost_ds_m=.1,frontmost_ds_m=.2),
            right=dict(present=False,points=0,rearmost_ds_m=None,frontmost_ds_m=None))
        state['views'].append(result);return result
    def region(scene,speed,target,*args,**kwargs):
        if state['fail']:raise RuntimeError('injected guard failure')
        state['targets'].append(target)
        return dict(unobserved=0,queries=100,masked_front_samples=0,masked_left=40,masked_right=0,
            masked_tail_extent_m=.009 if target==.1 else .012)
    monkeypatch.setattr(ps,'threat_view',view);monkeypatch.setattr(ps,'motion_region',region)
    for module in (gc,rp):monkeypatch.setattr(module,'source_valid',lambda _:True)
    def run(policy,*,shadow=False,owner=None,observer=None,speed=0.):
        owner=owner or ps.PassReleaseSelector(cfg)
        with policy.enabled(shadow=shadow,observer=observer):
            result=owner.select(scene,base,parallel,speed,0.,.02,raw_scan=raw)
        return result,deepcopy((owner.sides,owner.guard_anchor,owner.last_source,owner.last_ref,owner.last_choice,owner.epoch))
    raw=object()
    return cfg,scene,base,parallel,raw,state,run


@pytest.mark.parametrize('shadow',[False,True])
def test_real_select_guard_output_state_and_observer_with_one_view(rig,shadow):
    _,_,base,_,_,state,run=rig
    events=[]
    result,snapshot=run(rp,shadow=shadow,observer=lambda *args:events.append(deepcopy(args[3:6])))
    assert len(state['views'])==1 and state['targets']==[.35,.25,.15,.1]
    before,after,_=events[0]
    assert before[0]==0. and after[0]==.1 and result==(before if shadow else after)
    assert after[2]['reason']=='local_follow' and after[2]['behavior']=='FOLLOW'
    assert after[2]['path_points']==base[2]['path_points']
    assert snapshot[0]['left']['seen'] and snapshot[0]['left']['present'] and not snapshot[0]['left']['passed']


def test_new_call_and_owner_never_reuse_previous_view(rig,monkeypatch):
    *_,state,run=rig;received=[];original=gc.assess
    def capture(*args,**kwargs):
        received.append(kwargs['current_view']);return original(*args,**kwargs)
    monkeypatch.setattr(gc,'assess',capture)
    run(rp);run(rp)
    assert len(state['views'])==2 and len(received)==2
    assert received[0] is state['views'][0] and received[1] is state['views'][1]
    assert received[0] is not received[1]


@pytest.mark.parametrize('outer_shadow,inner_shadow',[(False,False),(True,False),(False,True)])
def test_nested_context_restoration_and_observer_semantics(rig,outer_shadow,inner_shadow):
    cfg,scene,base,parallel,raw,state,_=rig
    originals=(ps.PassReleaseSelector._select,ps.PassReleaseSelector._guard_curve,BypassHandoffFollow.command)
    def nested():
        events=[];owner=ps.PassReleaseSelector(cfg)
        with rp.enabled(shadow=outer_shadow,observer=lambda *a:events.append(('outer',deepcopy(a[3:6])))):
            installed=(ps.PassReleaseSelector._select,ps.PassReleaseSelector._guard_curve,BypassHandoffFollow.command)
            with rp.enabled(shadow=inner_shadow,observer=lambda *a:events.append(('inner',deepcopy(a[3:6])))):
                output=owner.select(scene,base,parallel,0.,0.,.02,raw_scan=raw)
            assert (ps.PassReleaseSelector._select,ps.PassReleaseSelector._guard_curve,BypassHandoffFollow.command)==installed
        assert (ps.PassReleaseSelector._select,ps.PassReleaseSelector._guard_curve,BypassHandoffFollow.command)==originals
        return output,events,deepcopy(owner.sides)
    output,events,sides=nested()
    assert output[0]==.1 and len(state['views'])==1
    assert events[0][0]=='outer' and events[0][1][0][0]==0. and events[0][1][1][0]==.1
    assert events[1][0]=='inner' and events[1][1][0][0]==(0. if outer_shadow else .1)
    assert events[1][1][1][0]==.1 and sides['left']['present'] and not sides['left']['passed']


def test_guard_exception_resets_call_frame_and_methods(rig):
    cfg,scene,base,parallel,raw,state,_=rig
    originals=(ps.PassReleaseSelector._select,ps.PassReleaseSelector._guard_curve,BypassHandoffFollow.command)
    with rp.enabled():
        context=inspect.getclosurevars(ps.PassReleaseSelector._guard_curve).nonlocals['current_call']
        state['fail']=True
        with pytest.raises(RuntimeError,match='injected guard'):
            ps.PassReleaseSelector(cfg).select(scene,base,parallel,0.,0.,.02,raw_scan=raw)
        assert context.get() is None
        state['fail']=False
        assert ps.PassReleaseSelector(cfg).select(scene,base,parallel,0.,0.,.02,raw_scan=raw)[0]==.1
        assert context.get() is None and len(state['views'])==2
    assert (ps.PassReleaseSelector._select,ps.PassReleaseSelector._guard_curve,BypassHandoffFollow.command)==originals


def test_observer_exception_restores_all_hooks(rig):
    *_,run=rig
    originals=(ps.PassReleaseSelector._select,ps.PassReleaseSelector._guard_curve,BypassHandoffFollow.command)
    def fail(*args):raise RuntimeError('observer failed')
    with pytest.raises(RuntimeError,match='observer failed'):run(rp,observer=fail)
    assert (ps.PassReleaseSelector._select,ps.PassReleaseSelector._guard_curve,BypassHandoffFollow.command)==originals
    assert rp._MEASURED.get() is None


def test_signed_command_scope_preserves_rejection_and_reset(rig,monkeypatch):
    cfg,scene,base,parallel,raw,state,_=rig
    owner=ps.PassReleaseSelector(cfg)
    def bridge(unused,scene,history,speed,side,previous,dt,*,raw_scan=None):
        return owner.select(scene,base,parallel,abs(speed),previous,dt,raw_scan=raw_scan)
    monkeypatch.setattr(BypassHandoffFollow,'command',bridge)
    with rp.enabled():
        output=BypassHandoffFollow.command(None,scene,None,-.05,'left',0.,.02,raw_scan=raw)
        assert output[0]==0. and state['targets']==[.35,.25,.15] and rp._MEASURED.get() is None
    assert BypassHandoffFollow.command is bridge
    def fail(*args,**kwargs):raise RuntimeError('command failed')
    monkeypatch.setattr(BypassHandoffFollow,'command',fail)
    with rp.enabled():
        with pytest.raises(RuntimeError,match='command failed'):
            BypassHandoffFollow.command(None,scene,None,-.05,'left',0.,.02,raw_scan=raw)
        assert rp._MEASURED.get() is None
    assert BypassHandoffFollow.command is fail


def test_released_branch_keeps_crawl_and_does_not_capture_guard(rig,monkeypatch):
    cfg,scene,base,parallel,raw,state,run=rig
    monkeypatch.setattr(ps,'lateral_excursion',lambda *args:.05)
    def owner():
        item=ps.PassReleaseSelector(cfg)
        item.sides['left'].update(seen=True,present=False,passed=True,clear=10,vanish='benign')
        item.last_source=scene.source_first_s
        return item
    actual,new_state=run(rp,owner=owner())
    assert actual[0]==.1 and actual[2]['selector_reason']=='released_masked_crawl_verified'
    assert new_state[0]['left']['passed'] and not new_state[0]['left']['present']
    assert actual[2]['selector_released_crawl_retry']['applied']
    assert len(state['views'])==1 and state['targets']==[.7,.35,.25,.15,.1]
