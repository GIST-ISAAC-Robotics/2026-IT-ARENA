import sys
from types import SimpleNamespace
import pytest
from dataclasses import replace
from arena_autonomy.pass_release_trial import PassReleaseSelector
from test_pass_release_trial import config,output,view

def make_setup(monkeypatch):
    cls=PassReleaseSelector;module=sys.modules[cls.__module__]
    state={'view':view(True,.25),'free':True,'region':dict(unobserved=1,masked_left=0,masked_right=0)}
    monkeypatch.setattr(module,'side_extent',lambda *a:0.)
    monkeypatch.setattr(module,'threat_view',lambda *a:state['view'])
    monkeypatch.setattr(module,'band_check',lambda *a:dict(free=state['free']))
    monkeypatch.setattr(module,'motion_region',lambda *a,**k:state['region'])
    monkeypatch.setattr(module,'same_path_risk',lambda *a:dict(accepted=False))
    ctl=cls(replace(config(),parallel_hold_trial=True,pass_release_trial=True,guard_curve_trial=True,traffic_recovery_trial=True))
    def call(t,base=None):
        scene=SimpleNamespace(source_first_s=t-.1,t_ref_s=t,source_epoch=0)
        return ctl.select(scene,output(.35) if base is None else base,output(0.),.2,0.,.02)
    call(1.);state['view']=view();call(1.1);call(1.2)
    assert ctl.mode=='RELEASED'
    return ctl,state,call

def test_retained_pass_is_not_permission_to_enter_unknown(monkeypatch):
    ctl,state,call=make_setup(monkeypatch);state['free']=False
    result=call(1.3)
    assert ctl.sides['left']['passed'] and result[0]==0
    assert result[2]['selector_reason']=='large_motion_region_unobserved'

def test_new_visible_side_object_cancels_old_pass(monkeypatch):
    ctl,state,call=make_setup(monkeypatch);state['view']=view(True,-.1)
    assert call(1.3)[0]==0 and ctl.mode=='GUARD'

def test_front_stop_remains_zero_with_old_pass(monkeypatch):
    _,state,call=make_setup(monkeypatch);state['free']=False
    assert call(1.3,output(0.,reason='follow_gap_hold'))[0]==0

def test_no_path_cancels_pass_instead_of_keeping_memory_forever(monkeypatch):
    ctl,state,call=make_setup(monkeypatch);state['view']=None
    assert call(1.3,output(0.,reason='follow_no_path'))[0]==0
    assert not ctl.sides['left']['passed']
