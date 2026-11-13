"""현재 런타임의 통과 확인 완료 연결에서 기존 저속/점유/정지 범위를 지키는지 확인한다."""
import importlib.util,math
from pathlib import Path
import pytest
import arena_autonomy.pass_release_trial as ps
import test_released_rear_sweep as linked

def execute(monkeypatch,**kw):
    return linked.execute(monkeypatch,retained=False,**kw)

@pytest.mark.parametrize('target',[.2,.35,.85])
def test_fully_confirmed_pass_uses_existing_rear_sweep_without_raising_g1(monkeypatch,target):
    owner,out=execute(monkeypatch,target=target)
    assert owner.mode=='RELEASED' and out[0]==min(target,.35)
    assert out[2]['selector_reason']=='released_confirmed_rear_fringe'
    assert out[2]['selector_region']['unobserved']==1
    assert out[2]['selector_released_rear_sweep']['residual_hidden_object_risk']
    assert not out[2]['selector_confirmed_rear_recheck']['unknown_filled']
    assert owner.last_choice=='baseline' and not out[2]['parallel_hold_selected']

@pytest.mark.parametrize('kwargs',[
    {'enabled':False},{'proof':False},{'present':True},{'speed':.351},
    {'target':0.,'baseline_reason':'follow_gap_hold'},
    {'target':0.,'no_path':True,'baseline_reason':'follow_no_path'},
    {'front_mask':1},{'tail_extent':.01001},{'neighbor':.19},{'neighbor':.4},
    {'value':math.nan},{'value':-math.inf},{'value':.02},{'mismatch':True},
    {'missing':tuple(range(449,455))},
])
def test_current_threat_fast_motion_input_and_front_guard_stay_blocked(monkeypatch,kwargs):
    assert execute(monkeypatch,**kwargs)[1][0]==0.

def test_original_retained_path_still_uses_original_branch(monkeypatch):
    _,out=linked.execute(monkeypatch,retained=True)
    assert out[0]==.35 and out[2]['selector_reason']=='released_bounded_rear_sweep'
    assert 'selector_confirmed_rear_recheck' not in out[2]
