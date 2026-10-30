"""후보의 시각·현재 관측 경계 검사."""
from types import SimpleNamespace as NS
import numpy as np
import pytest
import candidate as c
from arena_autonomy.bypass_handoff import handoff_baseline


@pytest.fixture
def setup(monkeypatch):
    memory=c.CurrentPairReference();config=handoff_baseline()
    scene=NS(t_ref_s=10.14,source_first_s=10.,source_frame_id='laser_frame',source_epoch=0)
    original=(0.,0.,-.15,1.5)
    walls={side:dict(slope=0.,intercept_m=0.,observed_x_m=[-.15,1.5]) for side in ('left','right')}
    monkeypatch.setattr(c,'source_valid',lambda _:True)
    monkeypatch.setattr(c,'ORIGINAL_REFERENCE',lambda *_:(original,dict(walls=walls)))
    monkeypatch.setattr(c,'transport_points',lambda points,*_:points)
    memory.resolve(scene,config,None)
    monkeypatch.setattr(c,'ORIGINAL_REFERENCE',lambda *_:(None,dict(reason='straight_entry_only')))
    monkeypatch.setattr(c,'tracked_straight_wall',lambda *args:(original,walls[args[2]]))
    scene.t_ref_s+=.1;scene.source_first_s+=.1
    return memory,scene,config


def test_current_pair_retains_original_coverage(setup):
    memory,scene,config=setup
    fit,detail=memory.resolve(scene,config,None)
    assert fit==(0.,0.,-.15,1.5)
    assert detail['all_returns_remain_occupied'] and not detail['remembered_line_is_current_observation']


@pytest.mark.parametrize('change',[dict(source_epoch=1),dict(source_frame_id='other'),
    dict(source_first_s=9.9),dict(t_ref_s=10.1),dict(t_ref_s=10.54,source_first_s=10.4),
    dict(t_ref_s=10.6)])
def test_changed_source_time_gap_or_stale_cannot_reuse(setup,change):
    memory,scene,config=setup
    for key,value in change.items():setattr(scene,key,value)
    assert memory.resolve(scene,config,None)[0] is None


def test_expired_seed_needs_fresh_two_wall_source(setup):
    memory,scene,config=setup
    for anchor in memory.anchors.values():anchor['time']-=1.
    assert memory.resolve(scene,config,None)[0] is None


def test_reused_scan_cannot_extend_seed(setup):
    memory,scene,config=setup
    assert memory.resolve(scene,config,None)[0] is not None
    stamp=memory.anchors['left']['time'];scene.t_ref_s+=.02
    assert memory.resolve(scene,config,None)[0] is not None
    assert memory.anchors['left']['time']==stamp


@pytest.mark.parametrize('fit',[None,(0.,0.,-.05,1.5),(0.,0.,-.15,.24),(0.,.5,-.15,1.5)])
def test_missing_short_or_disagreeing_current_wall_cannot_pass(setup,monkeypatch,fit):
    memory,scene,config=setup
    monkeypatch.setattr(c,'tracked_straight_wall',lambda scene,config,side,line:
        (fit if side=='left' else (0.,0.,-.15,1.5),dict(reason='test_current_fit')))
    assert memory.resolve(scene,config,None)[0] is None


def test_original_valid_reference_is_unchanged(setup,monkeypatch):
    memory,scene,config=setup
    value=((.02,.01,-.15,1.3),dict(walls={side:dict(slope=.02,intercept_m=.01,
        observed_x_m=[-.15,1.3]) for side in ('left','right')}))
    monkeypatch.setattr(c,'ORIGINAL_REFERENCE',lambda *_:value)
    assert memory.resolve(scene,config,None)==value


def test_no_motion_history_or_invalid_source(setup,monkeypatch):
    memory,scene,config=setup
    def unavailable(*args):raise ValueError('motion unavailable')
    monkeypatch.setattr(c,'transport_points',unavailable)
    assert memory.resolve(scene,config,None)[0] is None
    monkeypatch.setattr(c,'source_valid',lambda _:False)
    assert memory.resolve(scene,config,None)[0] is None and not memory.anchors


def test_offline_aliases_restore_even_after_error(setup,monkeypatch):
    memory,scene,config=setup
    obj=c.CurrentPairFollow(config,allow_simulation_trial=True)
    before=[m.straight_reference for m in (c.handoff,c.entry,c.corridor)]
    def fail(*args,**kwargs):raise RuntimeError('controlled failure')
    monkeypatch.setattr(c.handoff.BypassHandoffFollow,'command',fail)
    with pytest.raises(RuntimeError):obj.command(scene,None,0.,'left',0.,.02,raw_scan=object())
    assert before==[m.straight_reference for m in (c.handoff,c.entry,c.corridor)]
