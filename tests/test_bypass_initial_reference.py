"""후보의 시각·현재 관측 경계 검사."""
from types import SimpleNamespace as NS
import numpy as np
import pytest
import arena_autonomy.bypass_initial_reference as c
from arena_autonomy.bypass_handoff import handoff_baseline


@pytest.fixture
def setup(monkeypatch):
    memory=c.CurrentPairReference();config=handoff_baseline()
    scene=NS(t_ref_s=10.14,source_first_s=10.,source_frame_id='laser_frame',source_epoch=0)
    original=(0.,0.,-.15,1.5)
    walls={side:dict(slope=0.,intercept_m=0.,observed_x_m=[-.15,1.5]) for side in ('left','right')}
    monkeypatch.setattr(c,'source_valid',lambda _:True)
    monkeypatch.setattr(c,'straight_reference',lambda *_:(original,dict(walls=walls)))
    monkeypatch.setattr(c,'transport_points',lambda points,*_:points)
    memory.resolve(scene,config,None)
    monkeypatch.setattr(c,'straight_reference',lambda *_:(None,dict(reason='straight_entry_only')))
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
    monkeypatch.setattr(c,'straight_reference',lambda *_:value)
    assert memory.resolve(scene,config,None)==value


def test_no_motion_history_or_invalid_source(setup,monkeypatch):
    memory,scene,config=setup
    def unavailable(*args):raise ValueError('motion unavailable')
    monkeypatch.setattr(c,'transport_points',unavailable)
    assert memory.resolve(scene,config,None)[0] is None
    monkeypatch.setattr(c,'source_valid',lambda _:False)
    assert memory.resolve(scene,config,None)[0] is None and not memory.anchors


def subject():
    from dataclasses import replace
    from arena_autonomy.bypass_handoff import BypassHandoffFollow
    from test_bypass_handoff_corridor_trial import subject as original
    old=original()
    return BypassHandoffFollow(old.config,replace(old.execution,initial_reference_policy='ready_current_pair_v1'),
        allow_simulation_trial=True)


def test_policy_default_and_legacy_trace_are_off():
    from dataclasses import asdict,replace
    from arena_autonomy.bypass_handoff import HandoffExecutionConfig
    cfg=HandoffExecutionConfig();old=asdict(cfg);old.pop('initial_reference_policy')
    assert HandoffExecutionConfig.from_mapping(old).initial_reference_policy=='off'
    with pytest.raises(ValueError):replace(cfg,initial_reference_policy='ready_current_pair_v1')
    assert subject().bypass.guide.reference_provider is not None


@pytest.mark.parametrize('value',[None,True,1,'','unlimited','current_pair'])
def test_unregistered_policy_fails(value):
    from dataclasses import replace
    with pytest.raises(ValueError):replace(subject().execution,initial_reference_policy=value)


@pytest.mark.parametrize('ready,observed,moving',[(True,True,False),(False,True,False),
    (True,False,False),(True,True,True)])
def test_reference_provider_requires_current_ready_state(monkeypatch,ready,observed,moving):
    helper=c.ReadyInitialReference();scene=object();config=object()
    original=(None,dict(reason='straight_entry_only'))
    proposed=((0.,0.,-.15,1.5),dict(reason='current_tracked_pair_candidate'))
    monkeypatch.setattr(c,'straight_reference',lambda *_:original)
    monkeypatch.setattr(helper.memory,'resolve',lambda *_,**kwargs:proposed)
    helper.update(scene,config,None)
    monitor=NS(result=dict(ready=ready,observed=observed),motion_evidence=lambda:moving)
    expected=proposed if ready and observed and not moving else original
    assert helper.reference(scene,config,monitor)==expected
    assert helper.reference(object(),config,monitor)==original
    assert helper.reference(scene,object(),monitor)==original
    helper.clear()
    assert helper.reference(scene,config,monitor)==original


@pytest.mark.parametrize('directional',[False,True])
def test_original_fit_is_shared_within_call_but_recomputed_next_call(monkeypatch,directional):
    from unittest.mock import Mock
    from test_bypass_handoff import context
    original=c.straight_reference
    observed=Mock(wraps=original)
    monkeypatch.setattr(c,'straight_reference',observed)
    helper=c.ReadyInitialReference(directional_coverage=directional)
    config=handoff_baseline()
    for index,now in enumerate((.5,.52,.6),1):
        _,history,scene=context(now)
        helper.update(scene,config,history)
        assert observed.call_count==index
        assert helper.original==original(scene,config)
        assert helper.scene is scene and helper.config is config


@pytest.mark.parametrize('speed,raw_valid',[(None,True),(float('nan'),True),(0.,False),(-.05,True)])
def test_fault_or_invalid_input_clears_memory(speed,raw_valid):
    from test_bypass_handoff import context
    ctl=subject();scan,history,scene=context()
    ctl._initial_reference.memory.anchors={'left':object()}
    out=ctl.command(scene,history,speed,'left',0.,.02,raw_scan=scan if raw_valid else None)
    assert out[0]==0. and not ctl._initial_reference.memory.anchors
    assert ctl.bypass.guide.reference_provider is not None


def test_node_records_policy_and_replays(tmp_path):
    import json
    from arena_autonomy.follow_trace import replay_events
    from test_bypass_handoff import context,trace_node
    node=trace_node(tmp_path,g1_g2_preparation_policy='observed_entry_space_v1',
        g1_g2_persistence_window='trim_before_compare_v1',g1_g2_follow_entry_space=True,
        g1_g2_reacquire_policy='known_hold_v1',g1_g2_entry_policy='corridor_local_v1',
        g1_g2_corridor_policy='corridor_v2_signed_hold',g1_g2_initial_reference='ready_current_pair_v1')
    path=tmp_path/'initial_reference.jsonl';trace=node.create_follow_trace(str(path))
    for now in (.5,.6,.7):
        scan,history,scene=context(now)
        out=node.follow.command(scene,history,0.,'left',0.,.02,raw_scan=scan)
        trace.record_call(scan,history,now,0,0.,'left',0.,.02,True,30.,out)
    trace.close();events=[json.loads(line) for line in path.read_text().splitlines()]
    assert events[0]['data']['execution_config']['initial_reference_policy']=='ready_current_pair_v1'
    assert replay_events(events)['passed']



def test_first_entry_comparison_uses_same_reference_provider(monkeypatch):
    import arena_autonomy.bypass_entry_trial as entry
    from test_bypass_handoff import context
    ctl=subject();guide=ctl.bypass.guide
    _,history,scene=context()
    provider=guide.reference_provider;seen=[]
    monkeypatch.setattr(guide,'_confirm_initial_boundaries',lambda *_:(True,{}))
    def compare(*args,**kwargs):
        seen.append(kwargs.get('reference_provider'))
        return dict(reason='probe_first_entry_provider',selected=None,candidates=[])
    monkeypatch.setattr(entry,'_compare_entries',compare)
    monkeypatch.setattr(guide.intent,'select',lambda scene,result:result)
    result=guide.evaluate(scene,history,0.,0.,.02)
    assert seen==[provider] and provider is not None
    assert result['guide_mode']=='new_observed_entry'
