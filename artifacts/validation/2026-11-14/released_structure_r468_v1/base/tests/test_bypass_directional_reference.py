import pytest
from arena_autonomy.bypass_initial_reference import DirectionalPairReference
from arena_autonomy.bypass_initial_reference import CurrentPairReference
from arena_autonomy.bypass_handoff import handoff_baseline
from arena_autonomy.bypass_entry_trial import entry_path, EntryConfig


def walls(left=(-.05, 1.5), right=(-.4, 1.8), shift=0.):
    return {side:dict(slope=0., intercept_m=shift if side=='left' else 0.,
        observed_x_m=list(span)) for side, span in [('left', left), ('right', right)]}


def invoke(monkeypatch, boundary, **extra):
    detail=dict(reason='reference_does_not_cover_entry', walls=boundary,
        all_returns_remain_occupied=True, remembered_line_is_current_observation=False)
    detail.update(extra)
    monkeypatch.setattr(CurrentPairReference, 'resolve', lambda *_:(None, detail))
    return DirectionalPairReference().resolve(None, handoff_baseline(), None)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_only_observed_outer_side_can_generate_entry(monkeypatch, side):
    boundary=walls()
    if side=='left':boundary['left'],boundary['right']=boundary['right'],boundary['left']
    reference, detail=invoke(monkeypatch, boundary)
    assert reference[2]==-.05  # 공통 범위를 뒤로 연장하지 않는다.
    assert detail['entry_covering_sides']==[side]
    assert not detail['unknown_filled'] and detail['all_returns_remain_occupied']
    cfg=handoff_baseline()
    for name, wall in boundary.items():
        fit=(wall['slope'],wall['intercept_m'],*wall['observed_x_m'])
        if name==side:
            path=entry_path(fit,.16 if side=='left' else -.16,cfg,EntryConfig())
            assert path[0,0]==-cfg.wheelbase_m/2
            assert path[-1,0]<=wall['observed_x_m'][1]
        else:
            with pytest.raises(ValueError,match='prefix too short'):
                entry_path(fit,.16,cfg,EntryConfig())


@pytest.mark.parametrize('boundary', [walls(left=(0.,1.5),right=(0.,1.5)),
    walls(left=(1.3,1.5)), walls(left=(2.,3.)), walls(left=(-.05,.24),right=(-.4,.24)),
    walls(shift=.5)])
def test_missing_coverage_overlap_or_agreement_stays_unavailable(monkeypatch,boundary):
    assert invoke(monkeypatch,boundary)[0] is None


@pytest.mark.parametrize('detail', [dict(reason='current_pair_unavailable'),
    dict(reason='seed_transport_unavailable'),dict(all_returns_remain_occupied=False)])
def test_parent_rejections_not_overridden(monkeypatch,detail):
    assert invoke(monkeypatch,walls(),**detail)[0] is None


def test_valid_original_is_identical(monkeypatch):
    result=((0.,0.,-.15,1.5),dict(reason='current_two_wall_straight_reference'))
    monkeypatch.setattr(CurrentPairReference,'resolve',lambda *_:result)
    assert DirectionalPairReference().resolve(None,None,None) == result


def test_runtime_selection_remains_explicit_and_legacy_is_unchanged():
    from dataclasses import replace
    from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
    from test_bypass_initial_reference import subject
    old=subject()
    assert type(old._initial_reference.memory) is CurrentPairReference
    assert HandoffExecutionConfig().initial_reference_policy=='off'
    execution=replace(old.execution,initial_reference_policy='ready_directional_pair_v2')
    new=BypassHandoffFollow(old.config,execution,allow_simulation_trial=True)
    assert type(new._initial_reference.memory) is DirectionalPairReference
    assert new.bypass.guide.reference_provider is not None
    with pytest.raises(ValueError):
        replace(HandoffExecutionConfig(),initial_reference_policy='ready_directional_pair_v2')


@pytest.mark.parametrize('speed,raw_valid',[(None,True),(float('nan'),True),(0.,False),(-.05,True)])
def test_directional_memory_is_not_kept_across_fault(speed,raw_valid):
    from dataclasses import replace
    from arena_autonomy.bypass_handoff import BypassHandoffFollow
    from test_bypass_initial_reference import subject
    from test_bypass_handoff import context
    old=subject()
    ctl=BypassHandoffFollow(old.config,replace(old.execution,
        initial_reference_policy='ready_directional_pair_v2'),allow_simulation_trial=True)
    scan,history,scene=context()
    ctl._initial_reference.memory.anchors={'left':object()}
    out=ctl.command(scene,history,speed,'left',0.,.02,raw_scan=scan if raw_valid else None)
    assert out[0]==0. and not ctl._initial_reference.memory.anchors


def test_directional_node_trace_replays(tmp_path):
    import json
    from arena_autonomy.follow_trace import replay_events
    from test_bypass_handoff import context,trace_node
    node=trace_node(tmp_path,g1_g2_preparation_policy='observed_entry_space_v1',
        g1_g2_persistence_window='trim_before_compare_v1',g1_g2_follow_entry_space=True,
        g1_g2_reacquire_policy='known_hold_v1',g1_g2_entry_policy='corridor_local_v1',
        g1_g2_corridor_policy='corridor_v2_signed_hold',g1_g2_initial_reference='ready_directional_pair_v2')
    path=tmp_path/'directional_reference.jsonl';trace=node.create_follow_trace(str(path))
    for now in (.5,.6,.7):
        scan,history,scene=context(now)
        out=node.follow.command(scene,history,0.,'left',0.,.02,raw_scan=scan)
        trace.record_call(scan,history,now,0,0.,'left',0.,.02,True,30.,out)
    trace.close();events=[json.loads(line) for line in path.read_text().splitlines()]
    assert events[0]['data']['execution_config']['initial_reference_policy']=='ready_directional_pair_v2'
    assert replay_events(events)['passed']
