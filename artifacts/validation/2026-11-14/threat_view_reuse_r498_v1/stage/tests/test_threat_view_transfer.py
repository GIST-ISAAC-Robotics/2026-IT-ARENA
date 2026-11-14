"""호출 안 전달과 실제 selector 상속/wrapper의 상태 보존. 센서 인식 성능시험 아님."""
import copy
from contextvars import Context
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
import arena_autonomy.pass_release_trial as ps
import arena_autonomy.traffic_solo_gap as gap
from arena_autonomy.current_lane_preference import CurrentLaneSelector
from arena_autonomy.race_runtime_policy import released_plan
from arena_autonomy.local_follow import FollowConfig
from test_pass_release_trial import output, view
from test_local_follow import scan


def config():
    return FollowConfig(partial_observation_policy='bounded_local_v4',
        boundary_evidence_mode='corner_distance_candidate',occupancy_timing_mode='per_ray_candidate',
        parallel_hold_trial=True,pass_release_trial=True,guard_curve_trial=True,
        traffic_recovery_trial=True,oblique_gap_trial=True)


def sample(first, missing):
    raw=scan(first,[8.]*500)
    for i in missing:raw.ranges[i]=float('inf')
    return raw


def scene(t=1.):
    return SimpleNamespace(source_first_s=t-.1, t_ref_s=t, source_epoch=0)


def capture(owner, current, base, monkeypatch):
    calls=[]
    monkeypatch.setattr(ps, 'threat_view', lambda *a: calls.append(a) or view())
    transfer=[]
    assert gap.eligible(owner,current,base,sample(.5,range(301,306)),0.,None,view_transfer=transfer)
    return transfer[0], calls


@pytest.mark.parametrize('shadow',[False,True])
def test_real_select_inheritance_and_runtime_wrapper_keep_output_and_state(monkeypatch,shadow):
    original=CurrentLaneSelector(config()); optimized=CurrentLaneSelector(config())
    current=scene();base=output(.2);parallel=output(.2)
    transfer,calls=capture(optimized,current,base,monkeypatch)
    observed=[]
    with released_plan.enabled(shadow=shadow,observer=lambda *a:observed.append(a)):
        before=original.select(current,base,parallel,.1,0.,.02)
        after=optimized.select(current,base,parallel,.1,0.,.02,threat_view_transfer=transfer)
    assert after==before
    assert vars(original)==vars(optimized)
    assert len(calls)==2  # eligible + original select; candidate consumes same computation.
    assert len(observed)==2 and observed[0][3:6]==observed[1][3:6]
    assert gap._selection_view.get() is None


@pytest.mark.parametrize('changed',['scene','path','config','path_in_place','owner'])
def test_different_input_always_recomputes(monkeypatch,changed):
    owner=ps.PassReleaseSelector(config());current=scene();base=output(.2)
    transfer,calls=capture(owner,current,base,monkeypatch)
    receiver=owner
    if changed=='scene':current=scene(1.1)
    if changed=='path':base=(.2,0.,dict(base[2],path_points=[[0.,0.],[1.,.1]]))
    if changed=='path_in_place':base[2]['path_points'][1][1]=.1
    if changed=='config':owner.lat=replace(owner.lat,min_threat_points=4)
    if changed=='owner':receiver=ps.PassReleaseSelector(config())
    with gap.selection_view(owner,transfer):
        gap.view_for_selection(receiver,current,base[2]['path_points'])
    assert len(calls)==2
    assert gap._selection_view.get() is None


def test_nested_none_exception_and_new_context_do_not_inherit_transfer(monkeypatch):
    owner=ps.PassReleaseSelector(config());current=scene();base=output(.2)
    transfer,calls=capture(owner,current,base,monkeypatch)
    with gap.selection_view(owner,transfer):
        gap.view_for_selection(owner,current,base[2]['path_points'])
        assert len(calls)==1
        with pytest.raises(RuntimeError):
            with gap.selection_view(owner,None):
                gap.view_for_selection(owner,current,base[2]['path_points'])
                raise RuntimeError('injected')
        gap.view_for_selection(owner,current,base[2]['path_points'])
        assert len(calls)==2
        Context().run(gap.view_for_selection,owner,current,base[2]['path_points'])
        assert len(calls)==3
    assert gap._selection_view.get() is None
    owner.select(current,base,base,.1,0.,.02)
    assert len(calls)==4


def test_select_exception_resets_and_equal_frozen_config_can_match(monkeypatch):
    owner=ps.PassReleaseSelector(config());current=scene();base=output(.2)
    transfer,calls=capture(owner,current,base,monkeypatch)
    owner.lat=replace(owner.lat)
    assert transfer.matches(current,base[2]['path_points'],owner.lat)
    def fail(*a,**k):
        assert gap.view_for_selection(owner,current,base[2]['path_points']) is transfer.view
        raise RuntimeError('selector failure')
    monkeypatch.setattr(owner,'_select',fail)
    with pytest.raises(RuntimeError,match='selector failure'):
        owner.select(current,base,base,.1,0.,.02,threat_view_transfer=transfer)
    assert len(calls)==1 and gap._selection_view.get() is None
