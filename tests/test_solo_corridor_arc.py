"""단독 코너 시간 비교의 호길이 보완. 점유·확인 횟수·기본 교통 동작은 유지한다."""
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest

from arena_autonomy import corridor_observer as module
from arena_autonomy.local_follow import FollowConfig,path_agrees,path_agrees_arc
from test_low_speed_arc_reacquire import SOLO,corridor,launch_module,play_scene
from test_local_follow import history

FIXTURE=json.loads((Path(__file__).parent/'fixtures/solo_corridor_arc_paths.json').read_text(encoding='utf-8'))['cases']


@pytest.mark.parametrize('case',FIXTURE,ids=lambda c:str(c['t']))
def test_recorded_corner_pairs_share_arc_but_not_forward_x(case):
    first,second=np.array(case['current']),np.array(case['prior'])
    assert not path_agrees(first,second,.08)
    assert path_agrees_arc(first,second,.08)
    assert not path_agrees_arc(first,second+[0.,.30],.08)
    assert not path_agrees_arc(first,first[::-1],.08)


def config(enabled):
    return FollowConfig(**SOLO,solo_route_only=True,corridor_arc_agreement=enabled)


@pytest.mark.parametrize('changes',[{'solo_route_only':False},{'corridor_fallback_enabled':False},
                                    {'corridor_arc_agreement':1},{'corridor_arc_agreement':'true'}])
def test_arc_option_is_explicit_and_solo_only(changes):
    with pytest.raises(ValueError):replace(config(True),**changes)
    assert FollowConfig().corridor_arc_agreement is False


def observed(monkeypatch,enabled):
    path=np.array(FIXTURE[-1]['current'])
    assert not path_agrees(path,path,.08)
    control={'supported':True,'occupied':True,'path':path}
    def proposal(*args,**kwargs):
        return dict(geometry_supported=control['supported'],point_occupancy_supported=control['occupied'],
                    reason='geometry_and_point_occupancy_only' if control['occupied'] else 'existing_occupancy_margin_hold',
                    geometry={'path':control['path'],'supported':control['supported']},
                    proposed_speed_mps=.4 if control['occupied'] else 0.,steering_rad=.2,swept_clearance_m=.5)
    monkeypatch.setattr(module,'current_proposal',proposal)
    observer=module.CorridorObserver(config(enabled))
    def step(t,source):
        scene=SimpleNamespace(t_ref_s=t,source_first_s=source,source_frame_id='laser_frame',source_epoch=0)
        return observer.evaluate(scene,history(),0.,'left',.2,.02)
    return control,observer,step


def test_identical_bent_path_survives_reused_scan_without_counting_new_evidence(monkeypatch):
    _,observer,step=observed(monkeypatch,True)
    assert not step(.4,.3)['positive']
    again=step(.42,.3)
    assert again['confirmations']==1 and not again['fresh'] and not again['positive']
    assert again['path_agreement_method']=='common_arc'
    second=step(.5,.4)
    assert second['confirmations']==2 and second['release_count']==1 and not second['positive']
    third=step(.6,.5)
    assert third['positive'] and third['release_count']==2
    assert not third['driving_authorized'] and not third['memory_refresh_authorized']


def test_option_off_retains_original_self_disagreement(monkeypatch):
    _,observer,step=observed(monkeypatch,False)
    step(.4,.3)
    result=step(.42,.3)
    assert result['reason']=='reused_scan_path_unconfirmed' and observer.path is None
    assert 'path_agreement_method' not in result


@pytest.mark.parametrize('condition',['occupied','geometry','different_path','expired'])
def test_arc_agreement_does_not_replace_current_geometry_occupancy_or_ttl(monkeypatch,condition):
    state,observer,step=observed(monkeypatch,True)
    for t in (.4,.5,.6):step(t,t-.1)
    if condition=='occupied':state['occupied']=False
    elif condition=='geometry':state['supported']=False
    elif condition=='different_path':state['path']=state['path']+[0.,.3]
    elif condition=='expired':observer.path_source=.1
    result=step(.7,.6)
    assert not result['positive']
    if condition=='expired':assert not result.get('previous_path_agrees')


@pytest.mark.parametrize('front',[.22,.30,.38])
def test_linked_solo_arc_keeps_visible_front_obstacle_stop(front):
    rows=play_scene(corridor(((front,-.425,front,.425),)),True,
                    config_changes={'solo_route_only':True,'headway_s':.5,'corridor_arc_agreement':True})
    assert all(row.v==0 for row in rows)


def test_linked_solo_arc_does_not_fill_missing_front():
    rows=play_scene(corridor(),True,blind=((-.3,.3),),
                    config_changes={'solo_route_only':True,'headway_s':.5,'corridor_arc_agreement':True})
    assert all(row.v==0 for row in rows)


def test_profile_adds_only_the_arc_comparison():
    profiles=launch_module().SPEED_PROFILES
    assert profiles['g1_open_10kmh_solo_gap05_arc']=={
        **profiles['g1_open_10kmh_solo_gap05'],'g1_corridor_arc_agreement':True}
