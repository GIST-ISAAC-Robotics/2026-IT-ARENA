"""통과 후보에도 기존 단일 장착 사각·실제 저속·국소 폭 제한을 유지한다."""
from dataclasses import replace
import json
from pathlib import Path
import numpy as np
import pytest
from arena_autonomy import released_body_gap as gap, pass_release_trial as ps
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.scene_motion import motion_scene

FIXTURE=Path(__file__).resolve().parents[1]/'artifacts/validation/2026-11-12/adjacent_mount_gap_r149_v1/fixture.json'

def execute(monkeypatch,*,speed=0.,target=.15,mode='structure_aware_v1',sectors=None,modify=None):
    f=json.loads(FIXTURE.read_bytes())
    if modify:modify(f['input']['scan']['measurements'])
    owner,motion=make_controller(f['header'])
    cfg=replace(owner.config,guard_observation_policy='selected_r181',released_body_gap_trial=True,
                known_structure_policy=mode,
                known_structure_sectors_json='' if mode=='off' else owner.config.known_structure_sectors_json)
    if sectors is not None:cfg=replace(cfg,known_structure_sectors_json=sectors)
    item,scan,history=restore_call(f['input'],motion)
    scene=motion_scene(scan,history,item['reference_s'],timing_verified=item['timing_verified'],
                       epoch=item['epoch'],rear_half_angle_deg=item['rear_half_angle_deg'])
    queries=np.asarray(f['geometry']['queries']);padding=np.asarray(f['geometry']['padding'])
    def region(*args,context,**kwargs):
        context.update(queries=queries,padding=padding,free=np.zeros(len(queries),dtype=bool),
                       masked=np.zeros(len(queries),dtype=bool))
        return dict(unobserved=len(queries),masked_left=0,masked_right=0,masked_front_samples=0)
    monkeypatch.setattr(ps,'motion_region',region)
    return gap.check(scene,scan,speed,target,0.,0.,.02,cfg,ps.lateral_config(cfg),ps.SelectorConfig())

def test_same_declared_mount_gap_is_bounded_but_stays_unknown(monkeypatch):
    result=execute(monkeypatch)
    assert result['accepted'] and not result['unknown_filled']
    assert result['proof']['reason']=='bounded_adjacent_mount_gap_risk'
    assert result['proof']['maximum_chord_m']<=.05
    assert result['proof']['gaps'][0]['masked_indices']

@pytest.mark.parametrize('speed,target',[(.151,.15),(.2,.15),(0.,.151),(0.,.35)])
def test_mount_gap_does_not_borrow_the_looser_plain_gap_speed(monkeypatch,speed,target):
    assert not execute(monkeypatch,speed=speed,target=target)['accepted']

@pytest.mark.parametrize('field,value',[(89,{'nonfinite':'nan'}),(89,{'nonfinite':'-inf'}),(89,.01),(89,.15),(87,.08),(91,.2)])
def test_bad_or_near_returns_are_not_excused(monkeypatch,field,value):
    assert not execute(monkeypatch,modify=lambda m:m['ranges_m'].__setitem__(field,value))['accepted']

def test_unselected_structure_policy_does_not_use_declared_shadow(monkeypatch):
    assert not execute(monkeypatch,mode='off')['accepted']

def test_unrelated_declared_mount_does_not_match(monkeypatch):
    assert not execute(monkeypatch,sectors='[[180.0,30.0]]')['accepted']
