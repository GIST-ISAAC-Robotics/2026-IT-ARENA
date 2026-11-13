from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace
import copy,json
import importlib.util,sys
import numpy as np
import pytest
from arena_autonomy.follow_trace import restore_call,motion_scene,MotionConfig,FollowConfig
from arena_autonomy.bypass_entry_trial import EntryConfig
from arena_autonomy.short_entry_transition import compare_short_transition
A=Path(__file__).resolve().parents[1]/'artifacts/validation/2026-11-08/entry_preview_continuity_probe_v1'
def data(mirror=False):
    d=json.loads((A/'initial_snapshots.json').read_bytes());cap=d['captures'][0]
    cfg=FollowConfig(**d['header']['follow_config'])
    i,scan,h=restore_call(cap['input'],MotionConfig(**d['header']['motion_config']))
    scene=motion_scene(scan,h,i['reference_s'],timing_verified=i['timing_verified'],epoch=i['epoch'],rear_half_angle_deg=i['rear_half_angle_deg'])
    out=cap['output'][2]['bypass'];row=copy.deepcopy(out['corridor_plan']['rows']['right']);wall=copy.deepcopy(out['reference']['walls']['right'])
    if mirror:
        points=np.asarray(scene.occupied_endpoints_xy_m)*[1.,-1.]
        scene=SimpleNamespace(t_ref_s=scene.t_ref_s,rays=scene.rays,occupied_endpoints_xy_m=points)
        row.update(offset_m=-row['offset_m'],side='left');wall.update(slope=-wall['slope'],intercept_m=-wall['intercept_m'])
    return scene,cfg,EntryConfig(offsets_m=(0.,-.27,.27),transitions_m=(.6,.8,.9,1.2)),row,wall

@pytest.mark.parametrize('mirror',[False,True])
def test_same_current_geometry_and_mirror(mirror):
    out=compare_short_transition(*data(mirror))
    assert out['applied'] and out['transition_m']==.7 and out['permission_granted'] is False
    assert out['comparison'][0]['maximum_steering_rad']<.35

@pytest.mark.parametrize('change',[{'transition_limit_m':.6},{'transition_limit_m':1.1},{'transition_m':.6},{'offset_m':0.}])
def test_ordinary_selection_unchanged(change):
    scene,cfg,entry,row,wall=data();row.update(change)
    assert not compare_short_transition(scene,cfg,entry,row,wall)['applied']

def test_short_wall_is_not_extended():
    scene,cfg,entry,row,wall=data();wall['observed_x_m'][1]=.4
    out=compare_short_transition(scene,cfg,entry,row,wall)
    assert not out['applied'] and out['reason']=='incomplete_observed_transition'

def test_actual_steering_limit_kept():
    scene,cfg,entry,row,wall=data();cfg=replace(cfg,max_steering_rad=.2)
    assert not compare_short_transition(scene,cfg,entry,row,wall)['applied']

def test_newly_seen_intrusion_is_retained():
    scene,cfg,entry,row,wall=data()
    scene=SimpleNamespace(t_ref_s=scene.t_ref_s,
        rays=tuple(scene.rays)+(SimpleNamespace(status='return',acquired_at_s=scene.t_ref_s),),
        occupied_endpoints_xy_m=np.vstack((scene.occupied_endpoints_xy_m,[.35,-.25])))
    out=compare_short_transition(scene,cfg,entry,row,wall)
    assert not out['applied']

