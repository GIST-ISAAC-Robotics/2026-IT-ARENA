from pathlib import Path
from dataclasses import replace
import copy,json
import numpy as np
import pytest
from arena_autonomy.follow_trace import restore_call,motion_scene,MotionConfig,FollowConfig
from arena_autonomy.bypass_entry_trial import EntryConfig,straight_reference
from arena_autonomy.adaptive_corridor_offset import AdaptiveCorridorEntryConfig,compare_roomy_path
A=Path(__file__).resolve().parents[1]/'artifacts/validation/2026-11-08/entry_width_transition_probe_v2'

def data(mirror=False):
    d=json.loads((A/'initial.json').read_bytes());cap=d['call']
    cfg=FollowConfig(**d['header']['follow_config'])
    i,scan,h=restore_call(cap['input'],MotionConfig(**d['header']['motion_config']))
    scene=motion_scene(scan,h,i['reference_s'],timing_verified=i['timing_verified'],epoch=i['epoch'],rear_half_angle_deg=i['rear_half_angle_deg'])
    _,evidence=straight_reference(scene,cfg)
    row=copy.deepcopy(cap['output'][2]['bypass']['corridor_plan']['rows']['right'])
    wall=copy.deepcopy(evidence['walls']['right'])
    if mirror:
        flip=lambda p:None if p is None else (p[0],-p[1])
        original=scene.rays
        scene=replace(scene,rays=tuple(replace(r,ray_origin_xy_m=flip(r.ray_origin_xy_m),
            hit_xy_m=flip(r.hit_xy_m),direction_xy=flip(r.direction_xy),source_index=k,
            acquired_at_s=original[k].acquired_at_s) for k,r in enumerate(reversed(original))))
        row.update(offset_m=-row['offset_m'],want_offset_m=-row['want_offset_m'],side='left')
        wall.update(slope=-wall['slope'],intercept_m=-wall['intercept_m'])
        if 'source_ray_indices' in wall:
            wall['source_ray_indices']=[len(original)-1-k for k in wall['source_ray_indices']]
    return scene,cfg,AdaptiveCorridorEntryConfig(transitions_m=(.6,.8,.9,1.2)),row,wall,.08

@pytest.mark.parametrize('mirror',[False,True])
def test_observed_current_and_reused_geometry_gain(mirror):
    args=data(mirror);scene=args[0];before=tuple(scene.rays);out=compare_roomy_path(*args)
    assert out['applied'] and .27<abs(out['offset_m'])<=.305
    if not mirror:assert abs(out['offset_m'])==pytest.approx(.2875)
    assert not out['permission_granted'] and not out['unknown_filled']
    assert tuple(scene.rays)==before and out['existing_fixed_outer_returns']>100
    selected=next(r for r in out['comparison'] if r['offset_m']==out['offset_m'])
    assert selected['reused_progress_m']>=out['comparison'][0]['reused_progress_m']
    assert selected['current_gap_m']>0

@pytest.mark.parametrize('change',[{'transition_limit_m':.6},{'offset_m':0.},{'want_offset_m':-.2}])
def test_ordinary_or_no_need_is_unchanged(change):
    args=list(data());args[3].update(change);assert not compare_roomy_path(*args)['applied']

def test_legacy_geometry_limit_stays_27cm():
    args=list(data());args[2]=EntryConfig(transitions_m=(.6,.8,.9,1.2))
    assert not compare_roomy_path(*args)['applied']
    with pytest.raises(ValueError):EntryConfig(offsets_m=(0.,.3))
    with pytest.raises(ValueError):AdaptiveCorridorEntryConfig(offsets_m=(0.,.351))

def test_steering_and_observed_end_are_required():
    args=list(data());args[1]=replace(args[1],max_steering_rad=.32)
    assert not compare_roomy_path(*args)['applied']
    args=list(data());args[4]['observed_x_m'][1]=.4
    assert compare_roomy_path(*args)['reason']=='incomplete_observed_transition'

def test_road_half_width_caps_candidate_center():
    args=list(data());args[1]=replace(args[1],road_half_m=.27)
    assert not compare_roomy_path(*args)['applied']

def test_current_intrusion_cannot_be_removed_by_outer_wall_assumption():
    args=list(data());scene=args[0]
    rays=list(scene.rays);index=next(i for i,r in enumerate(rays) if r.status=='return' and r.direction_xy[0]>.99)
    rays[index]=replace(rays[index],hit_xy_m=(.08,0.))
    args[0]=replace(scene,rays=tuple(rays));out=compare_roomy_path(*args)
    assert not out['applied']
    assert args[0].rays[index].hit_xy_m==(.08,0.)

@pytest.mark.parametrize('reuse',[-.01,.11])
def test_unsupported_reuse_is_not_extrapolated(reuse):
    args=list(data());args[-1]=reuse;assert not compare_roomy_path(*args)['applied']

@pytest.mark.parametrize('enabled',[False,True])
def test_wider_geometry_type_is_only_in_explicit_adaptive_trial(enabled):
    from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
    d=json.loads((A/'initial.json').read_bytes());cfg=FollowConfig(**d['header']['follow_config'])
    execution=replace(HandoffExecutionConfig.from_mapping(d['header']['execution_config']),adaptive_entry_preview=enabled)
    controller=BypassHandoffFollow(cfg,execution,allow_simulation_trial=True)
    entry=controller.bypass.guide.registered_entry
    assert isinstance(entry,AdaptiveCorridorEntryConfig)==enabled
    assert entry.maximum_offset_m==(.35 if enabled else .27)
    assert max(abs(v) for v in entry.offsets_m)==.27
    assert entry.speed_cap_mps==.35
